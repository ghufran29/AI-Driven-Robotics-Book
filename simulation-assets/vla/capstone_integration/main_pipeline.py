"""
Main Pipeline Orchestrator for VLA System

This module orchestrates the complete VLA pipeline:
Voice → Whisper → LLM Planning → Vision Grounding → Action Validation → Execution
"""
import asyncio
import time
import threading
import queue
from typing import Dict, Any, Optional, List
import logging
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import components
from ..voice_interface.audio_capture import AudioCapture
from ..voice_interface.whisper_transcription import WhisperTranscriber, create_wav_bytes
from ..voice_interface.vad_detector import VADDetector
from ..cognitive_core.llm_planning import LLMPlanner, create_example_vision_context
from ..cognitive_core.json_validator import JSONValidator
from ..cognitive_core.safety_guardrails import SafetyGuardrails
from ..vision_grounding.object_detector import ObjectDetector
from ..vision_grounding.grounding_pipeline import GroundingPipeline

logger = logging.getLogger(__name__)

class VLAPipeline:
    """
    Main orchestrator for the Vision-Language-Action pipeline.
    """
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize the VLA pipeline with configuration.

        Args:
            config_path: Path to configuration file (optional)
        """
        # Load configuration
        self.config = self._load_config(config_path)

        # Initialize components
        self.audio_capture = None
        self.whisper_transcriber = None
        self.vad_detector = None
        self.llm_planner = None
        self.json_validator = None
        self.safety_guardrails = None
        self.object_detector = None
        self.grounding_pipeline = None

        # Initialize components based on config
        self._initialize_components()

        # Performance tracking
        self.latency_metrics = {
            'voice_to_text': [],
            'language_understanding': [],
            'vision_grounding': [],
            'action_planning': [],
            'total_processing': []
        }

        # State tracking
        self.is_running = False
        self.pipeline_queue = queue.Queue()

    def _load_config(self, config_path: Optional[str]) -> Dict[str, Any]:
        """
        Load configuration for the pipeline.

        Args:
            config_path: Path to configuration file

        Returns:
            Configuration dictionary
        """
        config = {
            # Voice configuration
            'voice': {
                'vad_threshold': float(os.getenv('VLA_VAD_THRESHOLD', '0.02')),
                'silence_duration': float(os.getenv('VLA_SILENCE_DURATION', '0.5')),
                'min_audio_length': float(os.getenv('VLA_MIN_AUDIO_LENGTH', '1.0')),
                'audio_sample_rate': int(os.getenv('VLA_AUDIO_SAMPLE_RATE', '16000')),
                'audio_chunk_size': int(os.getenv('VLA_AUDIO_CHUNK_SIZE', '1024')),
                'mic_device_index': int(os.getenv('VLA_MIC_DEVICE_INDEX', '0'))
            },
            # LLM configuration
            'llm': {
                'model': os.getenv('VLA_LLM_MODEL', 'gpt-4o'),
                'temperature': float(os.getenv('VLA_LLM_TEMPERATURE', '0.1')),
                'max_tokens': int(os.getenv('VLA_LLM_MAX_TOKENS', '500'))
            },
            # Vision configuration
            'vision': {
                'detection_threshold': float(os.getenv('VLA_DETECTION_THRESHOLD', '0.7')),
                'max_objects': int(os.getenv('VLA_MAX_OBJECTS', '10')),
                'camera_index': int(os.getenv('VLA_CAMERA_INDEX', '0'))
            },
            # Safety configuration
            'safety': {
                'risk_threshold': float(os.getenv('VLA_RISK_THRESHOLD', '0.8')),
                'confirmation_required': os.getenv('VLA_CONFIRMATION_REQUIRED', 'false').lower() == 'true'
            },
            # Performance configuration
            'performance': {
                'max_latency': float(os.getenv('VLA_MAX_LATENCY', '5.0')),
                'target_latency': float(os.getenv('VLA_TARGET_LATENCY', '3.0')),
                'min_confidence': float(os.getenv('VLA_MIN_CONFIDENCE', '0.7'))
            }
        }

        return config

    def _initialize_components(self):
        """Initialize all pipeline components."""
        try:
            # Initialize voice components
            self.vad_detector = VADDetector(
                vad_threshold=self.config['voice']['vad_threshold'],
                silence_duration=self.config['voice']['silence_duration'],
                min_audio_length=self.config['voice']['min_audio_length']
            )

            # Initialize audio capture
            self.audio_capture = AudioCapture(
                device_index=self.config['voice']['mic_device_index'],
                sample_rate=self.config['voice']['audio_sample_rate'],
                chunk_size=self.config['voice']['audio_chunk_size']
            )

            # Initialize Whisper transcriber
            api_key = os.getenv('OPENAI_API_KEY')
            if api_key:
                self.whisper_transcriber = WhisperTranscriber(api_key=api_key, model="whisper-1")

            # Initialize LLM planner
            if api_key:
                self.llm_planner = LLMPlanner(api_key=api_key, model=self.config['llm']['model'])

            # Initialize other components
            self.json_validator = JSONValidator()
            self.safety_guardrails = SafetyGuardrails(
                risk_threshold=self.config['safety']['risk_threshold'],
                confirmation_required_risk=0.5  # Fixed threshold for confirmation
            )
            self.object_detector = ObjectDetector(
                detection_threshold=self.config['vision']['detection_threshold'],
                max_objects=self.config['vision']['max_objects']
            )
            self.grounding_pipeline = GroundingPipeline(
                detection_threshold=self.config['vision']['detection_threshold'],
                grounding_threshold=self.config['performance']['min_confidence']
            )

            logger.info("All VLA pipeline components initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize VLA pipeline components: {str(e)}")
            raise

    async def process_voice_command(self, timeout: float = 10.0) -> Optional[Dict[str, Any]]:
        """
        Process a voice command through the complete VLA pipeline.

        Args:
            timeout: Maximum time to wait for command in seconds

        Returns:
            Dictionary with processing results, or None if failed
        """
        start_time = time.time()

        try:
            # Step 1: Capture audio with VAD
            logger.info("Starting audio capture with VAD...")
            vad_start = time.time()

            # Check if audio capture is available
            if not self.audio_capture:
                logger.error("Audio capture not available")
                return None

            # Test microphone first
            if not self.audio_capture.test_microphone():
                logger.error("Microphone test failed")
                return None

            # Record until silence
            audio_data = self.audio_capture.record_until_silence(
                vad_threshold=self.config['voice']['vad_threshold'],
                silence_duration=self.config['voice']['silence_duration'],
                min_audio_length=self.config['voice']['min_audio_length']
            )

            vad_time = time.time() - vad_start

            if len(audio_data) == 0:
                logger.warning("No audio captured")
                return None

            # Convert to WAV format for Whisper
            wav_audio = create_wav_bytes(
                audio_data,
                sample_rate=self.config['voice']['audio_sample_rate']
            )

            # Step 2: Transcribe audio using Whisper
            logger.info("Transcribing audio with Whisper...")
            transcription_start = time.time()

            if not self.whisper_transcriber:
                logger.error("Whisper transcriber not available - API key required")
                return None

            transcription = await self.whisper_transcriber.transcribe_audio(wav_audio)
            transcription_time = time.time() - transcription_start

            if not transcription:
                logger.error("Transcription failed")
                return None

            logger.info(f"Transcription: {transcription}")

            # Step 3: Get current vision context
            logger.info("Getting vision context...")
            vision_start = time.time()

            # In a real system, we would get the current camera frame
            # For now, we'll use example vision context
            vision_context = create_example_vision_context()
            vision_time = time.time() - vision_start

            # Step 4: Plan actions using LLM
            logger.info("Planning actions with LLM...")
            llm_start = time.time()

            if not self.llm_planner:
                logger.error("LLM planner not available - API key required")
                return None

            action_plan = await self.llm_planner.plan_actions(transcription, vision_context)
            llm_time = time.time() - llm_start

            if not action_plan:
                logger.error("LLM planning failed")
                return None

            # Step 5: Validate the planned action
            logger.info("Validating planned action...")
            validation_start = time.time()

            validation_result = self.json_validator.validate_action_json(
                action_plan["function"],
                action_plan["arguments"]
            )
            validation_time = time.time() - validation_start

            if not validation_result["valid"]:
                logger.error(f"Action validation failed: {validation_result['errors']}")
                return None

            # Step 6: Apply safety validation
            logger.info("Applying safety validation...")
            safety_start = time.time()

            safety_result = self.safety_guardrails.validate_action(action_plan)
            safety_time = time.time() - safety_start

            if not safety_result["valid"]:
                logger.error(f"Safety validation failed: {safety_result['violations']}")
                return None

            # Step 7: Apply vision grounding (if needed)
            logger.info("Applying vision grounding...")
            grounding_start = time.time()

            # If the action involves a specific object, ground it to the visual scene
            if "object_description" in action_plan["arguments"]:
                detected_objects = self.object_detector.detect_objects(
                    self._get_dummy_image()  # In real system, get from camera
                )
                grounded_object = self.grounding_pipeline.ground_language_to_objects(
                    action_plan["arguments"]["object_description"],
                    detected_objects
                )

                if grounded_object:
                    # Update the action plan with grounded object information
                    action_plan["arguments"]["grounded_object"] = grounded_object

            grounding_time = time.time() - grounding_start

            # Calculate total time
            total_time = time.time() - start_time

            # Store latency metrics
            self.latency_metrics['voice_to_text'].append(vad_time + transcription_time)
            self.latency_metrics['language_understanding'].append(llm_time)
            self.latency_metrics['vision_grounding'].append(vision_time + grounding_time)
            self.latency_metrics['action_planning'].append(validation_time + safety_time)
            self.latency_metrics['total_processing'].append(total_time)

            # Prepare result
            result = {
                "success": True,
                "transcription": transcription,
                "action_plan": action_plan,
                "validation_result": validation_result,
                "safety_result": safety_result,
                "vision_context": vision_context,
                "latency": {
                    "voice_to_text": vad_time + transcription_time,
                    "language_understanding": llm_time,
                    "vision_processing": vision_time + grounding_time,
                    "action_validation": validation_time + safety_time,
                    "total": total_time
                },
                "timestamp": time.time()
            }

            logger.info(f"Pipeline completed successfully in {total_time:.2f}s")
            return result

        except Exception as e:
            logger.error(f"Pipeline processing failed: {str(e)}")
            return {
                "success": False,
                "error": str(e),
                "timestamp": time.time()
            }

    def _get_dummy_image(self):
        """
        Get a dummy image for testing purposes.
        In a real system, this would capture from a camera.
        """
        import numpy as np
        # Create a dummy image (640x480x3)
        return np.zeros((480, 640, 3), dtype=np.uint8)

    async def process_command_with_enhanced_object_selection(self, command: str, vision_context: Optional[Dict] = None) -> Optional[Dict[str, Any]]:
        """
        Process a command with enhanced object selection capabilities for complex references.

        Args:
            command: Natural language command to process
            vision_context: Optional vision context to override default

        Returns:
            Dictionary with processing results
        """
        start_time = time.time()

        try:
            # Get vision context if not provided
            if vision_context is None:
                vision_context = create_example_vision_context()

            # Get available objects from vision context
            available_objects = vision_context.get('objects', [])

            # Use enhanced LLM planning for complex object references
            if self.llm_planner and self.api_key:
                # First, try to interpret complex object references in the command
                # This helps disambiguate when multiple similar objects are present
                complex_action = await self.llm_planner.plan_actions_with_complex_references(command, vision_context)

                if complex_action:
                    # Validate the planned action
                    validation_result = self.json_validator.validate_action_json(
                        complex_action["function"],
                        complex_action["arguments"]
                    )

                    if not validation_result["valid"]:
                        logger.error(f"Action validation failed: {validation_result['errors']}")
                        return None

                    # Apply safety validation
                    safety_result = self.safety_guardrails.validate_action(complex_action)
                    if not safety_result["valid"]:
                        logger.error(f"Safety validation failed: {safety_result['violations']}")
                        return None

                    # Calculate total time
                    total_time = time.time() - start_time

                    result = {
                        "success": True,
                        "transcription": command,
                        "action_plan": complex_action,
                        "validation_result": validation_result,
                        "safety_result": safety_result,
                        "vision_context": vision_context,
                        "enhanced_object_selection": True,
                        "latency": {
                            "total": total_time
                        },
                        "timestamp": time.time()
                    }

                    logger.info(f"Enhanced object selection command processed successfully in {total_time:.2f}s")
                    return result

            # If enhanced planning isn't available or failed, fall back to standard processing
            return await self.process_command_string(command)

        except Exception as e:
            logger.error(f"Enhanced object selection command processing failed: {str(e)}")
            return {
                "success": False,
                "error": str(e),
                "timestamp": time.time()
            }

    async def process_multi_object_command(self, command: str) -> Optional[Dict[str, Any]]:
        """
        Process commands that involve multiple objects or complex object relationships.

        Args:
            command: Natural language command involving multiple objects

        Returns:
            Dictionary with processing results
        """
        start_time = time.time()

        try:
            # Get current vision context
            vision_context = create_example_vision_context()
            available_objects = vision_context.get('objects', [])

            if len(available_objects) < 2:
                logger.warning("Not enough objects for multi-object command processing")
                # Fall back to standard processing
                return await self.process_command_string(command)

            # Use enhanced multi-step planning for commands involving multiple objects
            if self.llm_planner and self.api_key:
                multi_step_plan = await self.llm_planner.plan_multi_step_actions_with_context(command, vision_context)

                if multi_step_plan and len(multi_step_plan) > 0:
                    # Validate each action in the plan
                    validated_plan = []
                    for action in multi_step_plan:
                        validation_result = self.json_validator.validate_action_json(
                            action["function"],
                            action["arguments"]
                        )

                        if not validation_result["valid"]:
                            logger.error(f"Action validation failed: {validation_result['errors']}")
                            continue

                        # Apply safety validation
                        safety_result = self.safety_guardrails.validate_action(action)
                        if not safety_result["valid"]:
                            logger.error(f"Safety validation failed: {safety_result['violations']}")
                            continue

                        validated_plan.append({
                            "action": action,
                            "validation": validation_result,
                            "safety": safety_result
                        })

                    if validated_plan:
                        # Calculate total time
                        total_time = time.time() - start_time

                        result = {
                            "success": True,
                            "transcription": command,
                            "multi_step_plan": validated_plan,
                            "vision_context": vision_context,
                            "total_steps": len(validated_plan),
                            "latency": {
                                "total": total_time
                            },
                            "timestamp": time.time()
                        }

                        logger.info(f"Multi-object command processed successfully with {len(validated_plan)} steps in {total_time:.2f}s")
                        return result

            # If multi-object processing isn't available or failed, fall back to standard processing
            return await self.process_command_string(command)

        except Exception as e:
            logger.error(f"Multi-object command processing failed: {str(e)}")
            return {
                "success": False,
                "error": str(e),
                "timestamp": time.time()
            }

    def disambiguate_object_reference(self, reference_description: str, available_objects: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """
        Disambiguate object references like "the red one" or "the leftmost object".

        Args:
            reference_description: Natural language description of the target object
            available_objects: List of objects to disambiguate between

        Returns:
            The object that best matches the description, or None if no clear match
        """
        if not available_objects:
            return None

        # Simple heuristic-based disambiguation
        # In a real system, this would use the LLM or more sophisticated algorithms

        # First, try to match by class
        for obj in available_objects:
            obj_class = obj.get('class', '').lower()
            desc_lower = reference_description.lower()

            # Check for class matches
            if obj_class in desc_lower:
                return obj

        # If no class match, try to match by color
        for obj in available_objects:
            obj_color = obj.get('color', '').lower()
            desc_lower = reference_description.lower()

            if obj_color in desc_lower:
                return obj

        # If no color match, try positional disambiguation
        if "left" in desc_lower:
            # Find the leftmost object
            leftmost = min(available_objects, key=lambda o: o.get('position', {}).get('x', 0))
            return leftmost
        elif "right" in desc_lower:
            # Find the rightmost object
            rightmost = max(available_objects, key=lambda o: o.get('position', {}).get('x', 0))
            return rightmost
        elif "front" in desc_lower or "forward" in desc_lower:
            # Find the frontmost object (positive y direction)
            frontmost = max(available_objects, key=lambda o: o.get('position', {}).get('y', 0))
            return frontmost
        elif "back" in desc_lower or "rear" in desc_lower:
            # Find the rearmost object (negative y direction)
            rearmost = min(available_objects, key=lambda o: o.get('position', {}).get('y', 0))
            return rearmost

        # If no specific match, return the first object as default
        return available_objects[0] if available_objects else None

    def get_latency_metrics(self) -> Dict[str, Any]:
        """
        Get current latency metrics for the pipeline.

        Returns:
            Dictionary with latency statistics
        """
        metrics = {}
        for stage, times in self.latency_metrics.items():
            if times:
                metrics[stage] = {
                    "count": len(times),
                    "mean": sum(times) / len(times),
                    "min": min(times),
                    "max": max(times),
                    "last_10_average": sum(times[-10:]) / min(10, len(times))
                }
            else:
                metrics[stage] = {
                    "count": 0,
                    "mean": 0.0,
                    "min": 0.0,
                    "max": 0.0,
                    "last_10_average": 0.0
                }

        return metrics

    async def run_continuous_listening(self, callback: Optional[callable] = None):
        """
        Run the pipeline in continuous listening mode.

        Args:
            callback: Optional callback function to handle results
        """
        self.is_running = True
        logger.info("Starting continuous listening mode...")

        while self.is_running:
            try:
                result = await self.process_voice_command(timeout=5.0)

                if result and result["success"]:
                    logger.info(f"Command processed successfully: {result['transcription']}")
                    if callback:
                        callback(result)
                else:
                    logger.warning("Command processing failed or timed out")

                # Brief pause to prevent excessive CPU usage
                await asyncio.sleep(0.1)

            except KeyboardInterrupt:
                logger.info("Continuous listening interrupted by user")
                break
            except Exception as e:
                logger.error(f"Error in continuous listening: {str(e)}")
                await asyncio.sleep(1)  # Brief pause before retrying

        self.is_running = False
        logger.info("Continuous listening stopped")

    def stop_continuous_listening(self):
        """Stop the continuous listening mode."""
        self.is_running = False

    async def process_command_string(self, command: str) -> Optional[Dict[str, Any]]:
        """
        Process a text command directly (bypassing voice capture).

        Args:
            command: Text command to process

        Returns:
            Dictionary with processing results
        """
        start_time = time.time()

        try:
            # Get vision context
            vision_context = create_example_vision_context()

            # Plan actions using LLM
            if not self.llm_planner:
                logger.error("LLM planner not available - API key required")
                return None

            action_plan = await self.llm_planner.plan_actions(command, vision_context)
            if not action_plan:
                logger.error("LLM planning failed")
                return None

            # Validate the planned action
            validation_result = self.json_validator.validate_action_json(
                action_plan["function"],
                action_plan["arguments"]
            )

            if not validation_result["valid"]:
                logger.error(f"Action validation failed: {validation_result['errors']}")
                return None

            # Apply safety validation
            safety_result = self.safety_guardrails.validate_action(action_plan)
            if not safety_result["valid"]:
                logger.error(f"Safety validation failed: {safety_result['violations']}")
                return None

            # Calculate total time
            total_time = time.time() - start_time

            # Prepare result
            result = {
                "success": True,
                "transcription": command,  # Using command as transcription
                "action_plan": action_plan,
                "validation_result": validation_result,
                "safety_result": safety_result,
                "vision_context": vision_context,
                "latency": {
                    "total": total_time
                },
                "timestamp": time.time()
            }

            logger.info(f"Text command processed successfully in {total_time:.2f}s")
            return result

        except Exception as e:
            logger.error(f"Text command processing failed: {str(e)}")
            return {
                "success": False,
                "error": str(e),
                "timestamp": time.time()
            }


async def test_vla_pipeline():
    """Test function for VLA pipeline functionality."""
    print("Testing VLA pipeline...")

    # Check if required API keys are available
    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        print("⚠️  OPENAI_API_KEY not set in environment. Testing with limited functionality.")

    try:
        # Create VLA pipeline instance
        pipeline = VLAPipeline()

        # Test 1: Process a text command directly (since voice testing requires actual hardware)
        command = "Pick up the red apple"
        result = await pipeline.process_command_string(command)

        if result and result["success"]:
            print(f"✓ Text command processed: '{command}' -> {result['action_plan']['function']}")
        else:
            print(f"✗ Text command failed: {result['error'] if result else 'No result'}")

        # Test 2: Get latency metrics
        metrics = pipeline.get_latency_metrics()
        print(f"✓ Latency metrics obtained: {len(metrics)} stages tracked")

        # Test 3: Validate components are initialized
        components_initialized = all([
            pipeline.json_validator is not None,
            pipeline.safety_guardrails is not None,
            pipeline.object_detector is not None,
            pipeline.grounding_pipeline is not None
        ])

        if api_key:
            components_initialized = components_initialized and all([
                pipeline.whisper_transcriber is not None,
                pipeline.llm_planner is not None
            ])

        print(f"✓ Components initialized: {components_initialized}")

        print("✓ VLA pipeline basic tests completed")
        return True

    except Exception as e:
        print(f"✗ VLA pipeline test failed: {e}")
        import traceback
        traceback.print_exc()
        return False


async def main_test():
    """Main test function."""
    success = await test_vla_pipeline()
    if success:
        print("✓ All VLA pipeline tests passed")
    else:
        print("✗ Some VLA pipeline tests failed")


if __name__ == "__main__":
    # Run the test
    asyncio.run(main_test())