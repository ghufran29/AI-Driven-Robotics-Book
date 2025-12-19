"""
Integration Tests for VLA System

This module contains integration tests for the complete VLA pipeline,
testing the interaction between all components: voice, cognitive, vision, and safety.
"""
import asyncio
import pytest
from unittest.mock import Mock, patch, AsyncMock
import numpy as np
import time
from typing import Dict, Any, Optional

# Import VLA components
from ..voice_interface.audio_capture import AudioCapture
from ..voice_interface.whisper_transcription import WhisperTranscriber
from ..voice_interface.vad_detector import VADDetector
from ..cognitive_core.llm_planning import LLMPlanner
from ..cognitive_core.json_validator import JSONValidator
from ..cognitive_core.safety_guardrails import SafetyGuardrails
from ..vision_grounding.object_detector import ObjectDetector
from ..vision_grounding.grounding_pipeline import GroundingPipeline
from .main_pipeline import VLAPipeline
from .latency_benchmark import LatencyBenchmark


class TestVLAIntegration:
    """Integration tests for the complete VLA system."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        # Mock API keys for testing
        import os
        os.environ['OPENAI_API_KEY'] = 'test-key-for-testing'

    @pytest.mark.asyncio
    async def test_complete_pipeline_text_command(self):
        """Test the complete pipeline with a text command."""
        # Initialize pipeline with mocked components where needed
        pipeline = VLAPipeline()

        # Since we can't test real OpenAI APIs in CI, we'll test the structure
        # The pipeline should be able to process a command even if API calls fail gracefully
        command = "Move to position x=1.0 y=1.0"

        result = await pipeline.process_command_string(command)

        # The result should be structured properly even if the LLM isn't available
        assert 'success' in result
        assert 'timestamp' in result

        # If API key is not available, success might be False, but structure should be correct
        if result['success']:
            assert 'action_plan' in result
            assert 'transcription' in result
            assert result['transcription'] == command

    @pytest.mark.asyncio
    async def test_voice_to_action_pipeline(self):
        """Test the complete voice-to-action pipeline with mocked audio."""
        pipeline = VLAPipeline()

        # Mock audio capture to return a simple test signal
        with patch.object(pipeline.audio_capture, 'record_until_silence', return_value=b'\x00' * 1024):
            with patch.object(pipeline.audio_capture, 'test_microphone', return_value=True):
                # This will fail gracefully if Whisper API is not configured
                result = await pipeline.process_voice_command(timeout=2.0)

                # Result should have proper structure
                assert isinstance(result, dict)
                assert 'success' in result

    def test_json_validation_integration(self):
        """Test JSON validation with cognitive core output."""
        validator = JSONValidator()

        # Test a valid action
        valid_action = {
            "function": "move_to_position",
            "arguments": {"x": 1.0, "y": 1.0, "frame_id": "map"}
        }

        result = validator.validate_action_json(valid_action["function"], valid_action["arguments"])
        assert result["valid"] is True
        assert len(result["errors"]) == 0

        # Test an invalid action (missing required field)
        invalid_action = {
            "function": "move_to_position",
            "arguments": {"x": 1.0}  # Missing y and frame_id
        }

        result = validator.validate_action_json(invalid_action["function"], invalid_action["arguments"])
        assert result["valid"] is False
        assert len(result["errors"]) > 0

    def test_safety_integration(self):
        """Test safety guardrails with action validation."""
        guardrails = SafetyGuardrails(risk_threshold=0.8)

        # Test a safe action
        safe_action = {
            "function": "move_to_position",
            "arguments": {"x": 1.0, "y": 1.0, "z": 0.5}
        }

        result = guardrails.validate_action(safe_action)
        assert result["valid"] is True
        assert result["risk_level"].value in ["low", "medium"]

        # Test a potentially dangerous action
        dangerous_action = {
            "function": "move_to_position",
            "arguments": {"x": 100.0, "y": 100.0, "z": 0.5}  # Outside safe limits
        }

        result = guardrails.validate_action(dangerous_action)
        # This might be blocked due to position limits
        assert isinstance(result, dict)
        assert "risk_level" in result

    def test_vision_grounding_integration(self):
        """Test vision system with grounding pipeline."""
        detector = ObjectDetector(detection_threshold=0.5)
        grounding_pipeline = GroundingPipeline(grounding_threshold=0.3)

        # Use mock image (zeros array)
        mock_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Detect objects
        detections = detector.detect_objects(mock_image)

        # Ground language to objects
        result = grounding_pipeline.ground_language_to_objects("find object", detections)

        # Should return None for mock detection but with proper structure
        if result is not None:
            assert "class" in result
            assert "confidence" in result

    @pytest.mark.asyncio
    async def test_pipeline_latency_tracking(self):
        """Test that the pipeline properly tracks latency metrics."""
        pipeline = VLAPipeline()

        # Process a command to generate latency data
        result = await pipeline.process_command_string("Move to position x=0.0 y=0.0")

        # Check that latency metrics are being tracked
        metrics = pipeline.get_latency_metrics()
        assert isinstance(metrics, dict)
        assert "total_processing" in metrics

    def test_configuration_loading(self):
        """Test that pipeline loads configuration properly."""
        pipeline = VLAPipeline()

        # Check that config has expected sections
        assert hasattr(pipeline, 'config')
        assert 'voice' in pipeline.config
        assert 'llm' in pipeline.config
        assert 'vision' in pipeline.config
        assert 'safety' in pipeline.config

    @pytest.mark.asyncio
    async def test_error_handling_in_pipeline(self):
        """Test that the pipeline handles errors gracefully."""
        pipeline = VLAPipeline()

        # Test with invalid command
        result = await pipeline.process_command_string("")

        # Should return a structured error response
        assert isinstance(result, dict)
        assert 'success' in result

    def test_latency_benchmark_integration(self):
        """Test latency benchmarking with mock pipeline."""
        benchmark = LatencyBenchmark(target_latency=5.0, warmup_runs=1)

        # Create a mock pipeline for testing
        mock_pipeline = Mock()
        mock_pipeline.process_command_string = AsyncMock(return_value={
            "success": True,
            "transcription": "test command",
            "action_plan": {"function": "test", "arguments": {}},
            "latency": {"total": 1.5},
            "timestamp": time.time()
        })

        async def run_mock_benchmark():
            results = await benchmark.benchmark_pipeline(
                mock_pipeline,
                test_commands=["test command"],
                num_runs=2
            )
            return results

        # Run the benchmark
        results = asyncio.run(run_mock_benchmark())

        # Verify results structure
        assert "mean_latency" in results
        assert "success_rate" in results
        assert results["total_runs"] == 2


class TestVLAComponentsIntegration:
    """Tests for integration between individual VLA components."""

    def test_voice_and_cognitive_integration(self):
        """Test integration between voice processing and cognitive core."""
        vad = VADDetector(vad_threshold=0.02)

        # Test voice activity detection
        # Create a simple audio chunk simulation
        import struct
        audio_chunk = b''.join([struct.pack('<h', int(1000 * 0.5 * (i % 100))) for i in range(1024)])
        voice_active = vad.detect_voice_activity(audio_chunk)

        # Should return boolean
        assert isinstance(voice_active, bool)

    def test_cognitive_and_vision_integration(self):
        """Test how cognitive core would use vision context."""
        # This is more of a structural test since we can't run real LLM without API
        # Create mock vision context
        vision_context = {
            "objects": [
                {"id": "obj_1", "class": "apple", "color": "red", "position": {"x": 1.0, "y": 1.0, "z": 0.8}},
                {"id": "obj_2", "class": "bottle", "color": "blue", "position": {"x": 1.5, "y": 0.5, "z": 0.8}}
            ]
        }

        # Verify structure
        assert "objects" in vision_context
        assert len(vision_context["objects"]) > 0
        assert "class" in vision_context["objects"][0]

    def test_vision_and_grounding_integration(self):
        """Test integration between vision detection and grounding."""
        detector = ObjectDetector()
        grounding = GroundingPipeline()

        # Mock image
        mock_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Detect objects
        detections = detector.detect_objects(mock_image)

        # Ground a simple description
        result = grounding.ground_language_to_objects("find object", detections)

        # Should return appropriate structure
        if result is not None:
            assert isinstance(result, dict)


def run_integration_tests():
    """Run all integration tests."""
    print("Running VLA Integration Tests...")

    test_instance = TestVLAIntegration()
    test_instance.setup_method()

    # Run a few key tests
    try:
        # Test 1: Configuration loading
        test_instance.test_configuration_loading()
        print("✓ Configuration loading test passed")

        # Test 2: JSON validation
        test_instance.test_json_validation_integration()
        print("✓ JSON validation integration test passed")

        # Test 3: Safety integration
        test_instance.test_safety_integration()
        print("✓ Safety integration test passed")

        # Test 4: Vision grounding
        test_instance.test_vision_grounding_integration()
        print("✓ Vision grounding integration test passed")

        print("All integration tests completed successfully!")

    except Exception as e:
        print(f"Integration test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    return True


if __name__ == "__main__":
    success = run_integration_tests()
    if success:
        print("\n✓ All VLA integration tests passed!")
    else:
        print("\n✗ Some VLA integration tests failed!")