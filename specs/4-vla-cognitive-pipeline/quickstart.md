# Quickstart: Module 4: Vision-Language-Action (VLA)

**Feature**: 4-vla-cognitive-pipeline
**Created**: 2025-12-18
**Status**: Complete

## Overview

This quickstart guide provides a fast path to set up and run the Vision-Language-Action (VLA) system - a cognitive pipeline that transforms voice commands into robot actions. The system consists of four phases: voice interface, cognitive core, vision-language grounding, and capstone integration.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Python 3.11
- NVIDIA GPU with CUDA (for vision processing)
- Microphone for voice input
- OpenAI API key

### Dependencies Installation
```bash
# Install Python dependencies
pip install openai langchain pyaudio speechrecognition transformers torch rclpy

# Install ROS 2 dependencies
sudo apt update
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

## Setup

### 1. Environment Configuration
Create a `.env` file in the project root:
```env
OPENAI_API_KEY=your_openai_api_key_here
ROS_DOMAIN_ID=42
VLA_MIC_DEVICE_INDEX=0
VLA_AUDIO_SAMPLE_RATE=16000
VLA_AUDIO_CHUNK_SIZE=1024
```

### 2. Directory Structure
```bash
mkdir -p docs/module-04-vla-capstone/
mkdir -p simulation-assets/vla/{voice_interface,cognitive_core,vision_grounding,capstone_integration,config}
```

## Running the VLA System

### 1. Voice Interface Setup
```bash
# Test audio capture
python simulation-assets/vla/voice_interface/audio_capture.py --test

# Verify Whisper API connection
python simulation-assets/vla/voice_interface/whisper_transcription.py --test
```

### 2. Cognitive Core Configuration
```bash
# Test LLM prompt engineering
python simulation-assets/vla/cognitive_core/llm_planning.py --test

# Validate JSON output format
python simulation-assets/vla/cognitive_core/json_validator.py --test
```

### 3. Vision-Language Grounding
```bash
# Test object detection
python simulation-assets/vla/vision_grounding/object_detector.py --test

# Verify vision-language connection
python simulation-assets/vla/vision_grounding/grounding_pipeline.py --test
```

### 4. Full Pipeline Execution
```bash
# Run the complete cognitive pipeline
python simulation-assets/vla/capstone_integration/main_pipeline.py

# Monitor performance
python simulation-assets/vla/capstone_integration/latency_benchmark.py
```

## Basic Usage

### Voice Command to Action
1. Start the VLA system: `python main_pipeline.py`
2. Speak a command like "Pick up the red bottle"
3. System will:
   - Capture audio with VAD
   - Transcribe with Whisper
   - Parse intent with LLM
   - Identify objects with vision system
   - Generate ROS actions
   - Execute with safety validation

### Example Commands
- "Move to the left of the blue box"
- "Pick up the apple on the left"
- "Clean the room by picking up red toys"
- "Navigate to the kitchen and wait"

## Configuration

### Main Parameters (vla_parameters.yaml)
```yaml
voice:
  vad_threshold: 0.02
  silence_duration: 0.5
  min_audio_length: 1.0

llm:
  model: "gpt-4o"
  temperature: 0.1
  max_tokens: 500
  function_calling: true

vision:
  detection_threshold: 0.7
  max_objects: 10
  processing_frequency: 1.0

safety:
  risk_threshold: 0.8
  confirmation_required: ["move_to", "grasp"]
  blocked_actions: ["dangerous_command"]
```

## Testing

### Mock Testing (No Hardware)
```bash
# Test cognitive logic without robot
python -m pytest simulation-assets/vla/capstone_integration/integration_tests.py -k "mock"
```

### Latency Benchmarking
```bash
# Measure end-to-end performance
python simulation-assets/vla/capstone_integration/latency_benchmark.py --duration 60
```

### Capstone Validation
```bash
# Full "Clean Room" test
python simulation-assets/vla/capstone_integration/integration_tests.py --capstone
```

## Expected Results

- Voice-to-action latency: <5 seconds (95% of interactions)
- Object identification accuracy: >90% for clear objects
- Task completion rate: >85% for simple commands
- Safety validation: 100% of dangerous commands blocked
- Multi-step success rate: >80% for 3+ action tasks

## Troubleshooting

### Common Issues
- High latency: Check network connection to OpenAI APIs
- Poor transcription: Verify microphone and audio settings
- Object misidentification: Ensure good lighting and clear view
- Safety blocks: Review safety configuration in parameters

### Performance Tuning
- Adjust VAD sensitivity for better voice detection
- Modify LLM temperature for more/less creative responses
- Tune vision thresholds for accuracy vs speed trade-off
- Optimize safety parameters for appropriate risk level