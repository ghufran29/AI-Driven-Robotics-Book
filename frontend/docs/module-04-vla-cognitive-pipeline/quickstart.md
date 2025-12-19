---
sidebar_position: 5
title: "Quickstart: Module 4 - Vision-Language-Action (VLA)"
slug: "/module-04-vla-cognitive-pipeline/quickstart"
---

# Quickstart: Module 4 - Vision-Language-Action (VLA)

## Overview

This quickstart guide provides a fast path to set up and run the Vision-Language-Action (VLA) system - a cognitive pipeline that transforms voice commands into robot actions. The system consists of four phases: voice interface, cognitive core, vision-language grounding, and capstone integration.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- Python 3.11
- OpenAI API key
- Microphone for voice input
- Camera for vision processing (optional for initial testing)
- ROS 2 Humble Hawksbill (for robot integration)

### Dependencies Installation
```bash
# Install Python dependencies
pip install -r simulation-assets/vla/requirements.txt

# Install ROS 2 dependencies (if integrating with robot)
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
VLA_CAMERA_INDEX=0
VLA_RISK_THRESHOLD=0.8
```

### 2. Verify Installation
```bash
# Test audio capture
cd simulation-assets/vla/voice_interface
python -m pytest audio_capture.py::test_audio_capture -v

# Test VAD detector
python -m pytest vad_detector.py::test_vad_detector -v

# Test LLM planner (requires API key)
cd ../cognitive_core
python -c "from llm_planning import create_example_vision_context; print('LLM imports successfully')"
```

## Running the VLA System

### 1. Basic Text Command Processing
```bash
# Navigate to the capstone integration directory
cd simulation-assets/vla/capstone_integration

# Run a simple test with a text command
python -c "
import asyncio
from main_pipeline import VLAPipeline

async def test_pipeline():
    pipeline = VLAPipeline()
    result = await pipeline.process_command_string('Move to position x=1.0 y=1.0')
    print(f'Result: {result}')

asyncio.run(test_pipeline())
"
```

### 2. Voice Command Processing (Hardware Required)
```bash
# Run the complete voice pipeline (requires microphone)
python -c "
import asyncio
from main_pipeline import VLAPipeline

async def voice_test():
    pipeline = VLAPipeline()
    result = await pipeline.process_voice_command(timeout=10.0)
    print(f'Voice result: {result}')

asyncio.run(voice_test())
"
```

### 3. Continuous Listening Mode
```bash
# Run the system in continuous listening mode
python -c "
import asyncio
from main_pipeline import VLAPipeline

async def continuous_test():
    pipeline = VLAPipeline()

    def handle_result(result):
        print(f'Processed: {result.get(\"transcription\", \"Unknown\")}')

    await pipeline.run_continuous_listening(callback=handle_result)

asyncio.run(continuous_test())
"
```

## Basic Usage Examples

### Simple Commands
```python
from simulation_assets.vla.capstone_integration.main_pipeline import VLAPipeline
import asyncio

async def run_examples():
    pipeline = VLAPipeline()

    # Example 1: Simple movement
    result1 = await pipeline.process_command_string("Move to position x=1.0 y=1.0")
    print(f"Movement command: {result1['success']}")

    # Example 2: Object manipulation (with example vision context)
    result2 = await pipeline.process_command_string("Pick up the red apple")
    print(f"Manipulation command: {result2['success']}")

    # Example 3: Navigation
    result3 = await pipeline.process_command_string("Navigate to the kitchen")
    print(f"Navigation command: {result3['success']}")

asyncio.run(run_examples())
```

### Expected Results
- Voice-to-action latency: \&lt;5 seconds (95% of interactions)
- Object identification accuracy: >90% for clear objects
- Task completion rate: >85% for simple commands
- Safety validation: 100% of dangerous commands blocked

## Configuration

### Main Parameters (via environment variables)
```bash
# Voice configuration
export VLA_MIC_DEVICE_INDEX=0
export VLA_AUDIO_SAMPLE_RATE=16000
export VLA_AUDIO_CHUNK_SIZE=1024
export VLA_VAD_THRESHOLD=0.02
export VLA_SILENCE_DURATION=0.5

# LLM configuration
export VLA_LLM_MODEL=gpt-4o
export VLA_LLM_TEMPERATURE=0.1

# Vision configuration
export VLA_DETECTION_THRESHOLD=0.7
export VLA_MAX_OBJECTS=10

# Safety configuration
export VLA_RISK_THRESHOLD=0.8
```

## Testing & Validation

### 1. Unit Tests
```bash
# Run all component tests
python -m pytest simulation-assets/vla/voice_interface/ -v
python -m pytest simulation-assets/vla/cognitive_core/ -v
python -m pytest simulation-assets/vla/vision_grounding/ -v
```

### 2. Latency Benchmarking
```bash
# Run latency benchmark
cd simulation-assets/vla/capstone_integration
python latency_benchmark.py
```

### 3. Integration Test
```bash
# Run end-to-end test
python -c "
import asyncio
from main_pipeline import VLAPipeline
from latency_benchmark import LatencyBenchmark

async def integration_test():
    pipeline = VLAPipeline()
    benchmark = LatencyBenchmark(target_latency=5.0)

    # Test common commands
    commands = [
        'Move to position x=1.0 y=1.0',
        'Pick up object',
        'Navigate to kitchen'
    ]

    results = await benchmark.benchmark_pipeline(pipeline, commands, num_runs=3)
    print(f'Benchmark completed. Mean latency: {results[\"mean_latency\"]:.3f}s')
    print(f'Success rate: {results[\"success_rate\"]:.2%}')

asyncio.run(integration_test())
"
```

## Troubleshooting

### Common Issues

**Issue**: "Microphone not detected"
- **Solution**: Check `VLA_MIC_DEVICE_INDEX` and ensure proper permissions
- **Test**: `python -c "import pyaudio; print(pyaudio.PyAudio().get_device_count())"`

**Issue**: "OpenAI API errors"
- **Solution**: Verify `OPENAI_API_KEY` is set correctly
- **Test**: Try a simple API call outside the VLA system

**Issue**: "High latency"
- **Solution**: Check network connectivity to OpenAI APIs
- **Alternative**: Consider using local models for specific components

**Issue**: "Object detection not working"
- **Solution**: Ensure camera is properly connected and accessible

### Performance Tuning
- Adjust `VLA_VAD_THRESHOLD` for better voice detection in noisy environments
- Modify `VLA_DETECTION_THRESHOLD` to balance accuracy and speed
- Tune `VLA_RISK_THRESHOLD` based on safety requirements
- Optimize `VLA_LLM_TEMPERATURE` for more/less creative responses

## Next Steps

After successful setup:
1. Integrate with physical robot (Module 1-3 systems)
2. Calibrate vision system for your specific environment
3. Fine-tune safety parameters for your use case
4. Expand vocabulary and commands based on requirements
5. Implement continuous learning capabilities

The VLA system is now ready for integration with your robot platform and can be extended with additional capabilities as needed.