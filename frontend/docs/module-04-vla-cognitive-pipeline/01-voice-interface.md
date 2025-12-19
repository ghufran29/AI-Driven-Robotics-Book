---
sidebar_position: 1
title: "Chapter 1: The Voice Interface (Whisper)"
slug: "/module-04-vla-cognitive-pipeline/voice-interface"
---

# Chapter 1: The Voice Interface (Whisper)

## Overview

The Voice Interface is the "ear" of the Vision-Language-Action (VLA) system. It captures audio from the user's microphone, applies Voice Activity Detection (VAD) to identify when the user is speaking, and transcribes the spoken command using OpenAI's Whisper API. This chapter covers the implementation and configuration of the voice interface components.

## Components

### Audio Capture

The audio capture module handles microphone input and provides the foundation for voice processing:

- **AudioCapture Class**: Manages microphone access and audio recording
- **Configuration Options**:
  - `device_index`: Index of the audio input device
  - `sample_rate`: Audio sample rate in Hz (default: 16000)
  - `chunk_size`: Size of audio chunks to process (default: 1024)

```python
from simulation_assets.vla.voice_interface.audio_capture import AudioCapture

# Initialize audio capture
capture = AudioCapture(device_index=0, sample_rate=16000, chunk_size=1024)

# Test microphone
if capture.test_microphone():
    print("Microphone is working properly")
else:
    print("Microphone test failed")
```

### Voice Activity Detection (VAD)

The VAD detector identifies when the user is speaking versus when there is silence or background noise:

- **VADDetector Class**: Analyzes audio chunks for voice activity
- **Configuration Options**:
  - `vad_threshold`: Threshold for detecting voice activity (default: 0.02)
  - `silence_duration`: Duration of silence to stop recording (default: 0.5s)
  - `min_audio_length`: Minimum audio length before stopping (default: 1.0s)

```python
from simulation_assets.vla.voice_interface.vad_detector import VADDetector

# Initialize VAD detector
vad = VADDetector(vad_threshold=0.02, silence_duration=0.5, min_audio_length=1.0)

# Detect voice activity in an audio chunk
voice_active = vad.detect_voice_activity(audio_chunk)
```

### Whisper Transcription

The Whisper transcriber converts captured audio into text using OpenAI's Whisper API:

- **WhisperTranscriber Class**: Handles API communication and transcription
- **Configuration Options**:
  - `model`: Whisper model to use (default: "whisper-1")
  - `language`: Optional language specification (e.g., 'en', 'es', 'fr')

```python
from simulation_assets.vla.voice_interface.whisper_transcription import WhisperTranscriber

# Initialize transcriber
transcriber = WhisperTranscriber(api_key="your-api-key", model="whisper-1")

# Transcribe audio
transcription = await transcriber.transcribe_audio(wav_audio)
```

## Configuration

The voice interface can be configured through environment variables:

```env
VLA_MIC_DEVICE_INDEX=0
VLA_AUDIO_SAMPLE_RATE=16000
VLA_AUDIO_CHUNK_SIZE=1024
VLA_VAD_THRESHOLD=0.02
VLA_SILENCE_DURATION=0.5
VLA_MIN_AUDIO_LENGTH=1.0
```

## Best Practices

1. **Microphone Quality**: Use a quality microphone with good signal-to-noise ratio for better transcription accuracy
2. **Environmental Noise**: Minimize background noise for optimal VAD performance
3. **Threshold Tuning**: Adjust VAD thresholds based on your specific environment
4. **API Costs**: Be mindful of Whisper API usage for cost management

## Troubleshooting

- **Poor Transcription Quality**: Check microphone positioning and environmental noise levels
- **No Voice Detection**: Verify microphone permissions and adjust VAD thresholds
- **High Latency**: Consider using shorter audio segments or optimizing network connectivity
- **API Errors**: Verify API key validity and check OpenAI service status

## Next Steps

Once the voice interface is properly configured, proceed to Chapter 2: The Cognitive Core (LLM Planning) to implement the natural language understanding and action planning components that will process the transcribed commands.