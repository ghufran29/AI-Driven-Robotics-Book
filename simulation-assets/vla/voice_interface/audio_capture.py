"""
Audio Capture Module for VLA System

This module handles microphone audio capture with Voice Activity Detection (VAD).
"""
import pyaudio
import wave
import numpy as np
import time
import threading
from typing import Optional, Callable
import logging

logger = logging.getLogger(__name__)

class AudioCapture:
    def __init__(self, device_index: int = 0, sample_rate: int = 16000, chunk_size: int = 1024):
        """
        Initialize audio capture with specified parameters.

        Args:
            device_index: Index of the audio input device
            sample_rate: Audio sample rate in Hz
            chunk_size: Size of audio chunks to process
        """
        self.device_index = device_index
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        self.format = pyaudio.paInt16
        self.channels = 1

        self.audio = pyaudio.PyAudio()
        self.is_recording = False
        self.recording_thread = None

        # Audio data buffer
        self.audio_data = []

    def start_recording(self):
        """Start recording audio from the microphone."""
        if self.is_recording:
            logger.warning("Recording already in progress")
            return

        self.is_recording = True
        self.audio_data = []

        # Open audio stream
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=self.chunk_size
        )

        logger.info(f"Started recording from device {self.device_index}")

    def stop_recording(self) -> bytes:
        """Stop recording and return the recorded audio data."""
        if not self.is_recording:
            logger.warning("No recording in progress to stop")
            return b""

        # Close the audio stream
        self.stream.stop_stream()
        self.stream.close()

        # Combine all audio frames into a single bytes object
        audio_bytes = b"".join(self.audio_data)

        self.is_recording = False
        logger.info("Stopped recording")

        return audio_bytes

    def read_audio_chunk(self) -> bytes:
        """Read a single chunk of audio data from the stream."""
        if not self.is_recording:
            raise RuntimeError("No recording in progress")

        data = self.stream.read(self.chunk_size, exception_on_overflow=False)
        self.audio_data.append(data)
        return data

    def get_audio_level(self) -> float:
        """Get the current audio level (for VAD)."""
        if not self.is_recording:
            return 0.0

        # Read a chunk to analyze
        chunk = self.stream.read(self.chunk_size, exception_on_overflow=False)
        audio_data = np.frombuffer(chunk, dtype=np.int16)

        # Calculate RMS (Root Mean Square) for audio level
        rms = np.sqrt(np.mean(audio_data**2))

        # Add this chunk to our buffer
        self.audio_data.append(chunk)

        return rms

    def record_until_silence(self, vad_threshold: float = 0.02, silence_duration: float = 0.5,
                           min_audio_length: float = 1.0) -> bytes:
        """
        Record audio until a period of silence is detected.

        Args:
            vad_threshold: Threshold for detecting voice activity
            silence_duration: Duration of silence (in seconds) to stop recording
            min_audio_length: Minimum audio length (in seconds) before stopping

        Returns:
            Recorded audio data as bytes
        """
        if self.is_recording:
            logger.warning("Recording already in progress")
            return b""

        self.start_recording()

        silence_frames = 0
        min_frames = int(min_audio_length * self.sample_rate / self.chunk_size)
        silence_frames_threshold = int(silence_duration * self.sample_rate / self.chunk_size)

        frames_recorded = 0

        try:
            while True:
                audio_level = self.get_audio_level()
                frames_recorded += 1

                if audio_level > vad_threshold:
                    # Voice activity detected, reset silence counter
                    silence_frames = 0
                else:
                    # No voice activity, increment silence counter
                    silence_frames += 1

                # Check if we've reached minimum length and detected sufficient silence
                if frames_recorded >= min_frames and silence_frames >= silence_frames_threshold:
                    logger.info(f"Detected {silence_duration}s of silence, stopping recording")
                    break

        except KeyboardInterrupt:
            logger.info("Recording interrupted by user")
        finally:
            return self.stop_recording()

    def test_microphone(self) -> bool:
        """Test if the microphone is working properly."""
        try:
            # Open a short test stream
            test_stream = self.audio.open(
                format=self.format,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                input_device_index=self.device_index,
                frames_per_buffer=self.chunk_size
            )

            # Read a few chunks to test
            for _ in range(5):  # Test for ~0.3 seconds at 16kHz with 1024 chunk
                test_data = test_stream.read(self.chunk_size, exception_on_overflow=False)
                audio_level = np.sqrt(np.mean(np.frombuffer(test_data, dtype=np.int16)**2))

                if audio_level > 0:  # If we detect any audio, the mic is working
                    test_stream.stop_stream()
                    test_stream.close()
                    return True

            test_stream.stop_stream()
            test_stream.close()
            return False

        except Exception as e:
            logger.error(f"Microphone test failed: {e}")
            return False

    def __del__(self):
        """Clean up audio resources."""
        if hasattr(self, 'audio'):
            self.audio.terminate()


def test_audio_capture():
    """Test function for audio capture functionality."""
    import os

    # Get configuration from environment or use defaults
    device_index = int(os.getenv('VLA_MIC_DEVICE_INDEX', 0))
    sample_rate = int(os.getenv('VLA_AUDIO_SAMPLE_RATE', 16000))
    chunk_size = int(os.getenv('VLA_AUDIO_CHUNK_SIZE', 1024))

    print("Testing audio capture...")

    # Create audio capture instance
    capture = AudioCapture(device_index=device_index, sample_rate=sample_rate, chunk_size=chunk_size)

    # Test microphone
    if capture.test_microphone():
        print("✓ Microphone test passed")
    else:
        print("✗ Microphone test failed")
        return False

    # Test recording with VAD
    print("Recording audio (speak now, recording will stop after silence)...")
    audio_data = capture.record_until_silence(
        vad_threshold=0.02,
        silence_duration=0.5,
        min_audio_length=0.5
    )

    print(f"Recorded {len(audio_data)} bytes of audio data")

    if len(audio_data) > 0:
        print("✓ Audio recording test passed")

        # Save to a temporary WAV file for verification
        with wave.open("test_recording.wav", "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(pyaudio.PyAudio().get_sample_size(pyaudio.paInt16))
            wf.setframerate(sample_rate)
            wf.writeframes(audio_data)

        print("Saved recording to test_recording.wav")
        return True
    else:
        print("✗ Audio recording test failed - no audio captured")
        return False


if __name__ == "__main__":
    test_audio_capture()