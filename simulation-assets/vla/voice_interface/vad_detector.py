"""
Voice Activity Detection (VAD) Module for VLA System

This module implements voice activity detection algorithms to identify
when a user is speaking versus when there is silence or background noise.
"""
import numpy as np
from typing import List, Tuple, Optional
import logging

logger = logging.getLogger(__name__)

class VADDetector:
    """
    Voice Activity Detection class that analyzes audio data to detect speech.
    """
    def __init__(self, vad_threshold: float = 0.02, silence_duration: float = 0.5,
                 min_audio_length: float = 1.0):
        """
        Initialize VAD detector with specified parameters.

        Args:
            vad_threshold: Threshold for detecting voice activity (RMS value)
            silence_duration: Duration of silence (in seconds) to consider as end of speech
            min_audio_length: Minimum audio length (in seconds) before stopping
        """
        self.vad_threshold = vad_threshold
        self.silence_duration = silence_duration
        self.min_audio_length = min_audio_length

        # State variables
        self.is_speaking = False
        self.silence_start_time = None
        self.speech_start_time = None

    def detect_voice_activity(self, audio_chunk: bytes, sample_rate: int = 16000) -> bool:
        """
        Detect if the given audio chunk contains voice activity.

        Args:
            audio_chunk: Audio data as bytes
            sample_rate: Sample rate of the audio

        Returns:
            True if voice activity is detected, False otherwise
        """
        # Convert bytes to numpy array
        audio_data = np.frombuffer(audio_chunk, dtype=np.int16)

        # Calculate RMS (Root Mean Square) for audio level
        rms = np.sqrt(np.mean(audio_data.astype(np.float32)**2) / (2**31))

        # Normalize RMS value
        normalized_rms = rms

        # Check if RMS is above the threshold
        voice_detected = normalized_rms > self.vad_threshold

        return voice_detected

    def analyze_audio_stream(self, audio_chunks: List[bytes], sample_rate: int = 16000) -> List[bool]:
        """
        Analyze a sequence of audio chunks to detect voice activity.

        Args:
            audio_chunks: List of audio data chunks as bytes
            sample_rate: Sample rate of the audio

        Returns:
            List of boolean values indicating voice activity for each chunk
        """
        results = []
        for chunk in audio_chunks:
            voice_active = self.detect_voice_activity(chunk, sample_rate)
            results.append(voice_active)
        return results

    def detect_speech_segments(self, audio_chunks: List[bytes], sample_rate: int = 16000) -> List[Tuple[int, int]]:
        """
        Detect speech segments in a sequence of audio chunks.

        Args:
            audio_chunks: List of audio data chunks as bytes
            sample_rate: Sample rate of the audio

        Returns:
            List of tuples (start_index, end_index) for each speech segment
        """
        voice_activities = self.analyze_audio_stream(audio_chunks, sample_rate)

        segments = []
        in_speech = False
        start_idx = 0

        for i, is_voice in enumerate(voice_activities):
            if is_voice and not in_speech:
                # Start of speech segment
                start_idx = i
                in_speech = True
            elif not is_voice and in_speech:
                # End of speech segment
                segments.append((start_idx, i - 1))
                in_speech = False

        # Handle case where audio ends while still in speech
        if in_speech:
            segments.append((start_idx, len(voice_activities) - 1))

        return segments

    def is_silence_threshold_reached(self, audio_chunk: bytes, chunk_duration: float,
                                     sample_rate: int = 16000) -> bool:
        """
        Check if the silence threshold has been reached based on the audio chunk.

        Args:
            audio_chunk: Audio data as bytes
            chunk_duration: Duration of the chunk in seconds
            sample_rate: Sample rate of the audio

        Returns:
            True if silence threshold is reached, False otherwise
        """
        voice_active = self.detect_voice_activity(audio_chunk, sample_rate)

        if voice_active:
            # If voice is detected, reset the silence timer
            self.silence_start_time = None
            return False
        else:
            # No voice detected, check if we've been in silence long enough
            if self.silence_start_time is None:
                # Start timing the silence
                self.silence_start_time = 0

            self.silence_start_time += chunk_duration

            # Check if silence duration exceeds threshold
            return self.silence_start_time >= self.silence_duration

    def calculate_rms_energy(self, audio_chunk: bytes) -> float:
        """
        Calculate the RMS energy of an audio chunk.

        Args:
            audio_chunk: Audio data as bytes

        Returns:
            RMS energy value
        """
        audio_data = np.frombuffer(audio_chunk, dtype=np.int16)
        rms = np.sqrt(np.mean(audio_data.astype(np.float32)**2) / (2**31))
        return rms

    def calculate_zero_crossing_rate(self, audio_chunk: bytes) -> float:
        """
        Calculate the zero crossing rate of an audio chunk.

        Args:
            audio_chunk: Audio data as bytes

        Returns:
            Zero crossing rate
        """
        audio_data = np.frombuffer(audio_chunk, dtype=np.int16)
        zero_crossings = np.sum(np.diff(np.sign(audio_data)) != 0)
        zcr = zero_crossings / len(audio_data)
        return zcr

    def detect_using_multiple_features(self, audio_chunk: bytes, sample_rate: int = 16000) -> bool:
        """
        Detect voice activity using multiple features (RMS and ZCR).

        Args:
            audio_chunk: Audio data as bytes
            sample_rate: Sample rate of the audio

        Returns:
            True if voice activity is detected, False otherwise
        """
        # Calculate RMS energy
        rms_energy = self.calculate_rms_energy(audio_chunk)

        # Calculate zero crossing rate
        zcr = self.calculate_zero_crossing_rate(audio_chunk)

        # Use both features for detection
        # RMS should be above threshold AND ZCR should be within reasonable range
        rms_active = rms_energy > self.vad_threshold
        zcr_active = 0.001 < zcr < 0.1  # Typical ZCR range for speech

        return rms_active and zcr_active


class AdaptiveVADDetector(VADDetector):
    """
    Adaptive VAD detector that adjusts thresholds based on background noise.
    """
    def __init__(self, initial_threshold: float = 0.02, silence_duration: float = 0.5,
                 min_audio_length: float = 1.0, adaptation_rate: float = 0.01):
        """
        Initialize adaptive VAD detector.

        Args:
            initial_threshold: Initial threshold for voice detection
            silence_duration: Duration of silence (in seconds) to consider as end of speech
            min_audio_length: Minimum audio length (in seconds) before stopping
            adaptation_rate: Rate at which the threshold adapts to background noise
        """
        super().__init__(initial_threshold, silence_duration, min_audio_length)
        self.adaptation_rate = adaptation_rate
        self.background_noise_level = initial_threshold
        self.noise_estimation_samples = 0

    def update_background_noise(self, audio_chunk: bytes):
        """
        Update the background noise estimate based on the current audio chunk.

        Args:
            audio_chunk: Audio data as bytes
        """
        rms_energy = self.calculate_rms_energy(audio_chunk)

        # If this chunk appears to be background noise (low energy), update estimate
        if rms_energy < self.background_noise_level:
            # Adaptive update of background noise level
            self.background_noise_level = (
                (1 - self.adaptation_rate) * self.background_noise_level +
                self.adaptation_rate * rms_energy
            )

        # Update the threshold based on background noise
        # Add a margin above the background noise level
        self.vad_threshold = max(0.001, self.background_noise_level * 2.0)

    def detect_voice_activity(self, audio_chunk: bytes, sample_rate: int = 16000) -> bool:
        """
        Detect voice activity with adaptive threshold.

        Args:
            audio_chunk: Audio data as bytes
            sample_rate: Sample rate of the audio

        Returns:
            True if voice activity is detected, False otherwise
        """
        # Update background noise estimate
        self.update_background_noise(audio_chunk)

        # Calculate current audio level
        rms_energy = self.calculate_rms_energy(audio_chunk)

        # Check if current level is above adaptive threshold
        return rms_energy > self.vad_threshold


def test_vad_detector():
    """Test function for VAD detector functionality."""
    print("Testing VAD detector...")

    # Create VAD detector instance
    vad = VADDetector(vad_threshold=0.02, silence_duration=0.5, min_audio_length=1.0)

    # Test with simulated audio data
    # Create a simple sine wave (simulating speech)
    sample_rate = 16000
    duration = 0.1  # 100ms
    frequency = 440  # A4 note

    # Generate audio data for speech simulation
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    speech_signal = (0.5 * np.sin(2 * np.pi * frequency * t) * 32767).astype(np.int16)
    speech_bytes = speech_signal.tobytes()

    # Generate audio data for silence
    silence_signal = np.zeros(int(sample_rate * duration), dtype=np.int16)
    silence_bytes = silence_signal.tobytes()

    # Test voice detection on speech
    speech_detected = vad.detect_voice_activity(speech_bytes, sample_rate)
    print(f"Speech detection: {speech_detected}")

    # Test voice detection on silence
    silence_detected = vad.detect_voice_activity(silence_bytes, sample_rate)
    print(f"Silence detection: {silence_detected}")

    # Test with multiple chunks
    chunks = [speech_bytes, silence_bytes, speech_bytes, silence_bytes]
    activities = vad.analyze_audio_stream(chunks, sample_rate)
    print(f"Activities: {activities}")

    # Test speech segment detection
    segments = vad.detect_speech_segments(chunks, sample_rate)
    print(f"Speech segments: {segments}")

    # Test adaptive VAD
    adaptive_vad = AdaptiveVADDetector()
    adaptive_speech_detected = adaptive_vad.detect_voice_activity(speech_bytes, sample_rate)
    print(f"Adaptive speech detection: {adaptive_speech_detected}")

    # Check results
    success = (
        speech_detected and  # Speech should be detected
        not silence_detected and  # Silence should not be detected
        activities == [True, False, True, False] and  # Correct pattern
        adaptive_speech_detected  # Adaptive detection should work
    )

    if success:
        print("✓ VAD detector test passed")
        return True
    else:
        print("✗ VAD detector test failed")
        return False


if __name__ == "__main__":
    test_vad_detector()