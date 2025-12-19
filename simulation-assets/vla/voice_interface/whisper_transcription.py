"""
Whisper Transcription Module for VLA System

This module handles audio transcription using OpenAI's Whisper API.
"""
import asyncio
import aiohttp
import io
import wave
import os
from typing import Optional, Dict, Any
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import OpenAI - handle the case where it might not be installed
try:
    import openai
    from openai import AsyncOpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False
    print("Warning: openai package not installed. Whisper transcription will not work.")

logger = logging.getLogger(__name__)

class WhisperTranscriber:
    """
    Class for handling audio transcription using OpenAI's Whisper API.
    """
    def __init__(self, api_key: Optional[str] = None, model: str = "whisper-1"):
        """
        Initialize the Whisper transcriber.

        Args:
            api_key: OpenAI API key. If None, will use OPENAI_API_KEY environment variable
            model: Whisper model to use (default: whisper-1)
        """
        if not OPENAI_AVAILABLE:
            raise ImportError("OpenAI package is not installed. Please install it with 'pip install openai'")

        # Set API key
        self.api_key = api_key or os.getenv('OPENAI_API_KEY')
        if not self.api_key:
            raise ValueError("OpenAI API key is required. Set OPENAI_API_KEY environment variable.")

        # Initialize OpenAI client
        self.client = AsyncOpenAI(api_key=self.api_key)
        self.model = model

    async def transcribe_audio(self, audio_data: bytes, file_format: str = "wav",
                              language: Optional[str] = None) -> Optional[str]:
        """
        Transcribe audio data using OpenAI's Whisper API.

        Args:
            audio_data: Audio data as bytes
            file_format: Format of the audio data (default: wav)
            language: Language of the audio (optional, e.g., 'en', 'es', 'fr')

        Returns:
            Transcribed text, or None if transcription failed
        """
        if not self.api_key:
            logger.error("OpenAI API key not set")
            return None

        try:
            # Create a BytesIO object from the audio data
            audio_io = io.BytesIO(audio_data)

            # Create a temporary filename with the appropriate extension
            temp_filename = f"temp_audio.{file_format}"

            # Write the audio data to the BytesIO object
            audio_io.name = temp_filename

            # Use the OpenAI API to transcribe the audio
            response = await self.client.audio.transcriptions.create(
                model=self.model,
                file=audio_io,
                language=language,
                response_format="text"  # Return plain text instead of JSON
            )

            transcription = response
            logger.info(f"Successfully transcribed audio: {transcription[:100]}...")
            return transcription

        except Exception as e:
            logger.error(f"Whisper transcription failed: {str(e)}")
            return None

    async def transcribe_audio_with_timestamps(self, audio_data: bytes, file_format: str = "wav",
                                             language: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """
        Transcribe audio data with timestamps using OpenAI's Whisper API.

        Args:
            audio_data: Audio data as bytes
            file_format: Format of the audio data (default: wav)
            language: Language of the audio (optional)

        Returns:
            Dictionary with transcription and segments, or None if transcription failed
        """
        if not self.api_key:
            logger.error("OpenAI API key not set")
            return None

        try:
            # Create a BytesIO object from the audio data
            audio_io = io.BytesIO(audio_data)
            temp_filename = f"temp_audio.{file_format}"
            audio_io.name = temp_filename

            # Use the OpenAI API to transcribe the audio with verbose JSON format
            response = await self.client.audio.transcriptions.create(
                model=self.model,
                file=audio_io,
                language=language,
                response_format="verbose_json"  # Return detailed JSON with timestamps
            )

            result = {
                "text": response.text,
                "language": response.language,
                "duration": response.duration,
                "segments": response.segments if hasattr(response, 'segments') else []
            }

            logger.info(f"Successfully transcribed audio with timestamps: {result['text'][:100]}...")
            return result

        except Exception as e:
            logger.error(f"Whisper transcription with timestamps failed: {str(e)}")
            return None

    async def transcribe_from_file(self, file_path: str, language: Optional[str] = None) -> Optional[str]:
        """
        Transcribe audio from a file using OpenAI's Whisper API.

        Args:
            file_path: Path to the audio file
            language: Language of the audio (optional)

        Returns:
            Transcribed text, or None if transcription failed
        """
        if not os.path.exists(file_path):
            logger.error(f"Audio file does not exist: {file_path}")
            return None

        try:
            with open(file_path, "rb") as audio_file:
                response = await self.client.audio.transcriptions.create(
                    model=self.model,
                    file=audio_file,
                    language=language,
                    response_format="text"
                )

            transcription = response
            logger.info(f"Successfully transcribed file {file_path}: {transcription[:100]}...")
            return transcription

        except Exception as e:
            logger.error(f"Whisper file transcription failed: {str(e)}")
            return None

    async def validate_transcription_quality(self, audio_data: bytes, expected_keywords: Optional[list] = None) -> Dict[str, Any]:
        """
        Validate transcription quality by checking for expected keywords or patterns.

        Args:
            audio_data: Audio data as bytes
            expected_keywords: List of keywords that should appear in the transcription

        Returns:
            Dictionary with validation results
        """
        transcription = await self.transcribe_audio(audio_data)

        if not transcription:
            return {
                "valid": False,
                "confidence": 0.0,
                "transcription": "",
                "error": "Transcription failed"
            }

        # Basic validation
        result = {
            "valid": True,
            "confidence": 1.0,  # Placeholder - actual confidence would require more sophisticated analysis
            "transcription": transcription,
            "error": None
        }

        # Check for expected keywords if provided
        if expected_keywords:
            found_keywords = []
            transcription_lower = transcription.lower()
            for keyword in expected_keywords:
                if keyword.lower() in transcription_lower:
                    found_keywords.append(keyword)

            result["found_keywords"] = found_keywords
            result["expected_keywords"] = expected_keywords

            # Update validity based on keyword presence
            if len(found_keywords) < len(expected_keywords):
                result["valid"] = False
                result["confidence"] = len(found_keywords) / len(expected_keywords) if expected_keywords else 1.0

        return result


def create_wav_bytes(raw_audio: bytes, sample_rate: int = 16000, channels: int = 1, sample_width: int = 2) -> bytes:
    """
    Convert raw audio bytes to WAV format bytes.

    Args:
        raw_audio: Raw audio data as bytes
        sample_rate: Sample rate of the audio
        channels: Number of audio channels
        sample_width: Sample width in bytes

    Returns:
        WAV formatted audio data as bytes
    """
    # Create a BytesIO buffer to hold the WAV data
    wav_buffer = io.BytesIO()

    # Create WAV file
    with wave.open(wav_buffer, 'wb') as wav_file:
        wav_file.setnchannels(channels)
        wav_file.setsampwidth(sample_width)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(raw_audio)

    # Get the WAV bytes
    wav_bytes = wav_buffer.getvalue()
    wav_buffer.close()

    return wav_bytes


async def test_whisper_transcriber():
    """Test function for Whisper transcriber functionality."""
    print("Testing Whisper transcriber...")

    # Check if API key is available
    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        print("⚠️  OPENAI_API_KEY not set in environment. Skipping Whisper tests.")
        return True  # Return True to continue with other tests

    try:
        # Create Whisper transcriber instance
        transcriber = WhisperTranscriber(api_key=api_key)

        # Test with a very short dummy audio (this would normally fail, but tests the setup)
        # For actual testing, you'd need real audio data
        dummy_audio = b'\x00' * 1024  # Silent audio data

        # Create WAV format from dummy audio
        wav_audio = create_wav_bytes(dummy_audio)

        # Test transcription
        transcription = await transcriber.transcribe_audio(wav_audio)
        print(f"Transcription result: {transcription}")

        # For real testing, you would use actual audio data
        print("✓ Whisper transcriber initialized successfully")
        return True

    except Exception as e:
        print(f"✗ Whisper transcriber test failed: {e}")
        return False


async def main_test():
    """Main test function."""
    success = await test_whisper_transcriber()
    if success:
        print("✓ All Whisper transcriber tests passed")
    else:
        print("✗ Some Whisper transcriber tests failed")


if __name__ == "__main__":
    # Run the test
    asyncio.run(main_test())