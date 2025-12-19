# Troubleshooting: Module 4 - Vision-Language-Action (VLA)

## Overview

This troubleshooting guide addresses common issues encountered when setting up, configuring, and operating the Vision-Language-Action (VLA) system. Each section provides diagnostic steps and solutions for specific problems.

## Audio and Voice Interface Issues

### Problem: Microphone Not Detected
**Symptoms**: Audio capture fails, "No recording in progress" errors
**Solutions**:
1. Check microphone connections and permissions
2. Verify the correct device index: `python -c "import pyaudio; p = pyaudio.PyAudio(); print([p.get_device_info_by_index(i) for i in range(p.get_device_count())])"`
3. Set correct `VLA_MIC_DEVICE_INDEX` in environment variables
4. Test with a simple audio recording app first

### Problem: Poor Audio Quality
**Symptoms**: Low transcription accuracy, VAD not triggering
**Solutions**:
1. Check microphone placement (6-12 inches from speaker)
2. Reduce background noise in the environment
3. Adjust `VLA_VAD_THRESHOLD` (increase for noisy environments, decrease for quiet)
4. Verify audio sample rate matches configuration (`VLA_AUDIO_SAMPLE_RATE`)

### Problem: Whisper API Errors
**Symptoms**: Transcription fails, API errors returned
**Solutions**:
1. Verify `OPENAI_API_KEY` is correctly set
2. Check OpenAI account for API quota and billing
3. Ensure network connectivity to OpenAI services
4. Verify API key permissions and scope

## Cognitive Core Issues

### Problem: LLM Not Responding
**Symptoms**: Long delays, timeouts, or empty responses
**Solutions**:
1. Verify `OPENAI_API_KEY` is correctly set
2. Check network connectivity to OpenAI services
3. Verify API quota and billing status
4. Test with a simple API call outside the VLA system

### Problem: Unexpected Actions Generated
**Symptoms**: LLM generates actions not matching user intent
**Solutions**:
1. Review prompt engineering and function schemas
2. Adjust `VLA_LLM_TEMPERATURE` for more deterministic responses (lower values)
3. Verify vision context is properly provided to the planner
4. Check for ambiguous language in commands

### Problem: JSON Validation Failures
**Symptoms**: Actions fail validation, "Schema validation failed" errors
**Solutions**:
1. Verify action arguments match defined schemas
2. Check for missing required parameters
3. Validate data types (numbers vs strings)
4. Review function calling schemas in the LLM planner

## Vision and Grounding Issues

### Problem: Object Detection Not Working
**Symptoms**: No objects detected, "No vision data available" messages
**Solutions**:
1. Check camera connections and permissions
2. Verify camera index: `VLA_CAMERA_INDEX`
3. Ensure adequate lighting for object detection
4. Test camera independently before integration

### Problem: Poor Object Recognition
**Symptoms**: Low confidence scores, wrong object identification
**Solutions**:
1. Improve lighting conditions
2. Adjust `VLA_DETECTION_THRESHOLD` (lower for better detection, higher for accuracy)
3. Verify object is within camera's field of view
4. Check for occlusions or poor camera angle

### Problem: Grounding Failures
**Symptoms**: Wrong object selected, "No object found matching description"
**Solutions**:
1. Verify vision context is properly provided to grounding pipeline
2. Use more specific language descriptions
3. Check that target object is clearly visible
4. Adjust `VLA_MIN_CONFIDENCE` for grounding

## Performance Issues

### Problem: High Latency
**Symptoms**: Voice-to-action time exceeds 5 seconds
**Solutions**:
1. Check network connectivity to OpenAI APIs
2. Monitor system resources (CPU, memory, GPU if applicable)
3. Verify audio processing pipeline efficiency
4. Consider caching for frequently used commands

### Problem: System Slowdown
**Symptoms**: Gradual performance degradation over time
**Solutions**:
1. Monitor memory usage and implement garbage collection
2. Check for resource leaks in long-running processes
3. Verify proper cleanup of audio and image processing buffers
4. Consider restarting services periodically for long operations

### Problem: API Rate Limiting
**Symptoms**: Intermittent failures, "Rate limit exceeded" errors
**Solutions**:
1. Implement request queuing and retry logic
2. Add delays between API calls to stay within limits
3. Consider upgrading OpenAI account for higher rate limits
4. Implement local caching for repeated requests

## Safety and Validation Issues

### Problem: Actions Being Blocked Incorrectly
**Symptoms**: Safe actions flagged as dangerous, high risk level
**Solutions**:
1. Review safety configuration: `VLA_RISK_THRESHOLD`
2. Check for false positive keyword matches
3. Verify position limits are appropriate for environment
4. Adjust safety parameters conservatively

### Problem: Safety System Too Permissive
**Symptoms**: Potentially dangerous actions not blocked
**Solutions**:
1. Increase `VLA_RISK_THRESHOLD` for more conservative behavior
2. Add specific forbidden keywords to safety configuration
3. Implement additional environmental safety checks
4. Review and tighten position and force limits

## Integration Issues

### Problem: ROS Communication Failures
**Symptoms**: Action execution fails, ROS communication errors
**Solutions**:
1. Verify ROS 2 network configuration and domain ID
2. Check that required ROS actions are available
3. Confirm robot control system is properly initialized
4. Test ROS communication independently

### Problem: Vision System Not Updating
**Symptoms**: Outdated vision context, stale object positions
**Solutions**:
1. Check camera refresh rate and image capture frequency
2. Verify vision processing pipeline is running
3. Monitor for image buffer overflows
4. Synchronize vision and action timing

## Configuration Issues

### Problem: Environment Variables Not Loading
**Symptoms**: Default values used despite .env file present
**Solutions**:
1. Verify .env file is in correct location (project root)
2. Check file permissions and readability
3. Confirm variable names match exactly
4. Restart services after .env changes

### Problem: Component Initialization Failures
**Symptoms**: "Component not available" errors, initialization failures
**Solutions**:
1. Check that required API keys are available
2. Verify all dependencies are installed
3. Confirm network connectivity for external services
4. Review component initialization order

## Debugging Strategies

### Enable Detailed Logging
```bash
# Set logging level to DEBUG
export LOG_LEVEL=DEBUG

# Or in Python code:
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Test Components Individually
```python
# Test audio capture independently
from simulation_assets.vla.voice_interface.audio_capture import test_audio_capture
test_audio_capture()

# Test LLM planning independently
from simulation_assets.vla.cognitive_core.llm_planning import test_llm_planner
import asyncio
asyncio.run(test_llm_planner())
```

### Monitor System Resources
```bash
# Monitor CPU and memory usage
htop

# Monitor network connectivity
ping openai.com

# Check available disk space
df -h
```

## Common Error Messages

### "OpenAI API key is required"
- **Cause**: OPENAI_API_KEY not set in environment
- **Solution**: Set the API key in .env file or environment

### "No audio captured"
- **Cause**: Microphone not working or permissions issue
- **Solution**: Check microphone connections and permissions

### "Action validation failed"
- **Cause**: Action doesn't match expected schema
- **Solution**: Check action structure and required parameters

### "Safety validation failed"
- **Cause**: Action flagged as potentially dangerous
- **Solution**: Review safety configuration and action parameters

## Prevention Tips

1. **Regular Testing**: Test all components regularly to catch issues early
2. **Environment Monitoring**: Monitor network, CPU, and memory usage
3. **Configuration Management**: Keep backup configurations for quick recovery
4. **Safety First**: Always test with safety systems active
5. **Documentation**: Keep detailed logs of changes and their effects

## When to Seek Help

Contact support or development team when:
- Issues persist after following troubleshooting steps
- Safety systems are compromised
- Critical functionality is affected
- Root cause cannot be determined
- Performance issues impact operation significantly

## Conclusion

This troubleshooting guide covers the most common issues with the VLA system. Regular maintenance, proper configuration, and systematic debugging approaches will help maintain optimal system performance. Always prioritize safety when troubleshooting and testing.