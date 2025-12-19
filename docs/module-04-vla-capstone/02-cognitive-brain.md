# Chapter 2: The Cognitive Core (LLM Planning)

## Overview

The Cognitive Core is the "brain" of the Vision-Language-Action (VLA) system. It processes transcribed voice commands using Large Language Models (LLMs) with function calling to generate structured ROS 2 actions. This chapter covers the implementation of natural language understanding, action planning, and structured output generation.

## Components

### LLM Planning

The LLM planning module converts natural language commands into structured actions:

- **LLMPlanner Class**: Uses OpenAI's GPT-4o with function calling
- **Supported Actions**:
  - `move_to_position`: Move the robot to specific coordinates
  - `grasp_object`: Grasp an object at a specific location or identified by description
  - `navigate_to`: Navigate to a named location or relative position
  - `detect_object`: Detect and identify objects in the field of view
  - `manipulate_object`: Perform manipulation actions on objects

```python
from simulation_assets.vla.cognitive_core.llm_planning import LLMPlanner

# Initialize planner
planner = LLMPlanner(api_key="your-api-key", model="gpt-4o")

# Plan actions for a command
vision_context = {
    "objects": [
        {"id": "obj_001", "class": "apple", "color": "red", "position": {"x": 1.2, "y": 0.5, "z": 0.8}}
    ]
}

action_plan = await planner.plan_actions("Pick up the red apple", vision_context)
print(action_plan)
```

### JSON Validation

The JSON validator ensures LLM outputs conform to expected ROS 2 action schemas:

- **JSONValidator Class**: Validates action arguments against schemas
- **Safety Constraints**:
  - Position limits (x, y, z coordinates)
  - Forbidden actions (harm, damage, etc.)
  - Coordinate precision limits

```python
from simulation_assets.vla.cognitive_core.json_validator import JSONValidator

# Initialize validator
validator = JSONValidator()

# Validate an action
result = validator.validate_action_json("move_to_position", {"x": 1.5, "y": 2.0, "z": 0.0})
print(f"Valid: {result['valid']}, Errors: {result['errors']}")
```

### Safety Guardrails

The safety guardrails module prevents dangerous robot actions:

- **SafetyGuardrails Class**: Implements semantic analysis and action validation
- **Risk Levels**:
  - `LOW`: Actions that are safe to execute
  - `MEDIUM`: Actions that require monitoring
  - `HIGH`: Actions requiring human confirmation
  - `CRITICAL`: Actions that are blocked

```python
from simulation_assets.vla.cognitive_core.safety_guardrails import SafetyGuardrails, RiskLevel

# Initialize guardrails
guardrails = SafetyGuardrails(risk_threshold=0.8, confirmation_required_risk=0.6)

# Validate an action
action = {
    "function": "move_to_position",
    "arguments": {"x": 1.0, "y": 1.0, "z": 0.5}
}

result = guardrails.validate_action(action)
print(f"Valid: {result['valid']}, Risk: {result['risk_level'].value}")
```

## Function Calling Schema

The LLM uses function calling with predefined schemas for each ROS action:

```json
{
  "name": "move_to_position",
  "description": "Move the robot to a specific position",
  "parameters": {
    "type": "object",
    "properties": {
      "x": {"type": "number", "description": "X coordinate"},
      "y": {"type": "number", "description": "Y coordinate"},
      "z": {"type": "number", "description": "Z coordinate"},
      "frame_id": {"type": "string", "description": "Reference frame for the coordinates"}
    },
    "required": ["x", "y", "frame_id"]
  }
}
```

## Configuration

The cognitive core can be configured through environment variables:

```env
OPENAI_API_KEY=your_openai_api_key
VLA_LLM_MODEL=gpt-4o
VLA_LLM_TEMPERATURE=0.1
VLA_LLM_MAX_TOKENS=500
VLA_RISK_THRESHOLD=0.8
VLA_CONFIRMATION_REQUIRED=false
```

## Best Practices

1. **Prompt Engineering**: Use clear, specific prompts to guide the LLM toward desired actions
2. **Function Calling**: Leverage function calling for reliable structured output
3. **Safety First**: Always validate actions through safety guardrails before execution
4. **Context Integration**: Provide rich vision context to enable accurate grounding
5. **Error Handling**: Implement robust error handling for API failures

## Troubleshooting

- **Unexpected Actions**: Review prompt engineering and function schemas
- **Safety Blocks**: Check action parameters against safety constraints
- **API Errors**: Verify API key validity and check rate limits
- **Grounding Failures**: Ensure vision context is properly provided to the planner

## Next Steps

After implementing the cognitive core, proceed to Chapter 3: Vision-Language Grounding to connect linguistic descriptions to visual objects in the environment.