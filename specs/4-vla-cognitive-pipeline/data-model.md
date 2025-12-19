# Data Model: Module 4: Vision-Language-Action (VLA)

**Feature**: 4-vla-cognitive-pipeline
**Created**: 2025-12-18
**Status**: Complete

## Key Entities

### VoiceCommand
- **Description**: A natural language instruction spoken by the user
- **Fields**:
  - `id`: Unique identifier for the command
  - `timestamp`: When the command was captured
  - `audio_data`: Raw audio data (or reference)
  - `transcript`: Transcribed text from voice
  - `confidence`: Confidence score of transcription (0.0-1.0)
  - `language`: Detected language code
  - `user_intent`: Parsed intent from LLM
  - `objects_referenced`: List of object references in command

### CognitivePipeline
- **Description**: The processing flow that transforms voice input into robot actions
- **Fields**:
  - `id`: Unique identifier for the pipeline execution
  - `voice_command_id`: Reference to source VoiceCommand
  - `voice_to_text_time`: Processing time for transcription
  - `language_understanding_time`: Processing time for intent parsing
  - `vision_grounding_time`: Processing time for object identification
  - `action_planning_time`: Processing time for action generation
  - `total_processing_time`: Total time from voice to action
  - `status`: Current status (processing, completed, failed, cancelled)
  - `error_message`: Error details if processing failed

### VisionLanguageGrounding
- **Description**: The connection between linguistic descriptions and visual objects
- **Fields**:
  - `id`: Unique identifier for the grounding
  - `cognitive_pipeline_id`: Reference to parent pipeline
  - `object_description`: Text description from language
  - `visual_objects`: List of identified objects in scene
  - `selected_object`: The specific object chosen based on description
  - `confidence_score`: Confidence in object selection (0.0-1.0)
  - `bounding_box`: Coordinates of selected object in image
  - `visual_context`: Additional scene context provided to LLM

### ActionPlan
- **Description**: A structured sequence of ROS 2 actions implementing the user's request
- **Fields**:
  - `id`: Unique identifier for the action plan
  - `cognitive_pipeline_id`: Reference to parent pipeline
  - `actions`: Array of structured ROS 2 action definitions
  - `execution_order`: Order in which actions should be executed
  - `validation_status`: Whether actions pass safety checks
  - `estimated_duration`: Estimated time to complete all actions
  - `required_resources`: List of robot resources needed
  - `dependencies`: Action dependencies and preconditions

### SafetyGuardrails
- **Description**: Validation rules preventing dangerous commands
- **Fields**:
  - `id`: Unique identifier for the safety check
  - `action_plan_id`: Reference to action plan being validated
  - `check_type`: Type of safety check (physical, environmental, social)
  - `risk_level`: Assessed risk level (low, medium, high, critical)
  - `validation_result`: Pass/fail status of the check
  - `safety_violations`: List of specific violations detected
  - `override_required`: Whether human override is needed
  - `safety_confidence`: Confidence in safety assessment (0.0-1.0)

## State Transitions

### VoiceCommand States
- `captured` → `transcribing` → `transcribed` → `processing` → `completed` | `failed`

### CognitivePipeline States
- `initialized` → `voice_processing` → `language_processing` → `vision_processing` → `action_planning` → `executing` → `completed` | `failed` | `cancelled`

### ActionPlan States
- `planned` → `validated` → `approved` → `executing` → `completed` | `failed` | `interrupted`

## Validation Rules

### VoiceCommand Validation
- Transcript must not be empty
- Confidence score must be > 0.5 for processing
- Language must be supported
- Command must contain actionable intent

### VisionLanguageGrounding Validation
- At least one object must be identified in scene
- Confidence score must be > 0.7 for action
- Object must be reachable by robot
- Object must not be in dangerous location

### ActionPlan Validation
- All actions must be valid ROS 2 commands
- Safety guardrails must pass
- Robot must have required capabilities
- Actions must not exceed safety limits

### SafetyGuardrails Validation
- Risk level must be low or medium for automatic execution
- High risk requires human confirmation
- Critical risk blocks execution entirely
- All safety violations must be logged