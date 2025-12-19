# Tasks: Module 4: Vision-Language-Action (VLA)

**Input**: Design documents from `/specs/4-vla-cognitive-pipeline/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation**: `docs/module-04-vla-capstone/`
- **Implementation assets**: `simulation-assets/vla/`
- **Configuration**: `simulation-assets/vla/config/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure per implementation plan in docs/module-04-vla-capstone/
- [x] T002 [P] Create directory structure for simulation-assets/vla/{voice_interface,cognitive_core,vision_grounding,capstone_integration,config}
- [x] T003 Create .env file structure for OpenAI API keys management

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Define Python requirements in simulation-assets/vla/requirements.txt
- [x] T005 [P] Create configuration files in simulation-assets/vla/config/{openai_config.yaml,vla_parameters.yaml}
- [x] T006 Set up basic project documentation structure in docs/module-04-vla-capstone/

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Voice Command to Robot Action (Priority: P1) 🎯 MVP

**Goal**: Enable a user to speak a command like "Pick up the red bottle" and have the robot identify the object and execute the manipulation task within 5 seconds

**Independent Test**: Speaking voice commands to the system and observing if the robot correctly identifies the target object and begins the appropriate manipulation action within the latency requirement

### Implementation for User Story 1

- [x] T007 [P] Create audio capture module with VAD in simulation-assets/vla/voice_interface/audio_capture.py
- [x] T008 [P] Create Whisper transcription module in simulation-assets/vla/voice_interface/whisper_transcription.py
- [x] T009 [P] Create VAD detector module in simulation-assets/vla/voice_interface/vad_detector.py
- [x] T010 Create voice interface main module in simulation-assets/vla/voice_interface/__init__.py
- [x] T011 [P] Create LLM planning module in simulation-assets/vla/cognitive_core/llm_planning.py
- [x] T012 [P] Create JSON validator module in simulation-assets/vla/cognitive_core/json_validator.py
- [x] T013 Create cognitive core main module in simulation-assets/vla/cognitive_core/__init__.py
- [x] T014 [P] Create object detector module in simulation-assets/vla/vision_grounding/object_detector.py
- [x] T015 [P] Create grounding pipeline module in simulation-assets/vla/vision_grounding/grounding_pipeline.py
- [x] T016 Create vision grounding main module in simulation-assets/vla/vision_grounding/__init__.py
- [x] T017 Create safety guardrails module in simulation-assets/vla/cognitive_core/safety_guardrails.py
- [x] T018 Create main pipeline orchestrator in simulation-assets/vla/capstone_integration/main_pipeline.py
- [x] T019 Create latency benchmarking tool in simulation-assets/vla/capstone_integration/latency_benchmark.py
- [x] T020 Create basic documentation for Chapter 1 in docs/module-04-vla-capstone/01-voice-interface.md
- [x] T021 Create basic documentation for Chapter 2 in docs/module-04-vla-capstone/02-cognitive-brain.md
- [x] T022 Create basic documentation for Chapter 3 in docs/module-04-vla-capstone/03-vision-grounding.md
- [x] T023 Create basic documentation for Chapter 4 in docs/module-04-vla-capstone/04-capstone-autonomous.md

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Vision-Language Grounding for Object Identification (Priority: P2)

**Goal**: Enable the robot to correctly identify and select a specific object among multiple similar objects based on language description

**Independent Test**: Placing multiple similar objects in the robot's field of view and having the user specify which one to interact with, verifying the robot selects the correct object

### Implementation for User Story 2

- [x] T024 Enhance object detector for multi-object scenarios in simulation-assets/vla/vision_grounding/object_detector.py
- [x] T025 Update grounding pipeline for disambiguation in simulation-assets/vla/vision_grounding/grounding_pipeline.py
- [x] T026 Update LLM planning for complex object references in simulation-assets/vla/cognitive_core/llm_planning.py
- [x] T027 Update main pipeline for enhanced object selection in simulation-assets/vla/capstone_integration/main_pipeline.py
- [x] T028 Create advanced vision grounding documentation in docs/module-04-vla-capstone/03-vision-grounding.md
- [x] T029 Update capstone documentation for multi-object scenarios in docs/module-04-vla-capstone/04-capstone-autonomous.md

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Cognitive Core Planning (Priority: P3)

**Goal**: Convert complex natural language commands into structured robot actions that can be executed by ROS 2 action servers, including multi-step planning and safety validation

**Independent Test**: Giving the system complex commands like "Clean the room by picking up the red toys and placing them in the blue bin", verifying it breaks this into appropriate action sequences

### Implementation for User Story 3

- [x] T030 Enhance LLM planning for multi-step commands in simulation-assets/vla/cognitive_core/llm_planning.py
- [ ] T031 Update JSON validator for action sequences in simulation-assets/vla/cognitive_core/json_validator.py
- [ ] T032 Update safety guardrails for multi-step actions in simulation-assets/vla/cognitive_core/safety_guardrails.py
- [ ] T033 Update main pipeline for multi-step execution in simulation-assets/vla/capstone_integration/main_pipeline.py
- [ ] T034 Create advanced cognitive brain documentation in docs/module-04-vla-capstone/02-cognitive-brain.md
- [ ] T035 Update capstone documentation for multi-step scenarios in docs/module-04-vla-capstone/04-capstone-autonomous.md

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T036 [P] Create module summary documentation in docs/module-04-vla-capstone/module-summary-next-steps.md
- [x] T037 [P] Create quickstart guide in docs/module-04-vla-capstone/quickstart.md
- [x] T038 [P] Create glossary documentation in docs/module-04-vla-capstone/glossary.md
- [x] T039 [P] Create safety documentation in docs/module-04-vla-capstone/safety.md
- [x] T040 [P] Create troubleshooting documentation in docs/module-04-vla-capstone/troubleshooting.md
- [x] T041 Create integration tests in simulation-assets/vla/capstone_integration/integration_tests.py
- [x] T042 Run end-to-end validation tests for VLA system
- [x] T043 Update configuration parameters for optimal performance
- [x] T044 Final documentation review and polish

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 components
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Builds on US1/US2 components

### Within Each User Story

- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- All models within a story marked [P] can run in parallel
- Documentation tasks can run in parallel with implementation

---

## Parallel Example: User Story 1

```bash
# Launch all voice interface modules together:
Task: "Create audio capture module with VAD in simulation-assets/vla/voice_interface/audio_capture.py"
Task: "Create Whisper transcription module in simulation-assets/vla/voice_interface/whisper_transcription.py"
Task: "Create VAD detector module in simulation-assets/vla/voice_interface/vad_detector.py"

# Launch all cognitive core modules together:
Task: "Create LLM planning module in simulation-assets/vla/cognitive_core/llm_planning.py"
Task: "Create JSON validator module in simulation-assets/vla/cognitive_core/json_validator.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Each story adds value without breaking previous stories

---

## Notes

- [P] tasks = different files, no dependencies
- [US1], [US2], [US3] labels map task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently