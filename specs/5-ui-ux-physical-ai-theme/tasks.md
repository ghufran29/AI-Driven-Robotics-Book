# Tasks: UI/UX Upgrade: The Physical AI Theme

**Input**: Design documents from `/specs/5-ui-ux-physical-ai-theme/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation**: `specs/5-ui-ux-physical-ai-theme/`
- **Implementation assets**: `frontend/src/`, `frontend/static/`, `frontend/docusaurus.config.js`
- **Configuration**: `frontend/docusaurus.config.js`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create directory structure for frontend/src/{css,components,pages,theme}
- [x] T002 Create directory structure for frontend/static/{img,css}
- [x] T003 [P] Create basic CSS structure in frontend/src/css/custom.css

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T004 Configure docusaurus.config.js to load Inter and JetBrains Mono fonts
- [x] T005 [P] Define basic CSS color variables in frontend/src/css/custom.css
- [x] T006 [P] Set default dark mode in docusaurus.config.js theme settings
- [x] T007 [P] Create typography CSS variables in frontend/src/css/custom.css
- [x] T008 Create base spacing system in frontend/src/css/custom.css

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Enhanced Dark Mode Theme (Priority: P1) 🎯 MVP

**Goal**: Implement professional dark mode interface with cyber-physical aesthetics (deep slate background with electric blue accents) so that students and researchers can read technical documentation comfortably for extended periods without eye strain

**Independent Test**: User can navigate the site with the new dark theme active, see proper contrast ratios, and read code blocks with enhanced syntax highlighting without eye strain

### Implementation for User Story 1

- [x] T009 [P] [US1] Implement deep slate background color (#2a2f35) in CSS variables
- [x] T010 [P] [US1] Implement electric blue accent color (#00eeff) in CSS variables
- [x] T011 [P] [US1] Implement neon green accent color (#39ff14) in CSS variables
- [x] T012 [P] [US1] Update code block background to #1e1e1e in custom.css
- [x] T013 [P] [US1] Update text colors (light #e6e6e6, medium #a6a6a6) in CSS variables
- [x] T014 [P] [US1] Implement semantic colors for admonitions in custom.css
- [x] T015 [US1] Override Prism.js theme for dark mode syntax highlighting
- [x] T016 [US1] Test dark mode contrast ratios meet WCAG AA standards
- [x] T017 [US1] Implement light mode fallback with proper readability
- [x] T018 [US1] Test theme switching functionality between light and dark modes

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Professional Landing Page Redesign (Priority: P2)

**Goal**: Create a compelling landing page with a commanding title "Bridging Digital Brains & Physical Bodies" and clear feature highlights so that new visitors understand the course value proposition immediately

**Independent Test**: Visitor lands on the homepage and can immediately identify the three main course components (ROS 2, Simulation, GenAI) with clear visual icons and a prominent call-to-action

### Implementation for User Story 2

- [x] T019 [P] [US2] Create SVG icons for ROS 2, Simulation, and GenAI in static/img/
- [x] T020 [P] [US2] Create HomepageFeatures component in src/components/
- [x] T021 [US2] Update index.js to implement "Bridging Digital Brains & Physical Bodies" title
- [x] T022 [US2] Implement 3-column feature block layout in HomepageFeatures
- [x] T023 [US2] Style feature blocks with cyber-physical aesthetic
- [x] T024 [US2] Create prominent "Start the Course" CTA button with hover effects
- [x] T025 [US2] Implement CSS-based grid/node background pattern for hero section
- [x] T026 [US2] Test responsive layout collapses to single column on mobile
- [x] T027 [US2] Validate SVG icons scale properly across screen sizes

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Improved Documentation Reading Experience (Priority: P3)

**Goal**: Enhance sidebar navigation, improve code block presentation, and enhance table of contents so that learners can efficiently navigate and understand complex technical content

**Independent Test**: User can navigate through documentation with improved sidebar indicators, clearly see code syntax highlighting, and use the right sidebar TOC for quick navigation

### Implementation for User Story 3

- [x] T028 [P] [US3] Customize sidebar scrollbar styling with electric blue thumb
- [x] T029 [P] [US3] Implement clear active-state indicators in documentation sidebar
- [x] T030 [US3] Enhance code block syntax highlighting for better readability
- [x] T031 [US3] Improve floating right sidebar for table of contents navigation
- [x] T032 [US3] Implement custom admonition styling for technical readouts
- [x] T033 [US3] Test navigation efficiency improvements on wide screens
- [x] T034 [US3] Validate responsive behavior for TOC sidebar on smaller screens

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## Phase 6: User Story 4 - Modern Navigation & Footer (Priority: P4)

**Goal**: Implement modern glassmorphism navbar and professional footer with proper credits so that the site feels contemporary and trustworthy

**Independent Test**: User can navigate the site with the glassmorphism navbar and find appropriate footer information with social links and copyright

### Implementation for User Story 4

- [x] T035 [P] [US4] Implement glassmorphism effect for navbar using backdrop-filter
- [x] T036 [P] [US4] Create professional footer layout with University/Lab credits
- [x] T037 [US4] Add social links (GitHub/LinkedIn) to footer
- [x] T038 [US4] Implement copyright information in footer
- [x] T039 [US4] Test glassmorphism compatibility with older browsers
- [x] T040 [US4] Validate navigation consistency across all pages
- [x] T041 [US4] Test footer responsive behavior on mobile devices

**Checkpoint**: All user stories should now be independently functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T042 [P] Implement accessibility enhancements for all components
- [x] T043 [P] Optimize SVG assets for performance (under 2KB each)
- [x] T044 [P] Add smooth transitions and micro-interactions per design system
- [x] T045 [P] Implement responsive breakpoints for all components
- [x] T046 [P] Add focus states with electric blue outline for accessibility
- [x] T047 [P] Optimize font loading with preconnect and font-display: swap
- [x] T048 Run Lighthouse audit to ensure performance score >90
- [x] T049 Run accessibility audit to ensure WCAG AA compliance
- [x] T050 Update documentation for the new theme implementation
- [x] T051 Test cross-browser compatibility (Chrome, Firefox, Safari, Edge)
- [x] T052 Final validation of all user stories working together

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed sequentially in priority order (P1 → P2 → P3 → P4)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 components
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Builds on US1/US2 components
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Builds on US1/US2/US3 components

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
# Launch all color variable tasks together:
Task: "Implement deep slate background color (#2a2f35) in CSS variables"
Task: "Implement electric blue accent color (#00eeff) in CSS variables"
Task: "Implement neon green accent color (#39ff14) in CSS variables"
Task: "Update code block background to #1e1e1e in custom.css"

# Launch all text color tasks together:
Task: "Update text colors (light #e6e6e6, medium #a6a6a6) in CSS variables"
Task: "Implement semantic colors for admonitions in custom.css"
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
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

---

## Notes

- [P] tasks = different files, no dependencies
- [US1], [US2], [US3], [US4] labels map task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently