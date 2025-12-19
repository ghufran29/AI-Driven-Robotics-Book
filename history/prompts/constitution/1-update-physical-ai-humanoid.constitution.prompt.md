---
id: 1
title: Update Physical AI Humanoid Constitution
stage: constitution
date: 2025-12-15
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: master
user: user
command: "$ARGUMENTS"
labels: [constitution, ai, robotics, education]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

Act as a System Architect and Project Lead using Spec-Kit Plus. Generate a strict `sp.constitution` file for a project titled **"Physical AI & Humanoid Robotics: Bridging Digital & Physical Worlds"**.

**Reference Style:**
Use the following structure and tone:
- **Core Principles:** (Focus on Safety, Accuracy, Reproducibility)
- **Key Standards:** (Code verification, Citation styles, Hardware specifics)
- **Constraints:** (Tech stack, Word counts, Output formats)
- **Success Criteria:** (Deployment checks, Functional requirements)

**Project Context & Requirements to Embed:**
1.  **Deliverables:**
    * **Book:** "AI-Native Software Development" style textbook using Docusaurus & Spec-Kit Plus. Deploy to GitHub Pages.
    * **RAG Chatbot:** Integrated into the book using OpenAI Agents, FastAPI, Neon Serverless Postgres, and Qdrant Cloud.

2.  **Curriculum Scope:**
    * **Theme:** Embodied Intelligence, ROS 2, Gazebo, Unity, NVIDIA Isaac Sim/ROS, VLA (Vision-Language-Action).
    * **Hardware:** Specific constraints for NVIDIA RTX Workstations, Jetson Orin Nano (Edge), and Unitree/RealSense hardware.

3.  **Governance Rules (Critical):**
    * **Accuracy:** All hardware specifications (GPU VRAM, voltage) and ROS 2 commands must be verified against official documentation (NVIDIA/ROS 2 Humble).
    * **Code Integrity:** Python/C++ code snippets must be syntactically correct and safe for physical robot deployment (safety-critical).
    * **Pedagogy:** Content must bridge the gap between "Simulation" and "Real World" (Sim-to-Real transfer).
    * **Citations:** APA style for theoretical concepts; direct documentation links for technical implementations.

**Output:**
Generate only the `sp.constitution` file content. Ensure the tone is authoritative, academic, and technically rigorous.

## Response snapshot

Updated the constitution file at .specify/memory/constitution.md with project-specific principles for the Physical AI & Humanoid Robotics project, focusing on safety-first design, accuracy & verification, simulation-to-reality transfer, technical rigor, pedagogical excellence, and documentation standards.

## Outcome

- ✅ Impact: Created comprehensive constitution document with 6 core principles, technology constraints, workflow standards, and governance rules for the Physical AI & Humanoid Robotics project
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with project-specific governance principles
- 🔁 Next prompts: Project constitution is established for future development
- 🧠 Reflection: Constitution addresses safety, accuracy, and pedagogical requirements specific to physical AI and robotics applications

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - Constitution properly created with all required elements
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A