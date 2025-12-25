---
id: phr-004-plan-20251224
title: Create implementation plan
stage: plan
date: 2025-12-24
surface: skill
model: Claude Sonnet 4.5
feature: chat-ui-integration
branch: 004-chat-ui-integration
user: asada
command: /sp.plan
labels: [planning, chat, ui, docusaurus]
links:
  spec: specs/004-chat-ui-integration/spec.md
  plan: specs/004-chat-ui-integration/plan.md
  ticket:
  adr:
  pr:
files:
- path: specs/004-chat-ui-integration/plan.md
  action: created
  purpose: Implementation plan for chat UI integration feature
- path: specs/004-chat-ui-integration/research.md
  action: created
  purpose: Research findings for implementation approach
- path: specs/004-chat-ui-integration/data-model.md
  action: created
  purpose: Data model for chat UI entities
- path: specs/004-chat-ui-integration/quickstart.md
  action: created
  purpose: Quickstart guide for the feature
tests:

---
## Prompt

$ARGUMENTS

Implement Docusaurus Chat UI Integration

TASK: Plan the implementation of a floating Chat Bubble in Docusaurus based on 'specs/004-chat-ui-integration/spec.md'.

Requirements for the Plan:

1. **Frontend Architecture**:
   - Create a React component `ChatBubble` in `doc-book/src/components/`.
   - Define state management for messages (user & AI), loading status, and toggle (open/close).
   - Use `lucide-react` or similar for the chat icon.

2. **API Communication**:
   - Implement an async function to call `http://localhost:8000/ask` using the browser's `fetch` API.
   - Handle response parsing and error scenarios (e.g., backend offline).

3. **Global Integration**:
   - Determine the best way to inject the component globally (e.g., wrapping the Docusaurus `Layout` or using a `ThemeConfig` feature) so it appears on all pages.

4. **Styling & UX**:
   - Design a responsive floating UI (bottom-right corner).
   - Add a "Thinking..." indicator while waiting for the AI response.
   - Ensure Markdown rendering for AI answers (suggest using `react-markdown`).

5. **Dependencies**: Identify any npm packages needed (e.g., `react-markdown`, `clsx`, `tailwind-merge` if using Tailwind).

## Response snapshot

# Implementation Plan: Docusaurus Chat UI Integration

**Branch**: `004-chat-ui-integration` | **Date**: 2025-12-24 | **Spec**: [specs/004-chat-ui-integration/spec.md](../specs/004-chat-ui-integration/spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a floating Chat Bubble component for the Docusaurus site that communicates with the FastAPI backend. The implementation will include creating a React component with state management, API communication, responsive design, and proper integration with the Docusaurus layout.

## Technical Context

**Language/Version**: TypeScript 4.x, React 18.x
**Primary Dependencies**: Docusaurus 3.x, React, react-markdown, lucide-react
**Storage**: API communication with backend service at http://localhost:8000/ask
**Testing**: Manual verification in browser
**Target Platform**: Web browser
**Project Type**: Frontend UI component
**Performance Goals**: API responses within 15 seconds for 90% of requests, responsive UI
**Constraints**: Must be responsive, not block documentation text, handle loading/error states, use TypeScript
**Scale/Scope**: Single component integrated globally across all documentation pages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Spec-Driven Development Compliance
- [x] All implementation follows the Spec-Kit Plus workflow: specs → plans → tasks → implementation
- [x] Feature has a corresponding specification in `/specs/004-chat-ui-integration/spec.md`
- [x] Plan aligns with the established specification before proceeding

### Technical Accuracy Verification
- [x] Implementation maintains high technical precision with ROS 2 (rclpy), URDF, Gazebo/Unity, and NVIDIA Isaac platforms
- [x] Content reflects current best practices in robotics and AI systems
- [x] All technical claims are verifiable and accurate

### Modularity & Reusability Assessment
- [x] Components designed to be modular and reusable across different modules
- [x] Leverages Claude Code Subagents and Agent Skills where appropriate
- [x] Code structure supports maintainability and extension

### Docusaurus-First Architecture Alignment
- [x] All content generated inside `doc-book/docs/` using Docusaurus 3.x with TypeScript
- [x] Maintains compatibility with GitHub Pages deployment
- [x] Follows Docusaurus 3.x standards and conventions

## Project Structure

### Documentation (this feature)

```text
specs/004-chat-ui-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
doc-book/
├── src/
│   └── components/
│       └── ChatBubble.tsx    # New chat component with TypeScript
├── package.json             # Dependencies for react-markdown, lucide-react, etc.
└── docusaurus.config.js     # Configuration for global component integration
```

**Structure Decision**: New React component in doc-book/src/components/ with proper TypeScript typing and integration into Docusaurus layout.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

## Outcome

- ✅ Impact: Created complete implementation plan for chat UI integration feature
- 🧪 Tests: No automated tests created yet, will be implemented during development
- 📁 Files: Created plan.md, research.md, data-model.md, and quickstart.md in specs/004-chat-ui-integration/
- 🔁 Next prompts: Ready for task generation with /sp.tasks
- 🧠 Reflection: Plan comprehensively covers the implementation approach with proper research and data modeling

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable at plan stage
- Prompt variant (if applicable):
- Next experiment (smallest change to try): Generate tasks from this plan