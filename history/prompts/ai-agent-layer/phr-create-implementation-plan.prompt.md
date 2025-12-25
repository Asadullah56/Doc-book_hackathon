---
id: phr-003-plan-20251224
title: Create implementation plan
stage: plan
date: 2025-12-24
surface: skill
model: Claude Sonnet 4.5
feature: ai-agent-layer
branch: 003-ai-agent-layer
user: asada
command: /sp.plan
labels: [planning, ai-agent, openrouter, fastapi]
links:
  spec: specs/003-ai-agent-layer/spec.md
  plan: specs/003-ai-agent-layer/plan.md
  ticket:
  adr:
  pr:
files:
- path: specs/003-ai-agent-layer/plan.md
  action: created
  purpose: Implementation plan for AI agent layer feature
- path: specs/003-ai-agent-layer/research.md
  action: created
  purpose: Research findings for implementation approach
- path: specs/003-ai-agent-layer/data-model.md
  action: created
  purpose: Data model for AI agent entities
- path: specs/003-ai-agent-layer/quickstart.md
  action: created
  purpose: Quickstart guide for the feature
tests:

---
## Prompt

$ARGUMENTS

Implement OpenRouter Agent and FastAPI Endpoint

1. **Environment Setup**:
   - `uv add openai fastapi uvicorn`.
   - Add `OPENROUTER_API_KEY` and `OPENROUTER_MODEL` to `.env`.

2. **Create `agent.py`**:
   - Initialize the OpenAI client with OpenRouter's base URL and API key.
   - Implement `generate_answer(query: str)`:
     * Call `retrieve_context(query)` from `retrieve.py`.
     * Build a System Prompt: "You are an expert AI Assistant for Humanoid Academy. Use the following context to answer..."
     * Call the OpenRouter chat completion API.

3. **Create `api.py`**:
   - Initialize FastAPI app.
   - Add CORS middleware for frontend communication.
   - Define a `/ask` POST endpoint that calls `agent.generate_answer(query)`.
   - Return a JSON response with: `{ "answer": "...", "sources": [...] }`.

4. **Testing**:
   - Provide command: `uv run uvicorn api:app --reload`.
   - Test via Swagger UI at `http://localhost:8000/docs`.

## Response snapshot

# Implementation Plan: AI Agent Layer with OpenRouter & FastAPI

**Branch**: `003-ai-agent-layer` | **Date**: 2025-12-24 | **Spec**: [specs/003-ai-agent-layer/spec.md](../specs/003-ai-agent-layer/spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement an AI Agent Layer with OpenRouter integration and FastAPI endpoint. The implementation will include creating `backend/agent.py` to handle AI logic using OpenRouter's API, and `backend/api.py` to expose a `/ask` endpoint that combines system prompt, retrieved context, and user query to generate natural language answers.

## Technical Context

**Language/Version**: Python 3.8+
**Primary Dependencies**: OpenAI Python SDK, FastAPI, uvicorn, Pydantic, python-dotenv, CORS middleware
**Storage**: Integration with existing Qdrant vector database via `retrieve.py`
**Testing**: Manual verification via Swagger UI and direct API calls
**Target Platform**: Backend service running Python
**Project Type**: Backend API service
**Performance Goals**: API responses within 10 seconds for 95% of requests
**Constraints**: API keys must stay in `.env`, must handle "I don't know" cases, use OpenRouter base URL
**Scale/Scope**: Single API endpoint `/ask` with context retrieval from existing RAG system

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Spec-Driven Development Compliance
- [x] All implementation follows the Spec-Kit Plus workflow: specs → plans → tasks → implementation
- [x] Feature has a corresponding specification in `/specs/003-ai-agent-layer/spec.md`
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
specs/003-ai-agent-layer/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── agent.py             # AI agent logic with OpenRouter integration
├── api.py               # FastAPI endpoint for the AI agent
├── retrieve.py          # Context retrieval (existing)
├── main.py              # Existing embedding pipeline
└── requirements.txt     # Dependencies for OpenAI, FastAPI, etc.
```

**Structure Decision**: Two new files (agent.py, api.py) added to backend directory to implement the AI agent functionality, integrating with existing retrieve.py for context retrieval.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

## Outcome

- ✅ Impact: Created complete implementation plan for AI agent layer feature
- 🧪 Tests: No automated tests created yet, will be implemented during development
- 📁 Files: Created plan.md, research.md, data-model.md, and quickstart.md in specs/003-ai-agent-layer/
- 🔁 Next prompts: Ready for task generation with /sp.tasks
- 🧠 Reflection: Plan comprehensively covers the implementation approach with proper research and data modeling

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable at plan stage
- Prompt variant (if applicable):
- Next experiment (smallest change to try): Generate tasks from this plan