---
id: red-002
title: Fix API Key Loading in Agent
stage: red
date: 2025-12-26
surface: API Key Loading Fix
model: Claude Sonnet 4.5
feature: 004-chat-ui-integration
branch: 004-chat-ui-integration
user: asada
command: sp.implement
labels: [backend, api-key, environment, initialization]
links:
  spec: ../specs/004-chat-ui-integration/spec.md
  ticket:
  adr:
  pr:
files:
  - path: backend/agent.py
    change: Added load_dotenv() and API key verification print statements
tests:
  - name: Environment variable loading
    result: PASS
  - name: API key availability check
    result: PASS
---

## Prompt

TASK: Fix API Key loading in Agent.

1. In `backend/agent.py`, ensure `load_dotenv()` is called at the very top.
2. Check if the Cohere client is initialized using `os.getenv("COHERE_API_KEY")`.
3. Add a print statement like `print(f"Key loaded: {bool(os.getenv('COHERE_API_KEY'))}")` to verify keys on startup.

## Response snapshot

Successfully implemented fixes for API key loading in the Agent:

### 1. Added load_dotenv() at the top of agent.py
- Imported load_dotenv from dotenv
- Added load_dotenv() call to ensure environment variables are loaded
- This ensures the .env file is read when the module is imported

### 2. Added API key verification print statements
- Added print statements to verify that all required API keys are loaded:
  - OpenRouter API key
  - Cohere API key (used by retrieve_context function)
  - Qdrant URL
  - Qdrant API key

### 3. Maintained existing functionality
- The Cohere API key is used by the retrieve_context function in retrieve.py
- The agent.py file correctly checks for all required environment variables
- No breaking changes to existing functionality

## Outcome

- ✅ Impact: Ensured proper loading of environment variables and API keys in the agent module
- 🧪 Tests: All environment variables are now properly loaded and verified at startup
- 📁 Files: 1 file modified to improve API key loading
- 🔁 Next prompts: Ready for testing and deployment
- 🧠 Reflection: The implementation now provides visibility into API key loading status at startup

## Evaluation notes (flywheel)

- Failure modes observed: Previously, if .env wasn't loaded, API keys would be missing causing runtime errors
- Graders run and results (PASS/FAIL): PASS - All environment variables now loaded properly
- Prompt variant (if applicable): Standard API key loading fix
- Next experiment (smallest change to try): Add more comprehensive startup validation