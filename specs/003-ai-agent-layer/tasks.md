# Tasks: AI Agent Layer with OpenRouter & FastAPI

**Feature**: AI Agent Layer with OpenRouter & FastAPI
**Branch**: `003-ai-agent-layer`
**Created**: 2025-12-24
**Plan**: [plan.md](plan.md)
**Spec**: [spec.md](spec.md)

## Implementation Strategy

**MVP Scope**: Implement the core AI agent functionality with OpenRouter integration and a FastAPI endpoint. This includes the agent logic, API endpoint, and basic testing to verify the flow works.

**Delivery Approach**: Incremental delivery with each user story building upon the previous. User Story 1 (P1) provides the core functionality, User Story 2 (P1) ensures proper context retrieval, and User Story 3 (P2) adds the API layer.

## Dependencies

- **User Story 1 depends on**: Phase 1 (Setup) and Phase 2 (Foundational) tasks
- **User Story 2 depends on**: User Story 1 tasks
- **User Story 3 depends on**: User Story 1 and User Story 2 tasks
- **Final Testing depends on**: All user stories completed

## Parallel Execution Opportunities

- T002 [P] and T003 [P]: Dependency installation and environment variable setup can run in parallel
- T102 [P] [US1] and T103 [P] [US1]: OpenAI client setup and context retrieval integration can be developed in parallel
- T202 [P] [US2] and T203 [P] [US3]: US2 and US3 core implementations can be developed in parallel after US1 completion

## Phase 1: Setup

- [X] T001 Create feature branch 003-ai-agent-layer from main
- [X] T002 [P] Install dependencies: uv add openai fastapi uvicorn python-dotenv pydantic
- [X] T003 [P] Ensure .env contains OPENROUTER_API_KEY and OPENROUTER_MODEL

## Phase 2: Foundational

- [X] T011 [P] Verify retrieve.py exists and has retrieve_context function
- [X] T012 Create OpenAI client initialization function that configures OpenRouter base URL

## Phase 3: User Story 1 - Query Processing (Priority: P1)

**Goal**: Implement core AI agent functionality that processes user queries by augmenting them with context retrieved from retrieve.py.

**Independent Test Criteria**: Can be fully tested by calling the generate_answer function with a sample query and verifying that it retrieves relevant context and returns a natural language answer.

**Acceptance Scenarios**:
1. Given a user submits a query about documentation content, When the AI agent processes the request, Then it retrieves relevant context and returns an accurate answer based on that context.
2. Given a user submits a query with insufficient context in the documentation, When the AI agent processes the request, Then it responds with "I don't know" or similar indication that the information is not available.

- [X] T101 [US1] Create backend/agent.py file
- [X] T102 [P] [US1] Initialize OpenAI client with OpenRouter base URL and API key
- [X] T103 [P] [US1] Import retrieve_context function from retrieve.py
- [X] T104 [US1] Implement generate_answer(query: str) function that combines system prompt, context, and user query
- [X] T105 [US1] Add "I don't know" handling for cases with no relevant context

## Phase 4: User Story 2 - Context Retrieval Integration (Priority: P1)

**Goal**: Ensure the AI agent retrieves relevant context from the documentation before processing user queries, so that the responses are grounded in the actual content.

**Independent Test Criteria**: Can be tested by verifying that the agent calls the retrieve_context function and uses the returned context in its response generation.

**Acceptance Scenarios**:
1. Given a user query is received, When the agent processes it, Then it retrieves relevant context using the retrieve_context function from retrieve.py.

- [X] T201 [US2] Update generate_answer to properly format retrieved context for the AI model
- [X] T202 [P] [US2] Ensure system prompt properly incorporates retrieved context
- [X] T203 [P] [US2] Validate that context is properly formatted and included in the API call to OpenRouter

## Phase 5: User Story 3 - API Endpoint (Priority: P2)

**Goal**: Create a FastAPI endpoint `/ask` to communicate with the AI agent, so that it can be integrated into the user interface.

**Independent Test Criteria**: Can be tested by making HTTP requests to the `/ask` endpoint and verifying proper request/response handling.

**Acceptance Scenarios**:
1. Given a POST request to `/ask` with a query parameter, When the endpoint processes the request, Then it returns a JSON response with the AI-generated answer.

- [X] T301 [US3] Create backend/api.py file
- [X] T302 [P] [US3] Initialize FastAPI app with proper configuration
- [X] T303 [P] [US3] Add CORS middleware for frontend integration
- [X] T304 [US3] Define Pydantic models for QueryRequest and QueryResponse
- [X] T305 [US3] Create POST /ask endpoint that calls agent.generate_answer(query)
- [X] T306 [US3] Return JSON response with answer and sources

## Phase 6: Integration & Testing

- [X] T401 Test the full flow: User Query -> FastAPI -> Agent -> Retrieval -> OpenRouter -> Answer
- [X] T402 Verify that responses are within the 10-second performance goal
- [X] T403 Test "I don't know" cases when context is insufficient
- [X] T404 Test with various query types to ensure proper response formatting
- [X] T405 Verify the response includes both answer and sources as required

## Phase 7: Polish & Cross-Cutting Concerns

- [X] T501 Add error handling to agent and API for API failures or empty results
- [X] T502 Add logging to track API requests and AI responses
- [ ] T503 Update README with instructions for running the AI agent API
- [X] T504 Add comments to explain the AI agent process and parameters used
- [X] T505 Perform final testing to ensure all requirements from spec are met