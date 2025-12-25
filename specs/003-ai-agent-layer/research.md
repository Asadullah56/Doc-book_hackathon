# Research: OpenRouter Agent and FastAPI Implementation

## Decision: Use OpenAI SDK with OpenRouter base URL
**Rationale**: The OpenAI Python SDK can be configured to work with OpenRouter by setting the base_url parameter. This provides a familiar interface while leveraging OpenRouter's model selection capabilities.

## Decision: FastAPI with Pydantic models for API
**Rationale**: FastAPI provides automatic API documentation (Swagger UI), built-in request validation with Pydantic, and excellent performance. Pydantic models ensure type safety and validation for API requests and responses.

## Decision: CORS middleware for frontend integration
**Rationale**: CORS must be enabled to allow frontend applications to make requests to the backend API from different origins.

## Decision: System Prompt structure for RAG
**Rationale**: The system prompt will follow the pattern: "You are an expert AI Assistant for Humanoid Academy. Use the following context to answer the user's question. If the context doesn't contain enough information, respond with 'I don't know.'"

## Decision: Integration with existing retrieve.py
**Rationale**: The agent will import and use the existing `retrieve_context` function from `retrieve.py` to maintain consistency with the existing retrieval system.

## Decision: Response structure with sources
**Rationale**: The API response will include both the answer and sources to provide transparency about where the information came from, following: `{ "answer": "...", "sources": [...] }`

## Alternatives considered:
- Using different AI SDKs: OpenAI SDK was chosen for its compatibility with OpenRouter and familiarity
- Different web frameworks: FastAPI chosen over Flask for automatic documentation and Pydantic integration
- Different response formats: JSON response with answer and sources chosen for frontend compatibility