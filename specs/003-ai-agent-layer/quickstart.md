# Quickstart: AI Agent Layer with OpenRouter

## Prerequisites
- Python 3.8+
- Valid OPENROUTER_API_KEY environment variable
- Existing Qdrant collection with embedded content
- Existing retrieve.py with context retrieval functionality

## Setup
1. Install dependencies:
   ```bash
   uv add openai fastapi uvicorn python-dotenv pydantic
   ```

2. Add environment variables to `.env`:
   ```bash
   OPENROUTER_API_KEY="your-openrouter-api-key"
   OPENROUTER_MODEL="anthropic/claude-3.5-sonnet"  # or your preferred model
   ```

## Run the API Server
1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Start the FastAPI server:
   ```bash
   uv run uvicorn api:app --reload
   ```

3. Access the API at `http://localhost:8000`
4. API documentation available at `http://localhost:8000/docs`

## Test the Endpoint
1. Use the Swagger UI at `http://localhost:8000/docs` to test the `/ask` endpoint
2. Or make a POST request:
   ```bash
   curl -X POST "http://localhost:8000/ask" \
     -H "Content-Type: application/json" \
     -d '{"query": "Your question here"}'
   ```

## Expected Response
```json
{
  "answer": "Natural language answer based on retrieved context",
  "sources": [
    {
      "url": "source_url",
      "chapter_title": "chapter_title",
      "text": "text snippet"
    }
  ]
}
```