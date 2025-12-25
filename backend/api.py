from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict, Optional
from agent import generate_answer


# Define Pydantic models
class QueryRequest(BaseModel):
    query: str


class Source(BaseModel):
    url: str
    chapter_title: str
    text: str


class QueryResponse(BaseModel):
    answer: str
    sources: List[Source]


# Create FastAPI app
app = FastAPI(title="Humanoid Academy AI API", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods
    allow_headers=["*"],  # Allow all headers
)


@app.post("/ask", response_model=QueryResponse)
async def ask_endpoint(request: QueryRequest) -> QueryResponse:
    """
    Endpoint to ask questions to the AI agent.

    Args:
        request: QueryRequest containing the user's query

    Returns:
        QueryResponse containing the answer and sources
    """
    result = generate_answer(request.query)

    # Convert sources to the proper format
    sources = []
    for source in result.get("sources", []):
        sources.append(Source(
            url=source.get("url", ""),
            chapter_title=source.get("chapter_title", ""),
            text=source.get("text", "")
        ))

    return QueryResponse(
        answer=result["answer"],
        sources=sources
    )


@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify the API is running.
    """
    return {"status": "healthy", "message": "AI Agent API is running"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)