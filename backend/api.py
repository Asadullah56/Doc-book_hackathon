from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List
import uvicorn
import os
import sys
from dotenv import load_dotenv
from agent import generate_answer

load_dotenv()  # Load API keys from .env file

# Use stderr for logging to work with uvicorn
def log_debug(msg):
    print(f"[DEBUG] {msg}", file=sys.stderr)


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

# Add CORS middleware - Allow frontend origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://localhost:8000",
        "http://127.0.0.1:8000",
        "https://doc-book-hackathon.vercel.app",
        "https://*.hf.space",
    ],
    allow_origin_regex=r"https://.*\.hf\.space",  # Match any *.hf.space domain
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods
    allow_headers=["*"],  # Allow all headers
)


@app.get("/")
async def root():
    """
    Root endpoint to verify the API is running.
    """
    return {"status": "running"}


@app.post("/ask", response_model=QueryResponse)
async def ask_endpoint(request: QueryRequest) -> QueryResponse:
    """
    Endpoint to ask questions to the AI agent.

    Args:
        request: QueryRequest containing user's query

    Returns:
        QueryResponse containing answer and sources
    """
    log_debug(f"Received request with query: {request.query[:50]}...")  # Log first 50 chars of query
    try:
        result = generate_answer(request.query)

        # Check if result indicates an error
        if result.get("answer", "").startswith("Error:"):
            # Log error
            log_debug(f"ERROR in ask_endpoint: {result['answer']}")
            # Return a more informative response
            return QueryResponse(
                answer=result["answer"],
                sources=[]
            )

        # Convert sources to proper format
        sources = []
        for source in result.get("sources", []):
            sources.append(Source(
                url=source.get("url", ""),
                chapter_title=source.get("chapter_title", ""),
                text=source.get("text", "")
            ))

        log_debug(f"Successfully processed request, returning answer with {len(sources)} sources")
        return QueryResponse(
            answer=result["answer"],
            sources=sources
        )
    except Exception as e:
        log_debug(f"ERROR in ask_endpoint: {str(e)}")
        import traceback
        traceback.print_exc()  # Print full traceback for debugging
        return QueryResponse(
            answer=f"Error processing request: {str(e)}",
            sources=[]
        )


@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify the API is running.
    """
    return {"status": "healthy", "message": "AI Agent API is running"}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=7860)
