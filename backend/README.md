# Humanoid Academy - RAG Backend

## Project Description

An AI-powered backend to crawl Docusaurus documentation and create a searchable vector knowledge base. This system extracts text from deployed Docusaurus URLs, generates embeddings using Cohere, and stores them in Qdrant for RAG-based retrieval.

## Tech Stack

- **Python**: Core programming language
- **uv**: Modern Python package manager and project initializer
- **Cohere**: AI-powered text embeddings generation
- **Qdrant Cloud**: Vector database for similarity search
- **BeautifulSoup4**: Web scraping and HTML parsing
- **TikToken**: Token counting for text chunking
- **HTTPX**: HTTP client for API requests

## Setup Instructions

1. **Install uv** (if not already installed):
   ```bash
   pip install uv
   ```

2. **Sync dependencies**:
   ```bash
   cd backend
   uv sync
   ```

3. **Set up environment variables**:
   - Copy the `.env` template or create a new `.env` file:
   ```bash
   cp .env.example .env  # if available
   ```
   - Add your API keys to the `.env` file:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   ```

## Execution

To run the ingestion pipeline:

```bash
uv run python main.py
```

This will:
- Crawl all URLs from the target Docusaurus site (https://doc-book-hackathon.vercel.app/)
- Extract text content from each page
- Chunk content into 500-800 token segments
- Generate embeddings using Cohere
- Store embeddings in Qdrant collection named "rag_embedding"
- Include URL and chapter title as metadata

## Current Progress

**Active Focus: Spec 1 - Embedding Pipeline**

The system is currently focused on implementing the embedding pipeline that extracts text from deployed Docusaurus URLs, generates embeddings using Cohere, and stores them in Qdrant for RAG-based retrieval. The pipeline successfully processes content and stores it in a searchable vector database.

## Running the API Server

To run the FastAPI server that serves the chatbot API:

1. Make sure you have the required dependencies installed:
   ```bash
   pip install -r requirements.txt
   ```

2. Ensure your `.env` file contains the required API keys:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   OPENROUTER_API_KEY=your_openrouter_api_key_here
   OPENROUTER_MODEL=mistralai/devstral-2512:free
   ```

3. Start the API server:
   ```bash
   uvicorn api:app --host 0.0.0.0 --port 8000
   ```

4. The API will be available at `http://localhost:8000`
   - Health check: `http://localhost:8000/health`
   - Chat endpoint: `http://localhost:8000/ask`

## Starting the Server

To start the backend server, run:

```bash
cd backend
uvicorn api:app --host 0.0.0.0 --port 8000
```

Or use the provided script:
- On Windows: `start-server.bat`
- On Linux/Mac: `chmod +x start-server.sh && ./start-server.sh`

## Troubleshooting

1. If you get "failed to fetch" errors, ensure the backend server is running
2. Check that the `.env` file contains all required API keys
3. Make sure port 8000 is not being used by another application
4. Check the server logs for any error messages