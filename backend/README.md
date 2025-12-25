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