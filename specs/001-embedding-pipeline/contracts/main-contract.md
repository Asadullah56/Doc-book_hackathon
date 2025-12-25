# API Contracts: Embedding Pipeline

## Embedding Pipeline Functions Contract

### Function: get_all_urls(base_url: str) -> List[str]
- **Purpose**: Crawl the Docusaurus site and return all accessible URLs
- **Input**: Base URL of the Docusaurus site (https://doc-book-hackathon.vercel.app/)
- **Output**: List of all discovered URLs within the site
- **Errors**: Returns empty list if base URL is inaccessible

### Function: extract_text_from_url(url: str) -> Dict[str, str]
- **Purpose**: Extract clean text content and chapter title from a URL
- **Input**: Single URL from the site
- **Output**: Dictionary with 'content' and 'chapter_title' keys
- **Errors**: Returns empty content if URL is inaccessible

### Function: chunk_text(text: str, max_tokens: int = 800, min_tokens: int = 500) -> List[Dict[str, str]]
- **Purpose**: Split text into chunks of 500-800 tokens
- **Input**: Text content and token limits
- **Output**: List of text chunks with metadata
- **Errors**: May return chunks slightly outside token range to preserve semantic boundaries

### Function: embed(text_chunks: List[str]) -> List[List[float]]
- **Purpose**: Generate embeddings for text chunks using Cohere
- **Input**: List of text chunks
- **Output**: List of embedding vectors (numerical representations)
- **Errors**: Returns empty list if API key is invalid or rate limit exceeded

### Function: create_collection(collection_name: str = "rag_embedding")
- **Purpose**: Create a Qdrant collection for storing embeddings
- **Input**: Collection name (default: "rag_embedding")
- **Output**: Success confirmation
- **Errors**: Throws error if collection already exists or invalid credentials

### Function: save_chunk_to_qdrant(chunk_data: Dict, embedding: List[float])
- **Purpose**: Store a text chunk with its embedding in Qdrant
- **Input**: Chunk data with metadata (URL, chapter title) and embedding vector
- **Output**: Success confirmation with record ID
- **Errors**: Throws error if Qdrant is inaccessible or invalid credentials

### Main Execution Flow Contract
1. Validate environment variables (API keys)
2. Create Qdrant collection named "rag_embedding"
3. Get all URLs from https://doc-book-hackathon.vercel.app/
4. For each URL: extract text → chunk → embed → save to Qdrant
5. Output completion summary