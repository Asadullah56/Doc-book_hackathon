# Research: Embedding Pipeline Setup

## Decision: Project Structure
**Rationale**: Following user requirement for a single main.py file in backend directory to handle all embedding pipeline functionality
**Alternatives considered**: Multi-file structure with separate modules for each function, but single file approach was explicitly requested

## Decision: Technology Stack
**Rationale**:
- Python for backend processing
- Cohere API for embedding generation
- Qdrant for vector storage
- BeautifulSoup4 for HTML parsing
- Requests for HTTP operations
- python-dotenv for environment management
- tiktoken for token counting

**Alternatives considered**:
- OpenAI embeddings vs Cohere (Cohere was specified in requirements)
- Pinecone vs Qdrant (Qdrant was specified in requirements)
- Scrapy vs Requests/BeautifulSoup (Requests/BS4 simpler for this use case)

## Decision: Target URL
**Rationale**: Using https://doc-book-hackathon.vercel.app/ as specified in user requirements
**Alternatives considered**: None - this was explicitly provided as the target

## Decision: Chunk Size
**Rationale**: 500-800 token range as specified in requirements, using tiktoken to count tokens properly
**Alternatives considered**: Character-based chunking vs token-based chunking (token-based is more appropriate for embeddings)

## Decision: Qdrant Collection Name
**Rationale**: Using "rag_embedding" as specified in user requirements
**Alternatives considered**: None - this was explicitly specified

## Key Functions to Implement
1. `get_all_urls` - Crawl and extract all URLs from the target Docusaurus site
2. `extract_text_from_url` - Extract clean text content from each URL
3. `chunk_text` - Split text into 500-800 token chunks
4. `embed` - Generate embeddings using Cohere API
5. `create_collection` - Create Qdrant collection named "rag_embedding"
6. `save_chunk_to_qdrant` - Store embeddings with metadata in Qdrant
7. `main` - Execute the complete pipeline

## Required Environment Variables
- COHERE_API_KEY
- QDRANT_URL
- QDRANT_API_KEY

## Potential Challenges
1. Docusaurus site structure may require specific selectors to extract main content
2. Rate limiting from Cohere API
3. Proper handling of nested navigation and avoiding duplicate content
4. Memory management for large documents
5. Error handling for inaccessible URLs