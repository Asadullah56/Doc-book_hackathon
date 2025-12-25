# Data Model: Embedding Pipeline Setup

## Entities

### DocumentChunk
- **Fields**:
  - id: string (unique identifier for the chunk)
  - content: string (the text content of the chunk)
  - url: string (source URL of the content)
  - chapter_title: string (title of the chapter/section)
  - token_count: integer (number of tokens in the chunk)
  - created_at: datetime (timestamp when chunk was created)

- **Validation rules**:
  - content must not be empty
  - url must be a valid URL format
  - token_count must be between 500-800 tokens as per requirements
  - chapter_title must not be empty

### EmbeddingVector
- **Fields**:
  - id: string (unique identifier matching the document chunk)
  - vector: array<float> (numerical representation from Cohere embedding)
  - text: string (original text content)
  - metadata: object (containing url and chapter_title)
  - created_at: datetime (timestamp when embedding was generated)

- **Validation rules**:
  - vector must have consistent dimensions based on Cohere model
  - metadata must contain both url and chapter_title as per requirements
  - text must match the original content

### QdrantRecord
- **Fields**:
  - id: string (unique identifier)
  - payload: object (containing url, chapter_title, and original text)
  - vector: array<float> (embedding vector)
  - collection: string (name of the collection, "rag_embedding")

- **Validation rules**:
  - payload must contain url and chapter_title as specified in requirements
  - vector dimensions must match the Cohere embedding model output
  - collection name must be "rag_embedding" as specified

## Relationships
- Each DocumentChunk generates one EmbeddingVector
- Each EmbeddingVector is stored as one QdrantRecord
- Multiple DocumentChunks can originate from the same URL but different sections/chapters