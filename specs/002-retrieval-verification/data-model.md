# Data Model: Retrieval Verification

## Entities

### Search Query
- **Description**: User input text that will be converted to a vector for semantic search
- **Fields**:
  - query_text: string (the original search query)
  - query_vector: List[float] (the embedded representation of the query)
- **Validation**: Must be non-empty string

### Retrieved Chunk
- **Description**: Text content retrieved from the Qdrant collection that matches the query
- **Fields**:
  - text_content: string (the actual text content)
  - relevance_score: float (similarity score between query and chunk)
  - source_url: string (URL where the original content is located)
  - chapter_title: string (title of the chapter where content originated)
- **Validation**: relevance_score must be between 0 and 1

### Qdrant Point
- **Description**: Vector database entry containing text chunk and metadata
- **Fields**:
  - point_id: string (unique identifier for the vector)
  - vector: List[float] (1024-dimensional embedding vector)
  - payload: Dict (metadata including text, url, chapter_title)
- **Relationships**: Each point corresponds to one Retrieved Chunk

## Relationships

- One Search Query → Many Retrieved Chunks (via similarity search)
- One Retrieved Chunk ← One Qdrant Point (one-to-one mapping)

## State Transitions

1. User provides Search Query
2. Search Query is converted to vector representation
3. Qdrant performs similarity search against collection
4. Retrieved Chunks are filtered by similarity threshold
5. Top 3 Retrieved Chunks are returned to user