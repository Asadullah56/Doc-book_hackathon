# Quickstart: Retrieval Verification

## Prerequisites
- Python 3.8+
- Valid COHERE_API_KEY environment variable
- Valid QDRANT_URL and QDRANT_API_KEY environment variables
- Completed embedding pipeline (Spec 1) with content stored in 'rag_embedding' collection

## Setup
1. Ensure environment variables are set:
   ```bash
   export COHERE_API_KEY="your-cohere-api-key"
   export QDRANT_URL="your-qdrant-url"
   export QDRANT_API_KEY="your-qdrant-api-key"
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Run Retrieval Verification
1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Execute the main script which now includes retrieval verification:
   ```bash
   python main.py
   ```

3. The script will automatically run the retrieval verification test after the embedding pipeline completes.

## Expected Output
- The script will display the top 3 most relevant text chunks
- Each chunk will include:
  - Source URL where the content originated
  - Relevance score (similarity score)
  - The actual text content of the chunk
- Test query: "Explain ROS 2 communication in Module 1"

## Verification
- Check that the returned results are relevant to ROS 2 communication
- Confirm that relevance scores are above the threshold (default 0.3)
- Verify that source URLs point to appropriate documentation