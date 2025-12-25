# Quickstart: Embedding Pipeline Setup

## Prerequisites
- Python 3.8+
- API keys for Cohere and Qdrant Cloud

## Setup

1. **Create the backend directory:**
```bash
mkdir backend
cd backend
```

2. **Create a virtual environment:**
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

3. **Create requirements.txt:**
```txt
cohere==4.0
qdrant-client==1.9
beautifulsoup4==4.12
requests==2.31
python-dotenv==1.0
tiktoken==0.5
```

4. **Install dependencies:**
```bash
pip install -r requirements.txt
```

5. **Create .env file:**
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

6. **Create the main.py file with the embedding pipeline implementation**

## Usage

1. **Run the embedding pipeline:**
```bash
python main.py
```

2. **The pipeline will:**
   - Crawl all URLs from https://doc-book-hackathon.vercel.app/
   - Extract text content from each page
   - Chunk content into 500-800 token segments
   - Generate embeddings using Cohere
   - Store embeddings in Qdrant collection named "rag_embedding"
   - Include URL and chapter title as metadata

## Environment Variables
- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key