import os
import re
import time
import requests
import hashlib
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from typing import List, Dict, Tuple
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
import tiktoken
from dotenv import load_dotenv


# Load environment variables
load_dotenv()


def get_all_urls(base_url: str = "https://doc-book-hackathon.vercel.app/") -> List[str]:
    """
    Fetch all URLs from the sitemap.xml of the target Docusaurus site
    """
    sitemap_url = base_url.rstrip('/') + '/sitemap.xml'

    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'xml')
        urls = []

        # Find all <loc> tags which contain URLs
        for loc in soup.find_all('loc'):
            url = loc.text.strip()
            # Replace the incorrect domain in sitemap with the correct one
            if "your-github-username.github.io" in url:
                corrected_url = url.replace("your-github-username.github.io", "doc-book-hackathon.vercel.app")
                urls.append(corrected_url)
            elif url.startswith(base_url):
                urls.append(url)

        print(f"Found {len(urls)} URLs from sitemap")
        return urls
    except Exception as e:
        print(f"Error fetching sitemap: {e}")
        # If sitemap fails, try to get URLs by crawling the main page
        print("Attempting to crawl main page for URLs...")
        try:
            response = requests.get(base_url)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')
            found_urls = set()

            # Find all links on the main page
            for link in soup.find_all('a', href=True):
                href = link['href']
                full_url = urljoin(base_url, href)
                if full_url.startswith(base_url):
                    found_urls.add(full_url)

            urls = list(found_urls)
            print(f"Found {len(urls)} URLs by crawling main page")
            return urls
        except Exception as e2:
            print(f"Error crawling main page: {e2}")
            return []


def extract_text_from_url(url: str) -> Dict[str, str]:
    """
    Extract clean text content and chapter title from a given URL
    """
    try:
        response = requests.get(url)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
            script.decompose()

        # Try to find main content - Docusaurus typically uses these selectors
        main_content = (
            soup.find('main') or
            soup.find('article') or
            soup.find('div', class_=re.compile(r'.*main.*|.*content.*|.*doc.*')) or
            soup.find('div', {'role': 'main'}) or
            soup
        )

        # Get the page title as chapter title
        title_tag = soup.find('title')
        chapter_title = title_tag.get_text().strip() if title_tag else urlparse(url).path.split('/')[-1] or 'Unknown Chapter'

        # Extract text content
        text_content = main_content.get_text(separator=' ', strip=True)

        # Clean up excessive whitespace
        text_content = re.sub(r'\s+', ' ', text_content).strip()

        return {
            'content': text_content,
            'chapter_title': chapter_title
        }
    except Exception as e:
        print(f"Error extracting text from {url}: {e}")
        return {
            'content': '',
            'chapter_title': ''
        }


def chunk_text(text: str, min_tokens: int = 500, max_tokens: int = 800) -> List[Dict[str, str]]:
    """
    Split text into chunks of 500-800 tokens using tiktoken
    """
    if not text:
        return []

    # Initialize tiktoken encoder (using cl100k_base which is used by many models)
    encoder = tiktoken.get_encoding("cl100k_base")

    # Split text into sentences to preserve semantic boundaries
    sentences = re.split(r'(?<=[.!?])\s+', text)

    chunks = []
    current_chunk = ""
    current_token_count = 0

    for sentence in sentences:
        # Estimate token count for this sentence
        sentence_tokens = len(encoder.encode(sentence))

        # If adding this sentence would exceed max_tokens, save current chunk
        if current_token_count + sentence_tokens > max_tokens and current_chunk:
            # Only add chunk if it meets minimum token requirement
            if current_token_count >= min_tokens:
                chunks.append({
                    'text': current_chunk.strip(),
                    'token_count': current_token_count
                })
                current_chunk = sentence
                current_token_count = sentence_tokens
            else:
                # If current chunk doesn't meet minimum, add sentence to it anyway
                current_chunk += " " + sentence
                current_token_count += sentence_tokens
        else:
            # Add sentence to current chunk
            if current_chunk:
                current_chunk += " " + sentence
                current_token_count += sentence_tokens
            else:
                current_chunk = sentence
                current_token_count = sentence_tokens

    # Add the final chunk if it meets minimum requirements
    if current_chunk and current_token_count >= min_tokens:
        chunks.append({
            'text': current_chunk.strip(),
            'token_count': current_token_count
        })
    elif current_chunk and len(chunks) == 0:
        # If this is the only chunk and it's too small, add it anyway
        chunks.append({
            'text': current_chunk.strip(),
            'token_count': current_token_count
        })

    return chunks


def embed(text_chunks: List[str]) -> List[List[float]]:
    """
    Generate embeddings using Cohere's embed-english-v3.0 model
    """
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable not set")

    co = cohere.Client(cohere_api_key)

    # Cohere has a limit on the number of texts per request
    batch_size = 96  # Cohere's limit is 96 texts per request
    embeddings = []

    for i in range(0, len(text_chunks), batch_size):
        batch = text_chunks[i:i + batch_size]

        try:
            response = co.embed(
                texts=batch,
                model="embed-english-v3.0",
                input_type="search_document"  # Using search_document for content to be searched
            )

            embeddings.extend(response.embeddings)

            # Add a small delay to respect API rate limits
            time.sleep(0.1)

        except Exception as e:
            print(f"Error generating embeddings for batch {i//batch_size + 1}: {e}")
            # Return empty embeddings for failed batch
            embeddings.extend([[]] * len(batch))

    return embeddings


def create_collection(collection_name: str = "rag_embedding", vector_size: int = 1024):
    """
    Create a Qdrant collection named 'rag_embedding' with 1024 dimensions
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        timeout=10
    )

    # Check if collection already exists and delete it if it does
    try:
        existing_collections = client.get_collections()
        collection_exists = any(col.name == collection_name for col in existing_collections.collections)

        if collection_exists:
            print(f"Collection '{collection_name}' already exists, deleting it...")
            client.delete_collection(collection_name=collection_name)
            print(f"Deleted collection '{collection_name}'")

        # Create the collection fresh
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
        )
        print(f"Created collection '{collection_name}' with {vector_size} dimensions")
    except Exception as e:
        print(f"Error managing collection: {e}")
        raise e

    return client


def save_chunk_to_qdrant(client: QdrantClient, collection_name: str, text_chunk: str,
                        embedding: List[float], url: str, chapter_title: str, chunk_id: str = None):
    """
    Save a text chunk with its embedding to Qdrant with URL and chapter title metadata
    """
    # Generate a deterministic ID based on the content to prevent duplicates
    content_for_id = f"{url}_{text_chunk}"
    point_id = hashlib.md5(content_for_id.encode()).hexdigest()

    try:
        client.upsert(
            collection_name=collection_name,
            points=[
                models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "url": url,
                        "chapter_title": chapter_title,
                        "text": text_chunk,
                        "original_chunk_id": chunk_id,  # Store original ID in payload
                        "created_at": time.time()
                    }
                )
            ]
        )
        return True
    except Exception as e:
        print(f"Error saving chunk to Qdrant: {e}")
        return False


def main():
    """
    Main function to orchestrate the complete embedding pipeline
    """
    print("Starting Embedding Pipeline...")

    # Validate environment variables
    required_vars = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    for var in required_vars:
        if not os.getenv(var):
            raise ValueError(f"Required environment variable {var} not set")

    print("Environment variables validated")

    # Create Qdrant collection
    print("Creating Qdrant collection...")
    client = create_collection("rag_embedding", 1024)

    # Get all URLs from the target site
    print("Fetching URLs from sitemap...")
    urls = get_all_urls("https://doc-book-hackathon.vercel.app/")

    if not urls:
        print("No URLs found, exiting.")
        return

    print(f"Processing {len(urls)} URLs...")

    processed_count = 0
    saved_count = 0

    for i, url in enumerate(urls):
        print(f"Processing URL {i+1}/{len(urls)}: {url}")

        # Extract text from URL
        content_data = extract_text_from_url(url)

        if not content_data['content']:
            print(f"  No content extracted from {url}, skipping...")
            continue

        # Chunk the text
        text_chunks = chunk_text(content_data['content'])

        if not text_chunks:
            print(f"  No valid chunks created from {url}, skipping...")
            continue

        print(f"  Created {len(text_chunks)} chunks from {url}")

        # Extract embeddings for each chunk
        chunk_texts = [chunk['text'] for chunk in text_chunks]
        embeddings = embed(chunk_texts)

        # Save each chunk and its embedding to Qdrant
        for j, (chunk, embedding) in enumerate(zip(text_chunks, embeddings)):
            if embedding:  # Check if embedding was successfully generated
                chunk_id = f"{urlparse(url).path.replace('/', '_')}_{j}"
                success = save_chunk_to_qdrant(
                    client,
                    "rag_embedding",
                    chunk['text'],
                    embedding,
                    url,
                    content_data['chapter_title'],
                    chunk_id
                )

                if success:
                    saved_count += 1
                else:
                    print(f"  Failed to save chunk {j} from {url}")
            else:
                print(f"  Failed to generate embedding for chunk {j} from {url}")

        processed_count += 1

        # Add a small delay to be respectful to the APIs
        time.sleep(0.1)

    print(f"\nPipeline completed!")
    print(f"Processed {processed_count} URLs")
    print(f"Saved {saved_count} chunks to Qdrant collection 'rag_embedding'")


if __name__ == "__main__":
    main()