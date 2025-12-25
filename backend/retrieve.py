import os
import cohere
from qdrant_client import QdrantClient
from typing import List, Dict

# Load environment variables
from dotenv import load_dotenv
load_dotenv()


def retrieve_context(query: str, threshold: float = 0.3, limit: int = 3):
    """
    Retrieve the most relevant text chunks from Qdrant based on a query.

    Args:
        query (str): The user query to search for
        threshold (float): Minimum similarity score for results (default 0.3)
        limit (int): Maximum number of results to return (default 3)

    Returns:
        List[Dict]: List of dictionaries containing text, score, and URL for each result
    """
    # Validate environment variables
    required_vars = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    for var in required_vars:
        if not os.getenv(var):
            raise ValueError(f"Required environment variable {var} not set")

    # Initialize Cohere client
    cohere_api_key = os.getenv("COHERE_API_KEY")
    co = cohere.Client(cohere_api_key)

    # Generate embedding for the query using search_query input type
    try:
        response = co.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"  # Using search_query for query embeddings
        )
        query_embedding = response.embeddings[0]
    except Exception as e:
        print(f"Error generating query embedding: {e}")
        return []

    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        timeout=10
    )

    # Perform search in Qdrant - using the modern query_points API
    try:
        # Using the modern query_points API as requested
        response = client.query_points(
            collection_name="rag_embedding",
            query=query_embedding,
            limit=limit * 2,  # Get more results than needed for threshold filtering
            with_payload=True
        )

        # Extract results from the response
        search_results = response.points

        # Filter results based on threshold and format output
        filtered_results = []
        for result in search_results:
            if result.score >= threshold:
                filtered_results.append({
                    'text': result.payload.get('text', '')[:200] + "..." if len(result.payload.get('text', '')) > 200 else result.payload.get('text', ''),
                    'score': result.score,
                    'url': result.payload.get('url', ''),
                    'chapter_title': result.payload.get('chapter_title', '')
                })

        # Limit to top 3 results after filtering
        filtered_results = filtered_results[:limit]

        return filtered_results

    except AttributeError as e:
        # If query_points method doesn't exist, fall back to search method
        print(f"Modern query_points API not available: {e}")
        print("Falling back to search API...")

        try:
            search_results = client.search(
                collection_name="rag_embedding",
                query_vector=query_embedding,
                limit=limit * 2,
                with_payload=True
            )

            # Filter results based on threshold and format output
            filtered_results = []
            for result in search_results:
                if result.score >= threshold:
                    filtered_results.append({
                        'text': result.payload.get('text', '')[:200] + "..." if len(result.payload.get('text', '')) > 200 else result.payload.get('text', ''),
                        'score': result.score,
                        'url': result.payload.get('url', ''),
                        'chapter_title': result.payload.get('chapter_title', '')
                    })

            # Limit to top 3 results after filtering
            filtered_results = filtered_results[:limit]

            return filtered_results
        except Exception as e2:
            print(f"Error searching in Qdrant: {e2}")
            return []
    except Exception as e:
        print(f"Error searching in Qdrant: {e}")
        return []


def display_retrieval_results(results: List[Dict]):
    """
    Display retrieval results in a formatted way showing relevance scores and source URLs.

    Args:
        results (List[Dict]): List of retrieval results
    """
    if not results:
        print("No results found for the query.")
        return

    print(f"\nFound {len(results)} relevant results:\n")
    for i, result in enumerate(results, 1):
        print(f"Result {i}:")
        print(f"  Score: {result['score']:.4f}")
        print(f"  Source: {result['url']}")
        print(f"  Chapter: {result['chapter_title']}")
        print(f"  Text: {result['text']}")
        print()


def main():
    """
    Main function to test the retrieval functionality.
    """
    print("Testing retrieval functionality...")

    # Validate environment variables
    required_vars = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    for var in required_vars:
        if not os.getenv(var):
            raise ValueError(f"Required environment variable {var} not set")

    print("Environment variables validated")

    test_query = "Explain the ROS 2 communication backbone in Module 1"
    print(f"Testing retrieval with query: '{test_query}'")

    results = retrieve_context(test_query)
    display_retrieval_results(results)

    if results:
        print("✅ Retrieval test successful!")
        print(f"✅ Found {len(results)} relevant results for the test query.")
    else:
        print("⚠️  No results found for the test query. This may be expected if the content hasn't been indexed yet.")


if __name__ == "__main__":
    main()