import os
import sys
from typing import List, Dict, Optional
from openai import OpenAI
from retrieve import retrieve_context
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Use stderr for logging to work with uvicorn
def log_debug(msg):
    print(f"[DEBUG] {msg}", file=sys.stderr)

# Print statement to verify keys on startup
log_debug(f"OpenRouter API key loaded: {bool(os.getenv('OPENROUTER_API_KEY'))}")
log_debug(f"Cohere API key loaded: {bool(os.getenv('COHERE_API_KEY'))}")
log_debug(f"Qdrant URL loaded: {bool(os.getenv('QDRANT_URL'))}")
log_debug(f"Qdrant API key loaded: {bool(os.getenv('QDRANT_API_KEY'))}")


class AIAgent:
    def __init__(self):
        # Initialize OpenAI client with OpenRouter configuration
        self.client = OpenAI(
            api_key=os.getenv("OPENROUTER_API_KEY"),
            base_url="https://openrouter.ai/api/v1",
            default_headers={
                "HTTP-Referer": os.getenv("HUGGINGFACE_SPACE_URL", "https://huggingface.co/spaces/your-space-name"),
                "X-Title": "Doc-book Assistant"
            }
        )
        self.model = os.getenv("OPENROUTER_MODEL", "mistralai/devstral-2512:free")

    def generate_answer(self, query: str) -> Dict:
        """
        Generate an answer based on user query and retrieved context.

        Args:
            query (str): The user's query

        Returns:
            Dict: Contains answer and sources
        """
        log_debug(f"Received query: {query}")  # Debug logging

        # Check if required environment variables are set
        required_vars = ["OPENROUTER_API_KEY", "COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
        for var in required_vars:
            if not os.getenv(var):
                log_debug(f"ERROR: Required environment variable {var} not set")
                return {
                    "answer": f"Error: Required environment variable {var} not set",
                    "sources": []
                }

        # Retrieve relevant context
        retrieved_chunks = retrieve_context(query)
        log_debug(f"Retrieved {len(retrieved_chunks)} chunks from context")  # Debug logging

        # Build system prompt
        system_prompt = "You are Humanoid Academy AI. Answer only based on the provided robotics documentation. If context is missing, say you don't know."

        # Format context for AI
        context_text = ""
        sources = []

        if retrieved_chunks:
            for chunk in retrieved_chunks:
                log_debug(f"Processing chunk from {chunk['url']}")  # Debug logging
                context_text += f"Source: {chunk['url']}\n"
                context_text += f"Title: {chunk['chapter_title']}\n"
                context_text += f"Content: {chunk['text']}\n\n"
                sources.append({
                    "url": chunk['url'],
                    "chapter_title": chunk['chapter_title'],
                    "text": chunk['text'][:200] + "..." if len(chunk['text']) > 200 else chunk['text']
                })
        else:
            log_debug("No context retrieved for query")  # Debug logging
            # If no context is retrieved, set a default message
            context_text = "No relevant context found in the documentation."
            sources = []

        # Build user message with context
        user_message = f"Context:\n{context_text}\n\nQuestion: {query}"
        log_debug("Sending user message to OpenRouter API")  # Debug logging

        try:
            # Call to OpenRouter API
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.1,  # Lower temperature for more consistent answers
            )

            answer = response.choices[0].message.content
            log_debug("Successfully received response from OpenRouter API")  # Debug logging

            return {
                "answer": answer,
                "sources": sources
            }
        except Exception as e:
            log_debug(f"Exception in OpenRouter API call: {str(e)}")  # Error logging
            # Handle API errors gracefully
            return {
                "answer": f"Error generating answer: {str(e)}",
                "sources": sources
            }


# Create a global instance for easy access
agent = AIAgent()


def generate_answer(query: str) -> Dict:
    """
    Generate an answer based on user query and retrieved context.

    Args:
        query (str): The user's query

    Returns:
        Dict: Contains answer and sources
    """
    return agent.generate_answer(query)
