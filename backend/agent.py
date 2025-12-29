# Load environment variables FIRST before any other imports
from dotenv import load_dotenv
load_dotenv()

import os
from typing import List, Dict, Optional
from openai import OpenAI
from retrieve import retrieve_context


# Print statement to verify keys on startup
print(f"OpenRouter API key loaded: {bool(os.getenv('OPENROUTER_API_KEY'))}")
print(f"Cohere API key loaded: {bool(os.getenv('COHERE_API_KEY'))}")
print(f"Qdrant URL loaded: {bool(os.getenv('QDRANT_URL'))}")
print(f"Qdrant API key loaded: {bool(os.getenv('QDRANT_API_KEY'))}")
print(f"Hugging Face Space URL loaded: {bool(os.getenv('HUGGINGFACE_SPACE_URL'))}")


class AIAgent:
    def __init__(self):
        # Store configuration but don't initialize client yet
        self.api_key = os.getenv("OPENROUTER_API_KEY")
        self.base_url = "https://openrouter.ai/api/v1"
        self.model = os.getenv("OPENROUTER_MODEL", "mistralai/devstral-2512:free")
        self.client = None

    def _ensure_client(self):
        """Initialize the OpenAI client lazily when needed."""
        if self.client is None:
            if not self.api_key:
                raise ValueError("OPENROUTER_API_KEY environment variable is not set")
            self.client = OpenAI(
                api_key=self.api_key,
                base_url=self.base_url,
                default_headers={
                    "HTTP-Referer": "https://asadullah56-deploy-project-1.hf.space",
                    "X-Title": "Humanoid Academy"
                }
            )

    def generate_answer(self, query: str) -> Dict:
        """
        Generate an answer based on the user query and retrieved context.

        Args:
            query (str): The user's query

        Returns:
            Dict: Contains the answer and sources
        """
        print(f"DEBUG: Received query: {query}")  # Debug logging

        # Check if required environment variables are set
        required_vars = ["OPENROUTER_API_KEY", "COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
        for var in required_vars:
            if not os.getenv(var):
                print(f"ERROR: Required environment variable {var} not set")
                return {
                    "answer": f"Error: Required environment variable {var} not set",
                    "sources": []
                }

        # Retrieve relevant context
        retrieved_chunks = retrieve_context(query)
        print(f"DEBUG: Retrieved {len(retrieved_chunks)} chunks from context")  # Debug logging

        # Build the system prompt
        system_prompt = "You are the Humanoid Academy AI. Answer only based on the provided robotics documentation. If context is missing, say you don't know."

        # Format the context for the AI
        context_text = ""
        sources = []

        if retrieved_chunks:
            for chunk in retrieved_chunks:
                print(f"DEBUG: Processing chunk from {chunk['url']}")  # Debug logging
                context_text += f"Source: {chunk['url']}\n"
                context_text += f"Title: {chunk['chapter_title']}\n"
                context_text += f"Content: {chunk['text']}\n\n"
                sources.append({
                    "url": chunk['url'],
                    "chapter_title": chunk['chapter_title'],
                    "text": chunk['text'][:200] + "..." if len(chunk['text']) > 200 else chunk['text']
                })
        else:
            print("DEBUG: No context retrieved for the query")  # Debug logging
            # If no context is retrieved, set a default message
            context_text = "No relevant context found in the documentation."
            sources = []

        # Build the user message with context
        user_message = f"Context:\n{context_text}\n\nQuestion: {query}"
        print(f"DEBUG: Sending user message to OpenRouter API")  # Debug logging

        try:
            # Ensure client is initialized
            self._ensure_client()

            # Call the OpenRouter API
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_message}
                ],
                temperature=0.1,  # Lower temperature for more consistent answers
            )

            answer = response.choices[0].message.content
            print(f"DEBUG: Successfully received response from OpenRouter API")  # Debug logging

            return {
                "answer": answer,
                "sources": sources
            }
        except Exception as e:
            print(f"ERROR: Exception in OpenRouter API call: {str(e)}")  # Error logging
            # Handle API errors gracefully
            return {
                "answer": f"Error generating answer: {str(e)}",
                "sources": sources
            }


# Create a global instance for easy access
agent = AIAgent()


def generate_answer(query: str) -> Dict:
    """
    Generate an answer based on the user query and retrieved context.

    Args:
        query (str): The user's query

    Returns:
        Dict: Contains the answer and sources
    """
    return agent.generate_answer(query)
