import os
from typing import List, Dict, Optional
from openai import OpenAI
from retrieve import retrieve_context


class AIAgent:
    def __init__(self):
        # Initialize OpenAI client with OpenRouter configuration
        self.client = OpenAI(
            api_key=os.getenv("OPENROUTER_API_KEY"),
            base_url="https://openrouter.ai/api/v1"
        )
        self.model = os.getenv("OPENROUTER_MODEL", "mistralai/devstral-2512:free")

    def generate_answer(self, query: str) -> Dict:
        """
        Generate an answer based on the user query and retrieved context.

        Args:
            query (str): The user's query

        Returns:
            Dict: Contains the answer and sources
        """
        # Retrieve relevant context
        retrieved_chunks = retrieve_context(query)

        # Build the system prompt
        system_prompt = "You are the Humanoid Academy AI. Answer only based on the provided robotics documentation. If context is missing, say you don't know."

        # Format the context for the AI
        context_text = ""
        sources = []

        if retrieved_chunks:
            for chunk in retrieved_chunks:
                context_text += f"Source: {chunk['url']}\n"
                context_text += f"Title: {chunk['chapter_title']}\n"
                context_text += f"Content: {chunk['text']}\n\n"
                sources.append({
                    "url": chunk['url'],
                    "chapter_title": chunk['chapter_title'],
                    "text": chunk['text'][:200] + "..." if len(chunk['text']) > 200 else chunk['text']
                })
        else:
            # If no context is retrieved, set a default message
            context_text = "No relevant context found in the documentation."
            sources = []

        # Build the user message with context
        user_message = f"Context:\n{context_text}\n\nQuestion: {query}"

        try:
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

            return {
                "answer": answer,
                "sources": sources
            }
        except Exception as e:
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