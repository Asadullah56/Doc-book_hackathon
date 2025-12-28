#!/bin/bash
# Script to start the backend server
echo "Starting the AI backend server..."

# Navigate to the backend directory
cd "$(dirname "$0")"

# Install dependencies if not already installed
pip install -r requirements.txt

# Start the FastAPI server
echo "Starting server on http://localhost:8000"
uvicorn api:app --host 0.0.0.0 --port 8000