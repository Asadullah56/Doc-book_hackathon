@echo off
REM Script to start the backend server
echo Starting the AI backend server...

REM Navigate to the backend directory
cd /d "C:\Users\asada\OneDrive\Desktop\Work\backend"

REM Install dependencies if not already installed
pip install -r requirements.txt

REM Start the FastAPI server
echo Starting server on http://localhost:8000
uvicorn api:app --host 0.0.0.0 --port 8000