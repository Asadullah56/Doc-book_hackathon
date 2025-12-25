# Humanoid Academy - Docusaurus Chat UI

This repository contains the Humanoid Academy documentation site with an integrated chat UI for AI-powered assistance.

## Getting Started

### Prerequisites

- Node.js 18+
- npm or yarn

### Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd <repository-name>
```

2. Navigate to the doc-book directory:
```bash
cd doc-book
```

3. Install dependencies:
```bash
npm install
```

### Running the Development Server

1. Start the Docusaurus development server:
```bash
npm run start
```

2. Open your browser to `http://localhost:3000` to view the documentation site.

### Chat UI Integration

The chat UI is integrated globally across all documentation pages as a floating bubble component. To use:

1. Look for the chat bubble icon in the bottom-right corner of any documentation page
2. Click the bubble to open the chat interface
3. Type your question about the documentation content
4. The AI response will appear in the chat window with Markdown formatting support

### Backend API Connection

The chat component communicates with a backend API at `http://localhost:8000/ask`. Make sure your backend service is running on port 8000 to use the chat functionality.

## Project Structure

- `doc-book/` - Docusaurus documentation site
- `doc-book/src/components/ChatBubble.tsx` - Main chat UI component
- `doc-book/src/css/custom.css` - Custom styling
- `doc-book/docusaurus.config.ts` - Site configuration

## Features

- Responsive floating chat bubble UI
- Real-time AI responses with Markdown formatting
- Loading and error state handling
- Global integration across all documentation pages
- Mobile-friendly design