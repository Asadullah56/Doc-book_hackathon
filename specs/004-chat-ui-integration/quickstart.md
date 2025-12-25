# Quickstart: Docusaurus Chat UI Integration

## Prerequisites
- Node.js 18+ and npm/yarn/pnpm
- Docusaurus project already set up in `doc-book/` directory
- Backend API running at http://localhost:8000/ask

## Setup
1. Navigate to the doc-book directory:
   ```bash
   cd doc-book
   ```

2. Install required dependencies:
   ```bash
   npm install react-markdown lucide-react
   # or
   yarn add react-markdown lucide-react
   ```

3. Create the components directory if it doesn't exist:
   ```bash
   mkdir -p src/components
   ```

## Integration
1. The ChatBubble component will be created in `doc-book/src/components/ChatBubble.tsx`

2. Integrate the component globally in your Docusaurus layout (details in implementation)

## Development
1. Start the Docusaurus development server:
   ```bash
   cd doc-book
   npm run start
   # or
   yarn start
   ```

2. The chat bubble should appear on all documentation pages

## Testing
1. Open your browser to the documentation site
2. Verify the floating chat bubble appears on all pages
3. Test sending questions and receiving AI responses
4. Verify responsive behavior on different screen sizes
5. Test loading and error states