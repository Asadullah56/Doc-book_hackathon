# Data Model: Docusaurus Chat UI Integration

## Entities

### Chat Message
- **Description**: A message in the chat conversation
- **Fields**:
  - id: string (unique identifier for the message)
  - content: string (the text content of the message)
  - sender: 'user' | 'ai' (indicating who sent the message)
  - timestamp: Date (when the message was sent/received)
- **Validation**: content must be non-empty string, sender must be either 'user' or 'ai'

### Chat State
- **Description**: The current state of the chat interface
- **Fields**:
  - messages: Array<ChatMessage> (list of all messages in the conversation)
  - isLoading: boolean (indicates if the AI is processing a request)
  - isVisible: boolean (indicates if the chat bubble is open/closed)
  - error: string | null (error message if any error occurred)
- **Validation**: messages array must not exceed reasonable size limits

### API Request
- **Description**: Request payload sent to the backend API
- **Fields**:
  - query: string (the user's question)
- **Validation**: query must be non-empty string

### API Response
- **Description**: Response received from the backend API
- **Fields**:
  - answer: string (the AI-generated answer)
  - sources: Array<object> (list of source objects used to generate the answer)
- **Validation**: answer must be string (can be empty if error occurred)

## Relationships

- One Chat State → Many Chat Messages (one-to-many relationship)
- One API Request → One API Response (one-to-one mapping)

## State Transitions

1. User opens chat bubble → Chat State visibility changes to true
2. User submits message → New Chat Message (user) added to Chat State
3. System sends API Request → Loading state activated
4. System receives API Response → New Chat Message (AI) added to Chat State, loading state deactivated
5. Error occurs → Error state activated with error message