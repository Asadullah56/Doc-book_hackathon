import React, { useState, useRef, useEffect } from 'react';
import ReactMarkdown from 'react-markdown';
import { Bot, Sparkles, Send, X, User } from 'lucide-react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Add pulsing animation CSS when component loads
if (typeof document !== 'undefined') {
  const styleId = 'pulsing-dots-animation';
  if (!document.getElementById(styleId)) {
    const style = document.createElement('style');
    style.id = styleId;
    style.textContent = `
      @keyframes pulse {
        0%, 100% {
          transform: scale(0.8);
          opacity: 0.5;
        }
        50% {
          transform: scale(1.2);
          opacity: 1;
        }
      }
    `;
    document.head.appendChild(style);
  }
}

interface Message {
  id: string;
  content: string;
  sender: 'user' | 'ai';
  timestamp: Date;
}

const ChatBubble: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();
  const backendUrl = siteConfig.customFields.backendUrl as string || 'http://127.0.0.1:8000';

  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`https://asadullah56-deploy-project-1.hf.space/ask`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: inputValue }),
      });

      if (!response.ok) {
        if (response.status === 401) {
          throw new Error('Authentication failed: Please check your API key configuration');
        } else if (response.status === 404) {
          throw new Error('API endpoint not found: Please verify the backend server is running');
        } else if (response.status === 500) {
          throw new Error('Internal server error: Please check the backend logs');
        } else {
          throw new Error(`API failed with status ${response.status}: ${response.statusText}`);
        }
      }

      const data = await response.json();
      const aiMessage: Message = {
        id: Date.now().toString(),
        content: data.answer,
        sender: 'ai',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, aiMessage]);
    } catch (err) {
      console.error('Chat error:', err);
      let errorMessage = 'Backend connection error.';
      let errorContent = 'Sorry, I encountered an error. Please try again.';

      if (err instanceof TypeError && err.message.includes('fetch')) {
        // Network error (server not running, network issue, etc.)
        errorMessage = `Network error: Cannot connect to backend server at ${backendUrl}. Please ensure the backend server is running.`;
        errorContent = `Unable to connect to the AI backend at ${backendUrl}. Please make sure the Python backend server is started.`;
      } else if (err instanceof Error) {
        if (err.message.includes('401')) {
          errorMessage = 'Authentication error: API key may be invalid or missing';
        } else if (err.message.includes('404')) {
          errorMessage = `Connection error: Backend server not found at ${backendUrl}`;
        } else if (err.message.includes('500')) {
          errorMessage = 'Server error: Please check backend logs';
        } else {
          errorMessage = err.message;
        }
        errorContent = err.message;
      }

      setError(errorMessage);
      setMessages(prev => [...prev, {
        id: Date.now().toString(),
        content: errorContent,
        sender: 'ai',
        timestamp: new Date(),
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChat = () => setIsOpen(!isOpen);

  // Pulsing dots component for loading animation
  const PulsingDots = () => (
    <div style={{ display: 'flex', alignItems: 'center', gap: '4px' }}>
      <div style={{
        width: '8px',
        height: '8px',
        backgroundColor: '#6b7280',
        borderRadius: '50%',
        animation: 'pulse 1.4s infinite ease-in-out',
        marginRight: '2px'
      }}></div>
      <div style={{
        width: '8px',
        height: '8px',
        backgroundColor: '#6b7280',
        borderRadius: '50%',
        animation: 'pulse 1.4s infinite ease-in-out 0.2s',
        marginRight: '2px'
      }}></div>
      <div style={{
        width: '8px',
        height: '8px',
        backgroundColor: '#6b7280',
        borderRadius: '50%',
        animation: 'pulse 1.4s infinite ease-in-out 0.4s'
      }}></div>
    </div>
  );

  return (
    /* 1. Global Wrapper: Isolated from Docusaurus layout flow */
    <div style={{ position: 'fixed', bottom: 0, right: 0, height: 0, width: 0, overflow: 'visible', zIndex: 9999 }}>
      
      {isOpen && (
        /* 2. Chat Window: Floating independently */
        <div style={{
          position: 'absolute', bottom: '85px', right: '25px',
          width: '400px', maxWidth: '90vw', height: '500px', minHeight: '60vh', backgroundColor: 'white',
          borderRadius: '20px', boxShadow: '0 10px 40px rgba(0,0,0,0.4)',
          display: 'flex', flexDirection: 'column', overflow: 'hidden',
          border: '1px solid #e5e7eb', fontFamily: 'sans-serif'
        }}>

          {/* Header */}
          <div style={{
            padding: '15px',
            background: 'linear-gradient(135deg, #2563eb 0%, #1e40af 100%)',
            color: 'white',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center',
            borderTopLeftRadius: '20px',
            borderTopRightRadius: '20px'
          }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
              <Bot size={18} />
              <span style={{ fontWeight: 'bold' }}>AI Assistant</span>
              <div style={{
                width: '8px',
                height: '8px',
                backgroundColor: '#10b981',
                borderRadius: '50%',
                boxShadow: '0 0 6px #10b981'
              }}></div> {/* Online status dot */}
            </div>
            <button onClick={toggleChat} style={{ background: 'none', border: 'none', color: 'white', cursor: 'pointer' }}>
              <X size={20} />
            </button>
          </div>

          {/* Messages: flex-1 ensures it grows and pushes input bar to bottom */}
          <div style={{ flex: 1, overflowY: 'auto', padding: '20px', backgroundColor: '#f9fafb', display: 'flex', flexDirection: 'column', gap: '12px' }}>
            {messages.length === 0 ? (
              <div style={{ height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', color: '#9ca3af', textAlign: 'center' }}>
                <p>Ask me anything about the documentation!</p>
              </div>
            ) : (
              messages.map((message) => (
                <div key={message.id} style={{ display: 'flex', justifyContent: message.sender === 'user' ? 'flex-end' : 'flex-start' }}>
                  <div style={{
                    maxWidth: '85%',
                    padding: '10px 14px',
                    borderRadius: '16px',
                    borderBottomRightRadius: message.sender === 'user' ? '4px' : '16px',
                    borderBottomLeftRadius: message.sender === 'user' ? '16px' : '4px',
                    fontSize: '14px',
                    lineHeight: '1.5',
                    backgroundColor: message.sender === 'user' ? '#2563eb' : 'white',
                    color: message.sender === 'user' ? 'white' : '#1f2937',
                    boxShadow: message.sender === 'ai'
                      ? '0 4px 12px rgba(0,0,0,0.08)'
                      : '0 2px 4px rgba(0,0,0,0.05)',
                    border: message.sender === 'user' ? 'none' : '1px solid #e5e7eb'
                  }}>
                    <ReactMarkdown>{message.content}</ReactMarkdown>
                    <div style={{ fontSize: '10px', marginTop: '4px', opacity: 0.7, textAlign: 'right' }}>
                      {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                    </div>
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div style={{ display: 'flex', alignItems: 'center', gap: '8px', color: '#6b7280', fontSize: '13px' }}>
                <Sparkles size={14} /> AI is thinking... <PulsingDots />
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area: Sticky at the very bottom */}
          <div style={{ padding: '16px', backgroundColor: 'white', borderTop: '1px solid #eee' }}>
            <form onSubmit={handleSubmit} style={{ display: 'flex', gap: '12px' }}>
              <input
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                placeholder="Ask a question..."
                style={{
                  flex: 1,
                  border: '2px solid #e5e7eb',
                  borderRadius: '30px',
                  padding: '12px 20px',
                  outline: 'none',
                  fontSize: '14px',
                  transition: 'border-color 0.3s, box-shadow 0.3s'
                }}
                onFocus={(e) => {
                  e.target.style.borderColor = '#2563eb';
                  e.target.style.boxShadow = '0 0 0 3px rgba(37, 99, 235, 0.2)';
                }}
                onBlur={(e) => {
                  e.target.style.borderColor = '#e5e7eb';
                  e.target.style.boxShadow = 'none';
                }}
                disabled={isLoading}
              />
              <button
                type="submit"
                disabled={isLoading || !inputValue.trim()}
                style={{
                  backgroundColor: isLoading || !inputValue.trim() ? '#d1d5db' : '#2563eb',
                  color: 'white',
                  border: 'none',
                  borderRadius: '50%',
                  width: '44px',
                  height: '44px',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  cursor: isLoading ? 'not-allowed' : 'pointer',
                  transition: 'transform 0.2s, background-color 0.2s',
                  alignSelf: 'center'
                }}
                onMouseEnter={(e) => {
                  if (!(isLoading || !inputValue.trim())) {
                    e.currentTarget.style.transform = 'scale(1.05)';
                  }
                }}
                onMouseLeave={(e) => {
                  e.currentTarget.style.transform = 'scale(1)';
                }}
              >
                <Send size={18} />
              </button>
            </form>
            {error && <div style={{ color: '#ef4444', fontSize: '12px', marginTop: '8px', paddingLeft: '12px' }}>{error}</div>}
          </div>
        </div>
      )}

      {/* 3. Floating Action Button: Circular modern FAB */}
      <button
        onClick={toggleChat}
        style={{
          position: 'absolute', bottom: '20px', right: '20px',
          width: '60px', height: '60px', borderRadius: '50%',
          backgroundColor: '#2563eb', color: 'white', border: 'none',
          boxShadow: '0 4px 15px rgba(37, 99, 235, 0.4)',
          cursor: 'pointer', display: 'flex', alignItems: 'center', justifyContent: 'center',
          transition: 'transform 0.2s'
        }}
        onMouseOver={(e) => e.currentTarget.style.transform = 'scale(1.1)'}
        onMouseOut={(e) => e.currentTarget.style.transform = 'scale(1)'}
      >
        <Bot size={28} />
      </button>
    </div>
  );
};

export default ChatBubble;