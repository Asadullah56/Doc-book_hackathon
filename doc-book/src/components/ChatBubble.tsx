import React, { useState, useRef, useEffect } from 'react';
import ReactMarkdown from 'react-markdown';
import { MessageCircle, Send, X, Loader2 } from 'lucide-react';

interface Message {
  id: string;
  content: string;
  sender: 'user' | 'ai';
  timestamp: Date;
}

const ChatBubble: React.FC = () => {
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
      const response = await fetch('http://localhost:8000/ask', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: inputValue }),
      });

      if (!response.ok) throw new Error(`API failed: ${response.status}`);

      const data = await response.json();
      const aiMessage: Message = {
        id: Date.now().toString(),
        content: data.answer,
        sender: 'ai',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, aiMessage]);
    } catch (err) {
      console.error(err);
      setError('Backend connection error.');
      setMessages(prev => [...prev, {
        id: Date.now().toString(),
        content: 'Sorry, I encountered an error. Please try again.',
        sender: 'ai',
        timestamp: new Date(),
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChat = () => setIsOpen(!isOpen);

  return (
    /* 1. Global Wrapper: Isolated from Docusaurus layout flow */
    <div style={{ position: 'fixed', bottom: 0, right: 0, height: 0, width: 0, overflow: 'visible', zIndex: 9999 }}>
      
      {isOpen && (
        /* 2. Chat Window: Floating independently */
        <div style={{
          position: 'absolute', bottom: '85px', right: '25px',
          width: '360px', height: '520px', backgroundColor: 'white',
          borderRadius: '16px', boxShadow: '0 12px 40px rgba(0,0,0,0.2)',
          display: 'flex', flexDirection: 'column', overflow: 'hidden',
          border: '1px solid #e5e7eb', fontFamily: 'sans-serif'
        }}>
          
          {/* Header */}
          <div style={{ padding: '15px', backgroundColor: '#2563eb', color: 'white', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
              <MessageCircle size={18} />
              <span style={{ fontWeight: 'bold' }}>AI Assistant</span>
            </div>
            <button onClick={toggleChat} style={{ background: 'none', border: 'none', color: 'white', cursor: 'pointer' }}>
              <X size={20} />
            </button>
          </div>

          {/* Messages: flex-1 ensures it grows and pushes input bar to bottom */}
          <div style={{ flex: 1, overflowY: 'auto', padding: '15px', backgroundColor: '#f9fafb', display: 'flex', flexDirection: 'column', gap: '12px' }}>
            {messages.length === 0 ? (
              <div style={{ height: '100%', display: 'flex', alignItems: 'center', justifyContent: 'center', color: '#9ca3af', textAlign: 'center' }}>
                <p>Ask me anything about the documentation!</p>
              </div>
            ) : (
              messages.map((message) => (
                <div key={message.id} style={{ display: 'flex', justifyContent: message.sender === 'user' ? 'flex-end' : 'flex-start' }}>
                  <div style={{
                    maxWidth: '85%', padding: '10px 14px', borderRadius: '12px',
                    fontSize: '14px', lineHeight: '1.5',
                    backgroundColor: message.sender === 'user' ? '#2563eb' : 'white',
                    color: message.sender === 'user' ? 'white' : '#1f2937',
                    boxShadow: '0 2px 4px rgba(0,0,0,0.05)',
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
                <Loader2 className="animate-spin" size={14} /> AI is thinking...
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area: Sticky at the very bottom */}
          <div style={{ padding: '12px', borderTop: '1px solid #eee', backgroundColor: 'white' }}>
            <form onSubmit={handleSubmit} style={{ display: 'flex', gap: '8px' }}>
              <input
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                placeholder="Ask a question..."
                style={{ flex: 1, border: '1px solid #d1d5db', borderRadius: '20px', padding: '8px 15px', outline: 'none', fontSize: '14px' }}
                disabled={isLoading}
              />
              <button
                type="submit"
                disabled={isLoading || !inputValue.trim()}
                style={{
                  backgroundColor: isLoading || !inputValue.trim() ? '#d1d5db' : '#2563eb',
                  color: 'white', border: 'none', borderRadius: '50%', width: '38px', height: '38px',
                  display: 'flex', alignItems: 'center', justifyContent: 'center',
                  cursor: isLoading ? 'not-allowed' : 'pointer'
                }}
              >
                <Send size={18} />
              </button>
            </form>
            {error && <div style={{ color: '#ef4444', fontSize: '10px', marginTop: '4px' }}>{error}</div>}
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
        <MessageCircle size={28} />
      </button>
    </div>
  );
};

export default ChatBubble;