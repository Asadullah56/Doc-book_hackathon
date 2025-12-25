import React from 'react';
import ChatBubble from '../components/ChatBubble';

const Root = ({ children }: { children: React.ReactNode }) => {
  return (
    <>
      {children}
      <ChatBubble />
    </>
  );
};

export default Root;