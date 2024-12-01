import React, { useState, useEffect, useRef } from 'react';
import io from 'socket.io-client';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { tomorrow } from 'react-syntax-highlighter/dist/esm/styles/prism';
import { FaPen, FaTrash } from 'react-icons/fa';

const socket = io('http://localhost:5000');

interface Message {
  id?: string;
  text: string;
  isUser: boolean;
}

interface ConversationHistory {
  id: string;
  name: string;
  messages: Message[];
}

const ChatBox: React.FC = () => {
  const [conversations, setConversations] = useState<ConversationHistory[]>([]);
  const [currentConversationId, setCurrentConversationId] = useState<string | null>(null);
  const [inputMessage, setInputMessage] = useState('');
  const [editingConversationId, setEditingConversationId] = useState<string | null>(null);
  const [newConversationName, setNewConversationName] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const [showPlanTags, setShowPlanTags] = useState(false);
  const [currentAIResponse, setCurrentAIResponse] = useState<string>('');
  const [isReceivingPlan, setIsReceivingPlan] = useState(false);
  const [isTTSEnabled, setIsTTSEnabled] = useState(false);

  useEffect(() => {
    socket.on('connect', () => {
      console.log('Connected to server');
      socket.emit('get_conversations');
    });

    socket.on('conversations', (data: ConversationHistory[]) => {
      setConversations(data);
      if (data.length > 0 && !currentConversationId) {
        setCurrentConversationId(data[0].id);
      }
    });

    socket.on('new_message', (data: { conversationId: string, message: Message }) => {
      if (data.conversationId === currentConversationId) {
        if (data.message.isUser) {
          setConversations(prevConversations => 
            prevConversations.map(conv => 
              conv.id === data.conversationId 
                ? { ...conv, messages: [...conv.messages, data.message] }
                : conv
            )
          );
          setCurrentAIResponse('');
          setIsReceivingPlan(false);
        } else {
          let newContent = data.message.text;
          if (newContent.includes('<plan>')) {
            setIsReceivingPlan(true);
            newContent = newContent.split('<plan>')[0];
          }
          if (newContent.includes('</plan>')) {
            setIsReceivingPlan(false);
            newContent = newContent.split('</plan>')[1] || '';
          }
          if (!isReceivingPlan && newContent) {
            setCurrentAIResponse(prev => prev + newContent);
          }
        }
      }
    });

    socket.on('ai_response_complete', ({ conversationId }) => {
      if (conversationId === currentConversationId) {
        setConversations(prevConversations => 
          prevConversations.map(conv => 
            conv.id === conversationId 
              ? { ...conv, messages: [...conv.messages, { text: currentAIResponse, isUser: false }] }
              : conv
          )
        );
        setCurrentAIResponse('');
      }
    });

    return () => {
      socket.off('connect');
      socket.off('conversations');
      socket.off('new_message');
      socket.off('ai_response_complete');
    };
  }, [currentConversationId, currentAIResponse, isReceivingPlan, isTTSEnabled]);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [conversations]);

  const handleSendMessage = (e: React.FormEvent) => {
    e.preventDefault();
    if (inputMessage.trim() !== '') {
      if (!currentConversationId) {
        socket.emit('new_conversation', { message: inputMessage });
      } else {
        socket.emit('chat_message', { conversationId: currentConversationId, message: inputMessage });
      }
      setInputMessage('');
    }
  };

  const handleNewConversation = () => {
    socket.emit('new_conversation');
  };

  const handleRenameConversation = (id: string) => {
    if (newConversationName.trim() !== '') {
      socket.emit('rename_conversation', { id, name: newConversationName });
      setEditingConversationId(null);
      setNewConversationName('');
    }
  };

  const handleDeleteConversation = (id: string) => {
    if (window.confirm('Are you sure you want to delete this conversation?')) {
      socket.emit('delete_conversation', { id });
      if (currentConversationId === id) {
        setCurrentConversationId(null);
      }
    }
  };

  const currentConversation = conversations.find(conv => conv.id === currentConversationId);

  const renderMessage = (message: Message) => {
    if (message.isUser) {
      return message.text;
    }

    const codeBlockRegex = /```(\w+)?\s*([\s\S]*?)```/g;
    const planRegex = /<plan>[\s\S]*?<\/plan>/g;
    const parts = [];
    let lastIndex = 0;
    let match;

    let processedText = showPlanTags ? message.text : message.text.replace(planRegex, '');

    while ((match = codeBlockRegex.exec(processedText)) !== null) {
      if (match.index > lastIndex) {
        parts.push(processedText.slice(lastIndex, match.index));
      }
      const language = match[1] || 'text';
      const code = match[2].trim();
      parts.push(
        <SyntaxHighlighter 
          key={match.index}
          language={language} 
          style={tomorrow}
          customStyle={{
            padding: '1em',
            borderRadius: '0.5em',
            fontSize: '0.9em',
            lineHeight: '1.5',
          }}
          wrapLines={true}
          wrapLongLines={true}
        >
          {code}
        </SyntaxHighlighter>
      );
      lastIndex = match.index + match[0].length;
    }

    if (lastIndex < processedText.length) {
      parts.push(processedText.slice(lastIndex));
    }

    return parts.length > 0 ? parts : processedText;
  };

  const handleTTSToggle = (enabled: boolean) => {
    setIsTTSEnabled(enabled);
    socket.emit('toggle_tts', { enabled });
  };

  return (
    <div className="flex flex-col h-full">
      <div className="p-4 border-b border-red-200 flex justify-between items-center">
        <h2 className="text-xl font-bold text-red-700">Chat Interface</h2>
        <div className="flex items-center">
          <label className="flex items-center mr-4">
            <input
              type="checkbox"
              checked={showPlanTags}
              onChange={(e) => setShowPlanTags(e.target.checked)}
              className="mr-2"
            />
            Show Plan Tags
          </label>
          <label className="flex items-center mr-4">
            <input
              type="checkbox"
              checked={isTTSEnabled}
              onChange={(e) => handleTTSToggle(e.target.checked)}
              className="mr-2"
            />
            Enable Text-to-Speech
          </label>
          <button onClick={handleNewConversation} className="bg-red-600 text-white px-4 py-2 rounded">
            New Conversation
          </button>
        </div>
      </div>
      <div className="flex-grow flex overflow-hidden">
        <div className="w-1/4 border-r border-red-200 overflow-y-auto">
          {conversations.map(conv => (
            <div 
              key={conv.id} 
              className={`p-2 cursor-pointer flex items-center justify-between ${conv.id === currentConversationId ? 'bg-red-100' : ''}`}
            >
              {editingConversationId === conv.id ? (
                <input
                  type="text"
                  value={newConversationName}
                  onChange={(e) => setNewConversationName(e.target.value)}
                  onBlur={() => handleRenameConversation(conv.id)}
                  onKeyPress={(e) => e.key === 'Enter' && handleRenameConversation(conv.id)}
                  className="w-full p-1 border rounded"
                  autoFocus
                />
              ) : (
                <>
                  <span
                    className="cursor-pointer flex-grow"
                    onClick={() => setCurrentConversationId(conv.id)}
                  >
                    {conv.name}
                  </span>
                  <div className="flex items-center">
                    <FaPen
                      className="text-gray-500 hover:text-red-600 cursor-pointer ml-2"
                      onClick={(e) => {
                        e.stopPropagation();
                        setEditingConversationId(conv.id);
                        setNewConversationName(conv.name);
                      }}
                    />
                    <FaTrash
                      className="text-gray-500 hover:text-red-600 cursor-pointer ml-2"
                      onClick={(e) => {
                        e.stopPropagation();
                        handleDeleteConversation(conv.id);
                      }}
                    />
                  </div>
                </>
              )}
            </div>
          ))}
        </div>
        <div className="w-3/4 flex flex-col">
          <div className="flex-grow overflow-y-auto p-4 space-y-4">
            {currentConversation?.messages.map((message, index) => (
              <div key={message.id || index} className={`flex ${message.isUser ? 'justify-end' : 'justify-start'}`}>
                <div className={`max-w-3/4 p-3 rounded-lg ${
                  message.isUser ? 'bg-red-600 text-white' : 'bg-gray-200 text-gray-800'
                } break-words whitespace-pre-wrap overflow-x-auto`}>
                  {renderMessage(message)}
                </div>
              </div>
            ))}
            {currentAIResponse && (
              <div className="flex justify-start">
                <div className="max-w-3/4 p-3 rounded-lg bg-gray-200 text-gray-800 break-words whitespace-pre-wrap overflow-x-auto">
                  {renderMessage({ text: currentAIResponse, isUser: false })}
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={handleSendMessage} className="p-4 border-t border-red-200">
            <input
              type="text"
              value={inputMessage}
              onChange={(e) => setInputMessage(e.target.value)}
              placeholder="Command the robot..."
              className="w-full p-2 bg-white border border-red-300 rounded-lg text-gray-800 placeholder-gray-500 focus:outline-none focus:border-red-500"
            />
          </form>
        </div>
      </div>
    </div>
  );
};

export default ChatBox;