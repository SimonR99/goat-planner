import React, { useState } from 'react';

const HomePage: React.FC = () => {
  const [messages, setMessages] = useState<string[]>([]);
  const [inputMessage, setInputMessage] = useState('');
  const [robotState, setRobotState] = useState('Idle');

  const handleSendMessage = () => {
    if (inputMessage.trim()) {
      setMessages([...messages, inputMessage]);
      setInputMessage('');
      // TODO: Send message to backend
    }
  };

  return (
    <div className="flex h-screen">
      {/* Left Sidebar - Chat Interface */}
      <div className="w-1/4 flex flex-col border-r border-gray-300">
        <div className="flex-1 overflow-y-auto p-4 space-y-4">
          {messages.map((msg, index) => (
            <div key={index} className="bg-gray-100 p-3 rounded-lg">{msg}</div>
          ))}
        </div>
        <div className="p-4 flex">
          <input
            type="text"
            value={inputMessage}
            onChange={(e) => setInputMessage(e.target.value)}
            placeholder="Type a message..."
            className="flex-1 border border-gray-300 rounded-l-lg px-4 py-2 focus:outline-none focus:ring-2 focus:ring-blue-500"
          />
          <button 
            onClick={handleSendMessage}
            className="bg-blue-500 text-white px-4 py-2 rounded-r-lg hover:bg-blue-600 focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            Send
          </button>
        </div>
      </div>

      {/* Right Content Area */}
      <div className="w-3/4 flex flex-col">
        {/* Top Section - Robot State */}
        <div className="h-1/3 p-4 border-b border-gray-300">
          <h2 className="text-2xl font-bold mb-4">Robot State</h2>
          <p className="text-lg">Current State: {robotState}</p>
          {/* TODO: Add more detailed robot state information */}
        </div>

        {/* Bottom Section - Visualization */}
        <div className="h-2/3 p-4">
          <h2 className="text-2xl font-bold mb-4">Visualization</h2>
          {/* TODO: Add visualization component here */}
          <div className="bg-gray-200 h-full flex items-center justify-center">
            <p>Visualization Placeholder</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default HomePage;