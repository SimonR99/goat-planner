import React, { useState } from 'react';
import ChatBox from './components/ChatBox';
import BehaviorTree from './components/BehaviorTree';

function App() {
  const [isTreeVisible, setIsTreeVisible] = useState(true);

  const toggleTree = () => {
    setIsTreeVisible(!isTreeVisible);
  };

  return (
    <div className="flex flex-col h-screen bg-gray-100">
      <div className="bg-red-700 text-white p-4 flex justify-between items-center">
        <h1 className="text-2xl font-bold">PROTEUS AI Interface</h1>
        <button
          onClick={toggleTree}
          className="bg-white text-red-700 hover:bg-red-100 font-bold py-2 px-4 rounded transition duration-300 ease-in-out"
        >
          {isTreeVisible ? 'Hide Tree' : 'Show Tree'}
        </button>
      </div>
      <div className="flex flex-grow p-4 space-x-4 overflow-hidden">
        <div className={`${isTreeVisible ? 'w-1/2' : 'w-full'} bg-white rounded-lg shadow-lg transition-all duration-300 overflow-hidden`}>
          <ChatBox />
        </div>
        {isTreeVisible && (
          <div className="w-1/2 bg-white rounded-lg shadow-lg transition-all duration-300 overflow-hidden">
            <BehaviorTree />
          </div>
        )}
      </div>
    </div>
  );
}

export default App;
