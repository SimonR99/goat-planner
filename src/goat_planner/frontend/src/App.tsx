import React, { useState } from 'react';
import ChatBox from './components/ChatBox';
import BehaviorTree from './components/BehaviorTree';
import WorldState from './components/WorldState';

function App() {
  const [isTreeVisible, setIsTreeVisible] = useState(true);
  const [isWorldStateVisible, setIsWorldStateVisible] = useState(true);

  const toggleTree = () => {
    setIsTreeVisible(!isTreeVisible);
  };

  const toggleWorldState = () => {
    setIsWorldStateVisible(!isWorldStateVisible);
  };

  return (
    <div className="flex flex-col h-screen bg-gray-100">
      <div className="bg-red-700 text-white p-4 flex justify-between items-center">
        <h1 className="text-2xl font-bold">PROTEUS AI Interface</h1>
        <div className="space-x-4">
          <button
            onClick={toggleWorldState}
            className="bg-white text-red-700 hover:bg-red-100 font-bold py-2 px-4 rounded transition duration-300 ease-in-out"
          >
            {isWorldStateVisible ? 'Hide Objects' : 'Show Objects'}
          </button>
          <button
            onClick={toggleTree}
            className="bg-white text-red-700 hover:bg-red-100 font-bold py-2 px-4 rounded transition duration-300 ease-in-out"
          >
            {isTreeVisible ? 'Hide Tree' : 'Show Tree'}
          </button>
        </div>
      </div>
      <div className="flex flex-grow p-4 space-x-4 overflow-hidden">
        <div className={`${isTreeVisible ? 'w-1/2' : 'w-full'} bg-white rounded-lg shadow-lg transition-all duration-300 overflow-hidden`}>
          <ChatBox />
        </div>
        {isTreeVisible && (
          <div className="w-1/2 flex flex-col space-y-4">
            <div className="flex-1 bg-white rounded-lg shadow-lg transition-all duration-300 overflow-hidden">
              <BehaviorTree />
            </div>
            {isWorldStateVisible && (
              <div className="h-1/3 bg-white rounded-lg shadow-lg transition-all duration-300 overflow-auto">
                <WorldState />
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
}

export default App;

