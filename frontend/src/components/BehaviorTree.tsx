import React, { useState, useEffect } from 'react';
import { Tree } from 'react-d3-tree';
import io from 'socket.io-client';
import { FaList, FaRandom, FaRedo, FaPlay } from 'react-icons/fa';

const socket = io('http://localhost:5000');

const BehaviorTree: React.FC = () => {
  const [treeData, setTreeData] = useState<any>(null);
  const [viewMode, setViewMode] = useState<'json' | 'graph'>('graph');
  const [jsonText, setJsonText] = useState('');
  const [error, setError] = useState<string | null>(null);

  const fetchBehaviorTree = async () => {
    try {
      setError(null);
      console.log('Fetching behavior tree...');
      const response = await fetch('http://localhost:5000/api/behavior_tree');
      console.log('Response status:', response.status);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const data = await response.json();
      console.log('Fetched behavior tree data:', data);
      setTreeData(data);
      setJsonText(JSON.stringify(data, null, 2));
    } catch (error) {
      console.error('Error fetching behavior tree:', error);
      setError('Failed to fetch behavior tree. Please try again later.');
    }
  };

  useEffect(() => {
    fetchBehaviorTree();

    socket.on('plan_update', (updatedTree) => {
      console.log('Received plan update:', JSON.stringify(updatedTree, null, 2));
      if (validateTree(updatedTree)) {
        setTreeData(updatedTree);
        setJsonText(JSON.stringify(updatedTree, null, 2));
      } else {
        console.error('Invalid tree structure received');
        setError('Received invalid tree structure');
      }
    });

    const intervalId = setInterval(() => {
      socket.emit('request_plan_update');
    }, 1000); // Check for updates every second

    return () => {
      socket.off('plan_update');
      clearInterval(intervalId);
    };
  }, []);

  const validateTree = (tree: any): boolean => {
    if (!tree || typeof tree !== 'object') return false;
    if (!('name' in tree)) return false;
    if ('children' in tree) {
      if (!Array.isArray(tree['children'])) return false;
      return tree['children'].every(validateTree);
    }
    return true;
  };

  const handleJsonEdit = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const newJsonText = e.target.value;
    setJsonText(newJsonText);
    try {
      const parsedJson = JSON.parse(newJsonText);
      if (validateTree(parsedJson)) {
        setTreeData(parsedJson);
      } else {
        console.error('Invalid tree structure in JSON');
      }
    } catch (error) {
      console.error('Invalid JSON:', error);
    }
  };

  const renderCustomNodeElement = ({ nodeDatum, toggleNode }: any) => {
    let Icon;
    switch (nodeDatum.type) {
      case 'sequence':
        Icon = FaList;
        break;
      case 'fallback':
        Icon = FaRandom;
        break;
      case 'retry':
        Icon = FaRedo;
        break;
      default:
        Icon = FaPlay;
    }

    return (
      <g>
        <circle r="20" fill="#fff1f2" stroke="#dc2626" strokeWidth="2" onClick={toggleNode} />
        <foreignObject width="30" height="30" x="-15" y="-15">
          <div className="flex items-center justify-center w-full h-full">
            <Icon color="#dc2626" size="20" />
          </div>
        </foreignObject>
        <text fill="#dc2626" strokeWidth="1" x="25" y="5" textAnchor="start" alignmentBaseline="middle" fontSize="14">
          {nodeDatum.name}
        </text>
      </g>
    );
  };

  if (error) {
    return <div className="text-red-600">{error}</div>;
  }

  if (!treeData) {
    return <div>Loading behavior tree...</div>;
  }

  return (
    <div className="h-full flex flex-col overflow-hidden">
      <div className="flex justify-between items-center p-4 border-b border-red-200">
        <h2 className="text-xl font-bold text-red-700">Behavior Tree</h2>
        <label className="flex items-center cursor-pointer">
          <div className="relative">
            <input type="checkbox" className="sr-only" checked={viewMode === 'graph'} onChange={() => setViewMode(viewMode === 'json' ? 'graph' : 'json')} />
            <div className="block bg-gray-300 w-14 h-8 rounded-full"></div>
            <div className={`dot absolute left-1 top-1 bg-red-600 w-6 h-6 rounded-full transition ${viewMode === 'graph' ? 'transform translate-x-6' : ''}`}></div>
          </div>
          <div className="ml-3 text-gray-700 font-medium">
            {viewMode === 'json' ? 'JSON' : 'Graph'} View
          </div>
        </label>
      </div>
      <div className="flex-grow overflow-y-auto overflow-x-hidden p-4">
        {viewMode === 'json' ? (
          <textarea
            value={jsonText}
            onChange={handleJsonEdit}
            className="w-full h-full p-2 bg-white border border-red-200 rounded text-gray-800 font-mono resize-none overflow-y-auto overflow-x-hidden whitespace-pre-wrap"
            style={{ minHeight: '300px' }}
          />
        ) : (
          <div style={{ width: '100%', height: '100%', minHeight: '300px', overflow: 'hidden' }}>
            <Tree
              data={treeData}
              orientation="vertical"
              pathFunc="step"
              translate={{ x: 300, y: 50 }}
              separation={{ siblings: 2, nonSiblings: 2 }}
              renderCustomNodeElement={renderCustomNodeElement}
              nodeSize={{ x: 250, y: 100 }}
            />
          </div>
        )}
      </div>
    </div>
  );
};

export default BehaviorTree;