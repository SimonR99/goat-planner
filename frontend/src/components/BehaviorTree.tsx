import React, { useState, useEffect } from 'react';
import { Tree } from 'react-d3-tree';
import io from 'socket.io-client';
import { FaList, FaRandom, FaRedo, FaPlay } from 'react-icons/fa';

const socket = io('http://localhost:5000');

interface TreeNode {
  [key: string]: any;
}

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
    
    // Check root structure
    if ('root' in tree) {
      const root = tree.root;
      if (!root.main_tree_to_execute || !root.BehaviorTree) {
        return false;
      }
      
      // Validate BehaviorTree structure
      const behaviorTree = root.BehaviorTree;
      if (!behaviorTree.ID || behaviorTree.ID !== root.main_tree_to_execute) {
        return false;
      }
      
      // Recursively validate the rest of the tree
      return Object.values(behaviorTree).every(node => {
        if (typeof node === 'object' && node !== null) {
          return validateNode(node);
        }
        return true;
      });
    }
    
    // If not root, validate as a node
    return validateNode(tree);
  };

  const validateNode = (node: any): boolean => {
    if (!node || typeof node !== 'object') return false;
    
    // Check if it's a valid node structure
    if ('children' in node) {
      if (!Array.isArray(node.children)) return false;
      return node.children.every(validateNode);
    }
    
    // Recursively validate nested objects
    return Object.values(node).every(value => {
      if (typeof value === 'object' && value !== null) {
        if (Array.isArray(value)) {
          return value.every(item => {
            if (typeof item === 'object') return validateNode(item);
            return true;
          });
        }
        return validateNode(value);
      }
      return true;
    });
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

  const transformTreeData = (node: TreeNode): any => {
    // If this is the root node with BehaviorTree structure
    if (node.root && node.root.BehaviorTree) {
      return transformTreeData(node.root.BehaviorTree);
    }

    // Create a node object with name and children
    const transformedNode: any = {
      name: node.name || Object.keys(node)[0] || "Unknown",
      type: getNodeType(node),
      children: []
    };

    // Add all properties as attributes
    Object.entries(node).forEach(([key, value]) => {
      if (key !== 'children' && typeof value !== 'object') {
        transformedNode[key] = value;
      }
    });

    // Process children
    Object.entries(node).forEach(([key, value]) => {
      if (typeof value === 'object' && value !== null) {
        if (Array.isArray(value)) {
          // Handle array of children
          value.forEach(child => {
            if (typeof child === 'object') {
              transformedNode.children.push(transformTreeData(child));
            }
          });
        } else if (key !== 'name' && key !== 'type') {
          // Handle nested objects as children
          transformedNode.children.push(transformTreeData(value));
        }
      }
    });

    return transformedNode;
  };

  const getNodeType = (node: TreeNode): string => {
    if ('RecoveryNode' in node) return 'recovery';
    if ('PipelineSequence' in node) return 'sequence';
    if ('ReactiveFallback' in node) return 'fallback';
    if ('RateController' in node) return 'rate';
    if ('ReactiveSequence' in node) return 'sequence';
    if ('RoundRobin' in node) return 'round-robin';
    return 'action';
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
      case 'recovery':
        Icon = FaRedo;
        break;
      case 'rate':
        Icon = FaPlay;
        break;
      case 'round-robin':
        Icon = FaRandom;
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
        <text 
          fill="#dc2626" 
          strokeWidth="1" 
          x="25" 
          y="5" 
          textAnchor="start" 
          alignmentBaseline="middle" 
          fontSize="14"
        >
          {nodeDatum.name}
        </text>
        {/* Display additional attributes */}
        {Object.entries(nodeDatum)
          .filter(([key]) => !['name', 'type', 'children'].includes(key))
          .map(([key, value], index) => (
            <text
              key={key}
              fill="#666"
              x="25"
              y={25 + (index * 18)}
              fontSize="12"
            >
              {`${key}: ${value}`}
            </text>
          ))}
      </g>
    );
  };

  // Update the tree data before rendering
  const processedTreeData = treeData ? transformTreeData(treeData) : null;

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
              data={processedTreeData}
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