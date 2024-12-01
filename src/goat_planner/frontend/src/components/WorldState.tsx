import React, { useEffect, useState } from 'react';
import { socket } from '../socket';

interface WorldObject {
  id: string;
  type: string;
  position: {
    x: number;
    y: number;
    z: number;
  };
  properties: {
    caption: string;
    confidence: number;
    class: string;
    timestamp: number;
  };
  last_updated: string;
}

interface WorldState {
  objects: { [key: string]: WorldObject };
  behavior_tree: any;
  last_updated: string;
}

const WorldState: React.FC = () => {
  const [worldState, setWorldState] = useState<WorldState | null>(null);

  useEffect(() => {
    // Request initial state
    socket.emit('request_world_state');

    // Listen for state updates
    socket.on('state_update', (newState: WorldState) => {
      setWorldState(newState);
    });

    return () => {
      socket.off('state_update');
    };
  }, []);

  if (!worldState) {
    return <div className="p-2">Loading world state...</div>;
  }

  return (
    <div className="p-2">
      <h2 className="text-lg font-bold mb-2">Detected Objects</h2>
      <div className="grid grid-cols-1 gap-2">
        {Object.entries(worldState.objects).map(([id, obj]) => (
          <div key={id} className="bg-gray-50 p-2 rounded border border-gray-200">
            <div className="flex justify-between items-start">
              <div className="flex-1">
                <h3 className="font-bold text-sm truncate">
                  {obj.properties.caption || 'Unnamed Object'}
                </h3>
                <div className="text-xs text-gray-600">
                  <span className="inline-block mr-2">ID: {id}</span>
                  <span className="inline-block">
                    Pos: ({obj.position.x.toFixed(1)}, {obj.position.y.toFixed(1)}, {obj.position.z.toFixed(1)})
                  </span>
                </div>
              </div>
              <span className="text-xs bg-blue-100 text-blue-800 px-1.5 py-0.5 rounded">
                {(obj.properties.confidence * 100).toFixed(0)}%
              </span>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default WorldState; 