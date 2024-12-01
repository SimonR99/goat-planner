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
  properties: Record<string, any>;
  last_updated: string;
}

interface WorldState {
  objects: Record<string, WorldObject>;
  behavior_tree: Record<string, any>;
  last_updated: string;
}

export const WorldState: React.FC = () => {
  const [worldState, setWorldState] = useState<WorldState | null>(null);

  useEffect(() => {
    // Request initial state
    socket.emit('request_world_state');

    // Listen for state updates
    socket.on('state_update', (state: WorldState) => {
      setWorldState(state);
    });

    return () => {
      socket.off('state_update');
    };
  }, []);

  if (!worldState) {
    return <div>Loading world state...</div>;
  }

  return (
    <div className="world-state">
      <h2>World Objects</h2>
      <div className="objects-list">
        {Object.values(worldState.objects).map((obj) => (
          <div key={obj.id} className="object-card">
            <h3>{obj.type} ({obj.id})</h3>
            <p>Position: ({obj.position.x}, {obj.position.y}, {obj.position.z})</p>
            <div className="properties">
              {Object.entries(obj.properties).map(([key, value]) => (
                <p key={key}>{key}: {value}</p>
              ))}
            </div>
            <p className="timestamp">Last updated: {new Date(obj.last_updated).toLocaleString()}</p>
          </div>
        ))}
      </div>
    </div>
  );
}; 