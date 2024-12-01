from typing import Dict, List, Optional
import json
from dataclasses import dataclass, asdict
from datetime import datetime
import os
from .config import CONVERSATIONS_PATH

@dataclass
class WorldObject:
    id: str
    type: str
    position: Dict[str, float]
    properties: Dict
    last_updated: str

@dataclass
class GoatStateData:
    objects: Dict[str, WorldObject]
    behavior_tree: Dict
    conversations: List[Dict]
    last_updated: str

class GoatState:
    """
    Singleton class to maintain the central state of the Goat system
    """
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(GoatState, cls).__new__(cls)
            cls._instance._initialize()
        return cls._instance

    def _initialize(self):
        self.state = GoatStateData(
            objects={},
            behavior_tree={},
            conversations=self._load_conversations(),
            last_updated=datetime.now().isoformat()
        )

    def _load_conversations(self) -> List[Dict]:
        if os.path.exists(CONVERSATIONS_PATH):
            with open(CONVERSATIONS_PATH, 'r') as f:
                return json.load(f)
        return []

    def _save_conversations(self):
        # Ensure directory exists
        os.makedirs(os.path.dirname(CONVERSATIONS_PATH), exist_ok=True)
        with open(CONVERSATIONS_PATH, 'w') as f:
            json.dump(self.state.conversations, f, indent=2)

    def add_conversation(self, conversation: Dict):
        self.state.conversations.append(conversation)
        self._save_conversations()

    def update_conversation(self, conversation_id: str, updates: Dict):
        for conv in self.state.conversations:
            if conv['id'] == conversation_id:
                conv.update(updates)
                break
        self._save_conversations()

    def get_conversations(self) -> List[Dict]:
        return self.state.conversations

    def update_object(self, obj_id: str, obj_type: str, position: Dict[str, float], 
                     properties: Dict) -> None:
        """Update or add an object to the world state"""
        self.state.objects[obj_id] = WorldObject(
            id=obj_id,
            type=obj_type,
            position=position,
            properties=properties,
            last_updated=datetime.now().isoformat()
        )
        self.state.last_updated = datetime.now().isoformat()

    def remove_object(self, obj_id: str) -> bool:
        """Remove an object from the world state"""
        if obj_id in self.state.objects:
            del self.state.objects[obj_id]
            self.state.last_updated = datetime.now().isoformat()
            return True
        return False

    def get_objects(self) -> Dict[str, WorldObject]:
        """Get all objects in the world state"""
        return self.state.objects

    def update_behavior_tree(self, tree: Dict) -> None:
        """Update the behavior tree state"""
        self.state.behavior_tree = tree
        self.state.last_updated = datetime.now().isoformat()

    def get_behavior_tree(self) -> Dict:
        """Get the current behavior tree"""
        return self.state.behavior_tree

    def get_full_state(self) -> Dict:
        """Get the complete state as a dictionary"""
        return asdict(self.state)

    def clear_objects(self) -> None:
        """Clear all objects from the world state"""
        self.state.objects = {}
        self.state.last_updated = datetime.now().isoformat()

    def delete_conversation(self, conversation_id: str) -> None:
        """Delete a conversation by its ID"""
        self.state.conversations = [
            conv for conv in self.state.conversations 
            if conv["id"] != conversation_id
        ]
        self._save_conversations()
  