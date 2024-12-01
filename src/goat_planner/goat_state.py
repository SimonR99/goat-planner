from typing import Dict, List, Optional
import json
from dataclasses import dataclass, asdict
from datetime import datetime
import os
from pathlib import Path
import sqlite3

@dataclass
class WorldObject:
    id: str
    type: str
    position: Dict[str, float]
    properties: Dict
    last_updated: str

    def to_dict(self) -> Dict:
        """Convert WorldObject to dictionary"""
        return {
            "id": self.id,
            "type": self.type,
            "position": self.position,
            "properties": self.properties,
            "last_updated": self.last_updated
        }

@dataclass
class GoatStateData:
    objects: Dict[str, WorldObject]
    behavior_tree: Dict
    last_updated: str

class GoatState:
    """
    Singleton class to maintain the central state of the Goat system using SQLite
    """
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(GoatState, cls).__new__(cls)
            cls._instance._initialize()
        return cls._instance

    def _initialize(self):
        # Create database directory if it doesn't exist
        db_dir = Path.home() / '.goat_state'
        db_dir.mkdir(exist_ok=True)
        
        # Connect to SQLite database
        self.db_path = db_dir / 'goat_state.db'
        self.conn = sqlite3.connect(str(self.db_path), check_same_thread=False)
        self.cursor = self.conn.cursor()
        
        # Create tables if they don't exist
        self._create_tables()
        
        # Initialize behavior tree and conversations
        self.behavior_tree = {}
        self.conversations = []

    def _create_tables(self):
        # Create objects table
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS objects (
                id TEXT PRIMARY KEY,
                type TEXT,
                position TEXT,
                properties TEXT,
                last_updated TEXT
            )
        ''')
        
        # Create conversations table
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS conversations (
                id TEXT PRIMARY KEY,
                name TEXT,
                messages TEXT,
                last_updated TEXT
            )
        ''')
        self.conn.commit()

    def update_object(self, obj_id: str, obj_type: str, position: Dict[str, float], 
                     properties: Dict) -> None:
        """Update or add an object to the world state"""
        try:
            position_json = json.dumps(position)
            properties_json = json.dumps(properties)
            timestamp = datetime.now().isoformat()
            
            self.cursor.execute('''
                INSERT OR REPLACE INTO objects 
                (id, type, position, properties, last_updated)
                VALUES (?, ?, ?, ?, ?)
            ''', (obj_id, obj_type, position_json, properties_json, timestamp))
            
            self.conn.commit()
        except Exception as e:
            print(f"Error updating object in database: {e}")

    def get_objects(self) -> Dict[str, WorldObject]:
        """Get all objects in the world state"""
        try:
            self.cursor.execute('SELECT * FROM objects')
            rows = self.cursor.fetchall()
            
            objects = {}
            for row in rows:
                obj = WorldObject(
                    id=row[0],
                    type=row[1],
                    position=json.loads(row[2]),
                    properties=json.loads(row[3]),
                    last_updated=row[4]
                )
                objects[obj.id] = obj
            
            return objects
        except Exception as e:
            print(f"Error getting objects from database: {e}")
            return {}

    def remove_object(self, obj_id: str) -> bool:
        """Remove an object from the world state"""
        try:
            self.cursor.execute('DELETE FROM objects WHERE id = ?', (obj_id,))
            self.conn.commit()
            return True
        except Exception as e:
            print(f"Error removing object from database: {e}")
            return False

    def clear_objects(self) -> None:
        """Clear all objects from the world state"""
        try:
            self.cursor.execute('DELETE FROM objects')
            self.conn.commit()
        except Exception as e:
            print(f"Error clearing objects from database: {e}")

    def update_behavior_tree(self, tree: Dict) -> None:
        """Update the behavior tree state"""
        self.behavior_tree = tree

    def get_behavior_tree(self) -> Dict:
        """Get the current behavior tree"""
        return self.behavior_tree

    def get_full_state(self) -> Dict:
        """Get the complete state as a dictionary"""
        objects_dict = {}
        for obj_id, obj in self.get_objects().items():
            objects_dict[obj_id] = obj.to_dict()
            
        return {
            "objects": objects_dict,
            "behavior_tree": self.behavior_tree,
            "last_updated": datetime.now().isoformat()
        }

    # Add conversation methods back
    def get_conversations(self):
        """Get all conversations"""
        return self.conversations

    def add_conversation(self, conversation):
        """Add a new conversation"""
        self.conversations.append(conversation)
        self._save_conversations()

    def update_conversation(self, conversation_id, updates):
        """Update an existing conversation"""
        for conv in self.conversations:
            if conv['id'] == conversation_id:
                conv.update(updates)
                break
        self._save_conversations()

    def delete_conversation(self, conversation_id):
        """Delete a conversation"""
        self.conversations = [c for c in self.conversations if c['id'] != conversation_id]
        self._save_conversations()

    def _save_conversations(self):
        """Save conversations to database"""
        try:
            for conv in self.conversations:
                self.cursor.execute('''
                    INSERT OR REPLACE INTO conversations 
                    (id, name, messages, last_updated)
                    VALUES (?, ?, ?, ?)
                ''', (
                    conv['id'],
                    conv['name'],
                    json.dumps(conv.get('messages', [])),
                    datetime.now().isoformat()
                ))
            self.conn.commit()
        except Exception as e:
            print(f"Error saving conversations: {e}")
  