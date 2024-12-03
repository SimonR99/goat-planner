import json
import os
import sqlite3
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional


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
            "last_updated": self.last_updated,
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
        db_dir = Path.home() / ".goat_state"
        db_dir.mkdir(exist_ok=True)

        # Store the database path
        self.db_path = str(db_dir / "goat_state.db")

        # Initialize database
        self._setup_database()

        # Initialize behavior tree
        self.behavior_tree = {}

        # Load conversations from database
        self._load_conversations()

    def _get_connection(self):
        """Create a new database connection"""
        return sqlite3.connect(self.db_path)

    def _setup_database(self):
        """Set up database tables"""
        with self._get_connection() as conn:
            cursor = conn.cursor()

            # Create objects table
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS objects (
                    id TEXT PRIMARY KEY,
                    type TEXT,
                    position TEXT,
                    properties TEXT,
                    last_updated TEXT
                )
            """
            )

            # Create conversations table
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS conversations (
                    id TEXT PRIMARY KEY,
                    name TEXT,
                    messages TEXT,
                    last_updated TEXT
                )
            """
            )

            # Add behavior tree table
            cursor.execute(
                """
                CREATE TABLE IF NOT EXISTS behavior_tree (
                    id TEXT PRIMARY KEY,
                    tree TEXT,
                    last_updated TEXT
                )
            """
            )
            conn.commit()

    def update_object(
        self, obj_id: str, obj_type: str, position: Dict[str, float], properties: Dict
    ) -> None:
        """Update or add an object to the world state"""
        try:
            with self._get_connection() as conn:
                cursor = conn.cursor()
                position_json = json.dumps(position)
                properties_json = json.dumps(properties)
                timestamp = datetime.now().isoformat()

                cursor.execute(
                    """
                    INSERT OR REPLACE INTO objects 
                    (id, type, position, properties, last_updated)
                    VALUES (?, ?, ?, ?, ?)
                """,
                    (obj_id, obj_type, position_json, properties_json, timestamp),
                )

                conn.commit()
        except Exception as e:
            print(f"Error updating object in database: {e}")

    def get_objects(self) -> Dict[str, WorldObject]:
        """Get all objects in the world state"""
        try:
            with self._get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("SELECT * FROM objects")
                rows = cursor.fetchall()

                objects = {}
                for row in rows:
                    obj = WorldObject(
                        id=row[0],
                        type=row[1],
                        position=json.loads(row[2]),
                        properties=json.loads(row[3]),
                        last_updated=row[4],
                    )
                    objects[obj.id] = obj

                return objects
        except Exception as e:
            print(f"Error getting objects from database: {e}")
            return {}

    def remove_object(self, obj_id: str) -> bool:
        """Remove an object from the world state"""
        try:
            with self._get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("DELETE FROM objects WHERE id = ?", (obj_id,))
                conn.commit()
                return True
        except Exception as e:
            print(f"Error removing object from database: {e}")
            return False

    def clear_objects(self) -> None:
        """Clear all objects from the world state"""
        try:
            with self._get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("DELETE FROM objects")
                conn.commit()
        except Exception as e:
            print(f"Error clearing objects from database: {e}")

    def update_behavior_tree(self, tree: Dict) -> None:
        """Update the behavior tree state and persist it"""
        try:
            self.behavior_tree = tree
            tree_json = json.dumps(tree)
            timestamp = datetime.now().isoformat()
            self.behavior_tree_last_updated = timestamp  # Update the last_updated time

            with self._get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("DELETE FROM behavior_tree WHERE id = ?", ("current",))
                cursor.execute(
                    """
                    INSERT INTO behavior_tree 
                    (id, tree, last_updated)
                    VALUES (?, ?, ?)
                """,
                    ("current", tree_json, timestamp),
                )
                conn.commit()
        except Exception as e:
            print(f"Error updating behavior tree in database: {e}")

    def get_behavior_tree(self) -> Dict:
        """Get the current behavior tree from database"""
        try:
            with self._get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute(
                    "SELECT tree, last_updated FROM behavior_tree WHERE id = ?",
                    ("current",),
                )
                row = cursor.fetchone()

                if row and row[0]:
                    self.behavior_tree = json.loads(row[0])
                    self.behavior_tree_last_updated = row[
                        1
                    ]  # Retrieve the last_updated time
                else:
                    self.behavior_tree = {}
                    self.behavior_tree_last_updated = None

                return self.behavior_tree
        except Exception as e:
            print(f"Error getting behavior tree from database: {e}")
            return {}

    def get_full_state(self) -> Dict:
        """Get the complete state as a dictionary"""
        objects_dict = {}
        for obj_id, obj in self.get_objects().items():
            objects_dict[obj_id] = obj.to_dict()

        return {
            "objects": objects_dict,
            "behavior_tree": self.behavior_tree,
            "last_updated": datetime.now().isoformat(),
        }

    # Add conversation methods back
    def get_conversations(self):
        """Get all conversations"""
        # Ensure we have the latest conversations from the database
        self._load_conversations()
        return self.conversations

    def add_conversation(self, conversation):
        """Add a new conversation"""
        try:
            with self._get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute(
                    """
                    INSERT INTO conversations 
                    (id, name, messages, last_updated)
                    VALUES (?, ?, ?, ?)
                """,
                    (
                        conversation["id"],
                        conversation["name"],
                        json.dumps(conversation.get("messages", [])),
                        datetime.now().isoformat(),
                    ),
                )
                conn.commit()

                # Update in-memory state after successful database insert
                self.conversations.append(conversation)
        except Exception as e:
            print(f"Error adding conversation to database: {e}")

    def update_conversation(self, conversation_id: str, updates: Dict):
        """Update an existing conversation"""
        try:
            with self._get_connection() as conn:
                cursor = conn.cursor()

                # If we're updating the name
                if "name" in updates:
                    cursor.execute(
                        """
                        UPDATE conversations 
                        SET name = ?, last_updated = ?
                        WHERE id = ?
                    """,
                        (updates["name"], datetime.now().isoformat(), conversation_id),
                    )

                # If we're updating messages
                if "messages" in updates:
                    cursor.execute(
                        """
                        UPDATE conversations 
                        SET messages = ?, last_updated = ?
                        WHERE id = ?
                    """,
                        (
                            json.dumps(updates["messages"]),
                            datetime.now().isoformat(),
                            conversation_id,
                        ),
                    )

                conn.commit()

                # Update in-memory state
                for conv in self.conversations:
                    if conv["id"] == conversation_id:
                        conv.update(updates)
                        break

                # Reload conversations to ensure consistency
                self._load_conversations()

        except Exception as e:
            print(f"Error updating conversation in database: {e}")

    def delete_conversation(self, conversation_id):
        """Delete a conversation"""
        try:
            with self._get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute(
                    "DELETE FROM conversations WHERE id = ?", (conversation_id,)
                )
                conn.commit()

                # Update in-memory state
                self.conversations = [
                    c for c in self.conversations if c["id"] != conversation_id
                ]
        except Exception as e:
            print(f"Error deleting conversation from database: {e}")

    def _load_conversations(self):
        """Load conversations from database"""
        try:
            with self._get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute(
                    "SELECT id, name, messages, last_updated FROM conversations"
                )
                rows = cursor.fetchall()

                self.conversations = []
                for row in rows:
                    conversation = {
                        "id": row[0],
                        "name": row[1],
                        "messages": json.loads(row[2]) if row[2] else [],
                        "last_updated": row[3],
                    }
                    self.conversations.append(conversation)
        except Exception as e:
            print(f"Error loading conversations from database: {e}")
            self.conversations = []

    def _save_conversations(self):
        """Save conversations to database"""
        try:
            with self._get_connection() as conn:
                cursor = conn.cursor()
                for conv in self.conversations:
                    cursor.execute(
                        """
                        INSERT OR REPLACE INTO conversations 
                        (id, name, messages, last_updated)
                        VALUES (?, ?, ?, ?)
                    """,
                        (
                            conv["id"],
                            conv["name"],
                            json.dumps(conv.get("messages", [])),
                            datetime.now().isoformat(),
                        ),
                    )
                conn.commit()
        except Exception as e:
            print(f"Error saving conversations to database: {e}")
