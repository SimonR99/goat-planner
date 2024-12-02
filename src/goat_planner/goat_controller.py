import json
import os
import uuid
from typing import Callable, Dict, List, Optional

from ollama import Client, ResponseError

from .behavior_tree import BehaviorTree
from .goat_state import GoatState
from .models.text_to_speech import TextToSpeech


class GoatController:
    SYSTEM_MESSAGE = """
You are GoatBrain, an advanced AI robot assistant designed to help with questions and tasks. Your default state is an idle loop where you wait for input and then process it. When processing input, you either answer questions or perform tasks. When a user asks a question, answer it to the best of your ability. When a user requests a task to be performed, follow these steps:

1. Briefly acknowledge the task.
2. Generate a behavior tree in JSON format that represents the steps to complete the task.
3. IMPORTANT: Always enclose the behavior tree JSON within <plan></plan> tags.

The behavior tree should use the following node types:
- sequence: Executes children in order, stops if one fails
- fallback: Tries children in order until one succeeds
- retry: Retries its child node a specified number of times
- loop: Continuously executes its child nodes
- anything else: Represents a specific action

Example of a correct plan output:
<plan>
{
  "type": "Sequence",
  "name": "RetrieveFoodSequence",
  "nodes": [
    {
      "type": "Retry",
      "retries": "3",
      "nodes": [
        {
          "type": "Locate",
          "object": "apple",
          "location": "kitchen",
          "method": "camera_scan"
        }
      ]
    },
    {
      "type": "Fallback",
      "name": "LocateFoodFallback",
      "nodes": [
        {
          "type": "Locate",
          "object": "apple",
          "location": "kitchen",
          "method": "camera_scan"
        },
        {
          "type": "AskForHelp",
          "message": "Unable to locate the apple. Please assist."
        }
      ]
    },
    {
      "type": "Retry",
      "retries": "2",
      "nodes": [
        {
          "type": "NavigateTo",
          "location": "kitchen_table",
          "mode": "safe",
          "speed": "medium"
        }
      ]
    },
    {
      "type": "Retry",
      "retries": "2",
      "nodes": [
        {
          "type": "Pick",
          "object": "apple",
          "grip_strength": "medium",
          "precision": "high"
        }
      ]
    },
    {
      "type": "Retry",
      "retries": "2",
      "nodes": [
        {
          "type": "NavigateTo",
          "location": "dining_table",
          "mode": "direct",
          "speed": "fast"
        }
      ]
    },
    {
      "type": "Fallback",
      "name": "PlaceFoodFallback",
      "nodes": [
        {
          "type": "Place",
          "object": "apple",
          "surface": "dining_table",
          "orientation": "upright",
          "alignment": "center"
        },
        {
          "type": "AskForHelp",
          "message": "Unable to place the apple. Please assist."
        }
      ]
    }
  ]
}
</plan>

Remember, generating a plan is only necessary when the user explicitly requests a task to be performed. For general questions, simply provide an informative answer without a plan.

Always enclose the plan within <plan></plan> tags. This is crucial for proper processing.

Always strive to be helpful, clear, and concise in your responses. Refer to yourself as GoatBrain when appropriate.
"""

    def __init__(
        self,
        ollama_host: str = "http://0.0.0.0:11434",
        model: str = "llama3.2",
        on_message_callback: Optional[Callable] = None,
        on_plan_update_callback: Optional[Callable] = None,
        on_state_update_callback: Optional[Callable] = None,
        use_tts: bool = False,
    ):

        self.ollama_host = ollama_host
        self.model = model
        self.on_message_callback = on_message_callback
        self.on_plan_update_callback = on_plan_update_callback
        self.on_state_update_callback = on_state_update_callback

        self.behavior_tree = BehaviorTree()
        self.ollama_client = Client(host=ollama_host)
        self.state = GoatState()

        # TTS setup
        self.use_tts = use_tts
        self.tts = None  # Initialize TTS only when needed
        if use_tts:
            self.enable_tts()

    @property
    def conversations(self):
        return self.state.get_conversations()

    def create_conversation(self, initial_message: Optional[str] = None) -> Dict:
        conversation_id = str(uuid.uuid4())
        new_conversation = {
            "id": conversation_id,
            "name": f"Conversation {len(self.conversations) + 1}",
            "messages": [],
        }
        self.state.add_conversation(new_conversation)

        if initial_message:
            self.process_message(conversation_id, initial_message)

        return new_conversation

    def delete_conversation(self, conversation_id: str) -> bool:
        initial_length = len(self.conversations)
        self.state.delete_conversation(conversation_id)
        return len(self.conversations) < initial_length

    def get_behavior_tree(self) -> Dict:
        """Get the current behavior tree from database"""
        # Get the tree from the database instead of memory
        return self.state.get_behavior_tree()

    def process_message(
        self, conversation_id: str, message: str, context: Optional[Dict] = None
    ) -> Dict:
        """
        Process a message with optional context from different sources
        (e.g., ROS2, Shepherd, etc.)
        """
        conversation = next(
            (c for c in self.conversations if c["id"] == conversation_id), None
        )
        if not conversation:
            raise ValueError(f"Conversation {conversation_id} not found")

        # Add user message to conversation
        user_message = {"text": message, "isUser": True}
        conversation["messages"].append(user_message)

        if self.on_message_callback:
            self.on_message_callback(conversation_id, user_message)

        try:
            # Include world state in context
            if context is None:
                context = {}
            context["world_state"] = self.state.get_full_state()

            # Create messages list with system message first
            ollama_messages = [
                {"role": "system", "content": self.SYSTEM_MESSAGE},
                *[
                    {
                        "role": "user" if msg["isUser"] else "assistant",
                        "content": msg["text"],
                    }
                    for msg in conversation["messages"]
                ],
            ]

            # Process the message through Ollama
            response = self._process_ollama_response(conversation_id, ollama_messages)

            self.state._save_conversations()
            return response

        except ResponseError as e:
            error_response = {"error": str(e), "success": False}
            if self.on_message_callback:
                self.on_message_callback(
                    conversation_id, {"text": f"Error: {str(e)}", "isUser": False}
                )
            return error_response

    def _process_ollama_response(
        self, conversation_id: str, ollama_messages: List[Dict]
    ) -> Dict:

        stream = self.ollama_client.chat(
            model=self.model,
            messages=ollama_messages,
            stream=True,
        )

        ai_response = ""
        plan_json = ""
        in_plan = False
        plan_updated = False
        current_chunk = ""

        for chunk in stream:
            content = chunk["message"]["content"]
            ai_response += content
            current_chunk += content

            # Process plan tags and update behavior tree
            if "<plan>" in content:
                in_plan = True
                plan_json = ""

            if in_plan:
                plan_json += content.replace("<plan>", "").replace("</plan>", "")

            if "</plan>" in content:
                in_plan = False
                if self._try_update_plan(plan_json):
                    plan_updated = True

            # Send the chunk through callback
            if self.on_message_callback and current_chunk:
                self.on_message_callback(
                    conversation_id, {"text": current_chunk, "isUser": False}
                )
                current_chunk = ""

        # Add complete AI response to conversation
        conversation = next(c for c in self.conversations if c["id"] == conversation_id)
        ai_message_obj = {"text": ai_response, "isUser": False}
        conversation["messages"].append(ai_message_obj)

        # Process TTS after complete response
        if self.use_tts and self.tts and not in_plan:
            # Remove any plan JSON from the response
            clean_text = ai_response.split("<plan>")[0].strip()
            self.tts.speak(clean_text)

        # If no plan was found in streaming, try to extract it from complete response
        if not plan_updated:
            try:
                start_idx = ai_response.find("<plan>")
                end_idx = ai_response.find("</plan>")
                if start_idx != -1 and end_idx != -1:
                    plan_json = ai_response[start_idx + 6 : end_idx].strip()
                    self._try_update_plan(plan_json)
            except Exception as e:
                print(f"Error extracting plan from complete response: {e}")

        return {
            "success": True,
            "response": ai_response,
            "conversation_id": conversation_id,
        }

    def _try_update_plan(self, plan_json: str) -> bool:
        """Try to parse and update the behavior tree from JSON string"""
        try:
            # Clean up the plan JSON string
            clean_json = plan_json.strip()
            if not clean_json:
                return False

            plan_data = json.loads(clean_json)

            # Update both the behavior tree and state
            if self.behavior_tree.update_tree(plan_data):
                # Store the plan in the database through GoatState
                self.state.update_behavior_tree(plan_data)

                # Notify listeners about the plan update
                if self.on_plan_update_callback:
                    self.on_plan_update_callback(self.behavior_tree.get_tree())
                return True

        except json.JSONDecodeError as e:
            print(f"Failed to parse plan JSON: {e}")
        except Exception as e:
            print(f"Error updating plan: {e}")
        return False

    def _try_extract_plan(self, response: str) -> bool:
        try:
            json_start = response.find("{")
            json_end = response.rfind("}")
            if json_start != -1 and json_end != -1:
                potential_json = response[json_start : json_end + 1]
                plan_data = json.loads(potential_json)
                if self.behavior_tree.update_tree(plan_data):
                    if self.on_plan_update_callback:
                        self.on_plan_update_callback(self.behavior_tree.get_tree())
                    return True
        except json.JSONDecodeError:
            pass
        return False

    def update_world_object(
        self, obj_id: str, obj_type: str, position: Dict[str, float], properties: Dict
    ) -> None:
        """Update a world object and notify listeners"""
        self.state.update_object(obj_id, obj_type, position, properties)
        if self.on_state_update_callback:
            self.on_state_update_callback(self.state.get_full_state())

    def enable_tts(self):
        """Enable TTS if not already enabled"""
        if not self.tts:
            self.tts = TextToSpeech()
        self.use_tts = True

    def disable_tts(self):
        """Disable TTS"""
        self.tts = None
        self.use_tts = False
