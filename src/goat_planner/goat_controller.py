import json
import os
import uuid
from typing import Callable, Dict, List, Optional

from ollama import Client, ResponseError

from .behavior_tree import BehaviorTree
from .goat_state import GoatState


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
- action: A leaf node representing a specific action
- loop: Continuously executes its child nodes

Example of a correct plan output:
<plan>
{
  "name": "Root",
  "type": "sequence",
  "children": [
    {
      "name": "Open Door",
      "type": "fallback",
      "children": [
        {
          "name": "Check if Door is Open",
          "type": "action",
          "children": []
        },
        {
          "name": "Open Door",
          "type": "retry",
          "children": [
            {
              "name": "Attempt to Open Door",
              "type": "action",
              "children": []
            }
          ],
          "attempts": 5
        }
      ]
    },
    {
      "name": "Enter Room",
      "type": "action",
      "children": []
    },
    {
      "name": "Close Door",
      "type": "action",
      "children": []
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
    ):

        self.ollama_host = ollama_host
        self.model = model
        self.on_message_callback = on_message_callback
        self.on_plan_update_callback = on_plan_update_callback
        self.on_state_update_callback = on_state_update_callback

        self.behavior_tree = BehaviorTree()
        self.ollama_client = Client(host=ollama_host)
        self.state = GoatState()

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
        self.state.conversations = [
            conv for conv in self.conversations if conv["id"] != conversation_id
        ]
        if len(self.conversations) < initial_length:
            self.state._save_conversations()
            return True
        return False

    def get_behavior_tree(self) -> Dict:
        return self.behavior_tree.get_tree()

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

            # Update system message with context
            system_message = self.SYSTEM_MESSAGE
            system_message += f"\nWorld State: {json.dumps(context['world_state'])}\n"

            current_state = f"Current state: {self.behavior_tree.to_json()}\n\n"

            ollama_messages = [
                {"role": "system", "content": system_message + current_state},
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

        for chunk in stream:
            content = chunk["message"]["content"]
            ai_response += content

            # Process plan tags and update behavior tree
            if "<plan>" in content:
                in_plan = True
                plan_json = ""
            elif "</plan>" in content:
                in_plan = False
                self._try_update_plan(plan_json)
                plan_updated = True

            if in_plan:
                plan_json += content.replace("<plan>", "").replace("</plan>", "")
            elif self.on_message_callback:
                self.on_message_callback(
                    conversation_id, {"text": content, "isUser": False}
                )

        # Add complete AI response to conversation
        conversation = next(c for c in self.conversations if c["id"] == conversation_id)
        ai_message_obj = {"text": ai_response, "isUser": False}
        conversation["messages"].append(ai_message_obj)

        # If no plan was found, try to extract it from the response
        if not plan_updated:
            self._try_extract_plan(ai_response)

        return {
            "success": True,
            "response": ai_response,
            "conversation_id": conversation_id,
        }

    def _try_update_plan(self, plan_json: str) -> bool:
        try:
            plan_data = json.loads(plan_json)
            if self.behavior_tree.update_tree(plan_data):
                if self.on_plan_update_callback:
                    self.on_plan_update_callback(self.behavior_tree.get_tree())
                return True
        except json.JSONDecodeError:
            pass
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