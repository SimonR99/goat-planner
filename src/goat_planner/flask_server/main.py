from ipaddress import ip_address
from flask import Flask, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS  # Add this import
import ollama
import time
import uuid
import json
import os
from behavior_tree import BehaviorTree
from ollama import Client

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes
socketio = SocketIO(app, cors_allowed_origins="*")

# Load conversations from file or initialize empty list
CONVERSATIONS_FILE = 'conversations.json'
if os.path.exists(CONVERSATIONS_FILE):
    with open(CONVERSATIONS_FILE, 'r') as f:
        conversations = json.load(f)
else:
    conversations = []

behavior_tree = BehaviorTree()

def save_conversations():
    with open(CONVERSATIONS_FILE, 'w') as f:
        json.dump(conversations, f)

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

This example shows a sequence of three main actions:
1. Open Door: A fallback node that first checks if the door is open, and if not, retries opening the door up to 5 times.
2. Enter Room: A simple action to enter the room once the door is open.
3. Close Door: Another simple action to close the door after entering.

Remember, generating a plan is only necessary when the user explicitly requests a task to be performed. For general questions, simply provide an informative answer without a plan.

Always enclose the plan within <plan></plan> tags. This is crucial for proper processing.

Always strive to be helpful, clear, and concise in your responses. Refer to yourself as GoatBrain when appropriate.
"""

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('conversations', conversations)

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('new_conversation')
def handle_new_conversation(data=None):
    conversation_id = str(uuid.uuid4())
    new_conversation = {
        'id': conversation_id,
        'name': f'Conversation {len(conversations) + 1}',
        'messages': []
    }
    conversations.append(new_conversation)
    save_conversations()
    emit('conversations', conversations)

    # If an initial message was provided, process it
    if data and 'message' in data:
        handle_chat_message({'conversationId': conversation_id, 'message': data['message']})

@socketio.on('rename_conversation')
def handle_rename_conversation(data):
    conversation_id = data['id']
    new_name = data['name']
    for conv in conversations:
        if conv['id'] == conversation_id:
            conv['name'] = new_name
            break
    save_conversations()
    emit('conversations', conversations)

@socketio.on('get_conversations')
def handle_get_conversations():
    emit('conversations', conversations)

@socketio.on('chat_message')
def handle_chat_message(data):
    conversation_id = data['conversationId']
    user_message = data['message']
    conversation = next((c for c in conversations if c['id'] == conversation_id), None)
    
    if not conversation:
        return

    # Add user message to conversation
    user_message_obj = {'text': user_message, 'isUser': True}
    conversation['messages'].append(user_message_obj)
    emit('new_message', {'conversationId': conversation_id, 'message': user_message_obj})

    # Emit new_message event to indicate the start of AI response
    emit('new_message', {'conversationId': conversation_id, 'message': {'text': '', 'isUser': False}})

    # Generate a response using Ollama
    try:
        current_state = f"Current state: {behavior_tree.to_json()}\n\n"
        ollama_messages = [
            {'role': 'system', 'content': SYSTEM_MESSAGE + current_state},
            *[{'role': 'user' if msg['isUser'] else 'assistant', 'content': msg['text']} for msg in conversation['messages']]
        ]

        
        client = Client(host='http://0.0.0.0:11434')

        stream = client.chat(
            model='llama3.2',
            messages=ollama_messages,
            stream=True,
        )

        ai_response = ""
        plan_json = ""
        in_plan = False
        plan_updated = False

        for chunk in stream:
            content = chunk['message']['content']
            ai_response += content

            if '<plan>' in content:
                in_plan = True
                plan_json = ""
            elif '</plan>' in content:
                in_plan = False
                try:
                    plan_data = json.loads(plan_json)
                    if behavior_tree.update_tree(plan_data):
                        print("Updated behavior tree:", json.dumps(plan_data, indent=2))
                        emit('plan_update', behavior_tree.get_tree())
                        plan_updated = True
                    else:
                        print("Invalid plan structure:", json.dumps(plan_data, indent=2))
                except json.JSONDecodeError as e:
                    print("Error decoding plan JSON:", e)
                    print("Raw plan JSON:", plan_json)

            if in_plan:
                plan_json += content.replace('<plan>', '').replace('</plan>', '')
            else:
                emit('new_message', {'conversationId': conversation_id, 'message': {'text': content, 'isUser': False}})

        # Add complete AI response to conversation
        ai_message_obj = {'text': ai_response, 'isUser': False}
        conversation['messages'].append(ai_message_obj)

        # If no plan was found in the response, try to extract it
        if not plan_updated:
            try:
                # Look for JSON-like structure in the response
                json_start = ai_response.find('{')
                json_end = ai_response.rfind('}')
                if json_start != -1 and json_end != -1:
                    potential_json = ai_response[json_start:json_end+1]
                    plan_data = json.loads(potential_json)
                    if behavior_tree.update_tree(plan_data):
                        print("Updated behavior tree from extracted JSON:", json.dumps(plan_data, indent=2))
                        emit('plan_update', behavior_tree.get_tree())
                        plan_updated = True
                    else:
                        print("Invalid plan structure from extracted JSON:", json.dumps(plan_data, indent=2))
                else:
                    print("No JSON-like structure found in the response")
            except json.JSONDecodeError as e:
                print("Error decoding extracted JSON:", e)
                print("Extracted potential JSON:", potential_json)

        # Only update to idle state if no plan was found or updated
        if not plan_updated:
            idle_tree = {
                "name": "Root",
                "type": "sequence",
                "children": [
                    {
                        "name": "Idle Loop",
                        "type": "loop",
                        "children": [
                            {
                                "name": "Wait for Input",
                                "type": "action",
                                "children": []
                            },
                            {
                                "name": "Process Input",
                                "type": "fallback",
                                "children": [
                                    {
                                        "name": "Answer Question",
                                        "type": "action",
                                        "children": []
                                    },
                                    {
                                        "name": "Perform Task",
                                        "type": "sequence",
                                        "children": [
                                            {
                                                "name": "Plan Task",
                                                "type": "action",
                                                "children": []
                                            },
                                            {
                                                "name": "Execute Task",
                                                "type": "action",
                                                "children": []
                                            }
                                        ]
                                    }
                                ]
                            }
                        ]
                    }
                ]
            }
            behavior_tree.update_tree(idle_tree)
            emit('plan_update', behavior_tree.get_tree())

    except ollama.ResponseError as e:
        print('Error:', e.error)
        error_message = {'text': f"Error: {e.error}", 'isUser': False}
        conversation['messages'].append(error_message)
        emit('new_message', {'conversationId': conversation_id, 'message': error_message})

    save_conversations()
    emit('ai_response_complete', {'conversationId': conversation_id})

@app.route('/api/behavior_tree', methods=['GET'])
def get_behavior_tree():
    tree = behavior_tree.get_tree()
    print("Sending behavior tree:", json.dumps(tree, indent=2))
    return jsonify(tree)

@socketio.on('get_behavior_tree')
def handle_get_behavior_tree():
    emit('tree_update', behavior_tree.get_tree())

@socketio.on('delete_conversation')
def handle_delete_conversation(data):
    conversation_id = data['id']
    global conversations
    conversations = [conv for conv in conversations if conv['id'] != conversation_id]
    save_conversations()
    emit('conversations', conversations)

@socketio.on('request_plan_update')
def handle_request_plan_update():
    emit('plan_update', behavior_tree.get_tree())

if __name__ == '__main__':
    socketio.run(app, debug=True)