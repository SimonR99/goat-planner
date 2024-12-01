import json
import os
import time
import uuid
from ipaddress import ip_address

import ollama
from flask import Flask, jsonify
from flask_cors import CORS  # Add this import
from flask_socketio import SocketIO, emit
from ollama import Client

from goat_planner.behavior_tree import BehaviorTree
from goat_planner.goat_controller import GoatController
from goat_planner.goat_state import GoatState

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes
socketio = SocketIO(app, cors_allowed_origins="*")


def message_callback(conversation_id, message):
    socketio.emit(
        "new_message", {"conversationId": conversation_id, "message": message}
    )


def plan_update_callback(tree):
    socketio.emit("plan_update", tree)


def state_update_callback(state):
    socketio.emit("state_update", state)


# Initialize GoatController with Socket.IO callbacks
goat_controller = GoatController(
    on_message_callback=message_callback,
    on_plan_update_callback=plan_update_callback,
    on_state_update_callback=state_update_callback,
)


@socketio.on("connect")
def handle_connect():
    print("Client connected")
    emit("conversations", goat_controller.conversations)


@socketio.on("disconnect")
def handle_disconnect():
    print("Client disconnected")


@socketio.on("new_conversation")
def handle_new_conversation(data=None):
    conversation = goat_controller.create_conversation(
        initial_message=data.get("message") if data else None
    )
    emit("conversations", goat_controller.conversations)


@socketio.on("rename_conversation")
def handle_rename_conversation(data):
    conversation_id = data["id"]
    new_name = data["name"]
    for conv in goat_controller.conversations:
        if conv["id"] == conversation_id:
            conv["name"] = new_name
            break
    goat_controller.state.update_conversation(conversation_id, {"name": new_name})
    emit("conversations", goat_controller.conversations)


@socketio.on("get_conversations")
def handle_get_conversations():
    emit("conversations", goat_controller.conversations)


@socketio.on("chat_message")
def handle_chat_message(data):
    conversation_id = data["conversationId"]
    message = data["message"]

    # Process message through GoatController
    goat_controller.process_message(conversation_id, message)
    emit("ai_response_complete", {"conversationId": conversation_id})


@app.route("/api/behavior_tree", methods=["GET"])
def get_behavior_tree():
    tree = goat_controller.get_behavior_tree()
    print("Sending behavior tree:", json.dumps(tree, indent=2))
    return jsonify(tree)


@socketio.on("get_behavior_tree")
def handle_get_behavior_tree():
    emit("tree_update", goat_controller.get_behavior_tree())


@socketio.on("delete_conversation")
def handle_delete_conversation(data):
    conversation_id = data["id"]
    goat_controller.delete_conversation(conversation_id)
    emit("conversations", goat_controller.conversations)


@socketio.on("request_plan_update")
def handle_request_plan_update():
    emit("plan_update", goat_controller.get_behavior_tree())


@app.route("/api/world_state", methods=["GET"])
def get_world_state():
    return jsonify(goat_controller.state.get_full_state())


@socketio.on("request_world_state")
def handle_request_world_state():
    emit("state_update", goat_controller.state.get_full_state())


@socketio.on("toggle_tts")
def handle_tts_toggle(data):
    """Handle TTS toggle from frontend"""
    enabled = data.get("enabled", False)
    goat_controller.use_tts = enabled
    if enabled:
        goat_controller.enable_tts()
    elif not enabled:
        goat_controller.disable_tts()


if __name__ == "__main__":
    socketio.run(app, debug=True)
