import json
import sys
from typing import Dict, Optional
import argparse

from goat_planner.goat_controller import GoatController


class CliChat:
    def __init__(self, use_tts: bool = False):
        self.controller = GoatController(
            on_message_callback=self.on_message,
            on_plan_update_callback=self.on_plan_update,
            on_state_update_callback=self.on_state_update,
            use_tts=use_tts,
        )
        self.current_conversation = None
        self.current_response = ""
        self.is_streaming = False

    def on_message(self, conversation_id: str, message: Dict):
        """Callback for new messages"""
        if not message["isUser"]:
            if not self.is_streaming:
                self.is_streaming = True
                print("\nAI: ", end="", flush=True)

            # Print the new content only (not the full message)
            new_content = message["text"]
            print(new_content, end="", flush=True)

            # If the message ends with a newline, mark streaming as complete
            if new_content.endswith("\n"):
                self.is_streaming = False

    def on_plan_update(self, tree: Dict):
        """Callback for plan updates"""
        print("\n\nPlan Updated:")
        print(json.dumps(tree, indent=2))

    def on_state_update(self, state: Dict):
        """Callback for state updates"""
        print("\n\nState Updated:")
        print(json.dumps(state, indent=2))

    def start_new_conversation(self):
        """Start a new conversation"""
        self.current_conversation = self.controller.create_conversation()
        print("\nStarting new conversation...")
        print("Type 'exit' to quit, 'new' to start a new conversation")
        print("Enter your message: ")

    def run(self):
        """Run the interactive chat loop"""
        print("Welcome to GoatBrain CLI Chat!")
        self.start_new_conversation()

        while True:
            try:
                user_input = input("\nYou: ").strip()

                if user_input.lower() == "exit":
                    print("\nGoodbye!")
                    sys.exit(0)

                if user_input.lower() == "new":
                    self.start_new_conversation()
                    continue

                if not user_input:
                    continue

                if self.current_conversation:
                    # Reset streaming state for new message
                    self.is_streaming = False
                    self.current_response = ""

                    self.controller.process_message(
                        self.current_conversation["id"], user_input
                    )

                    # Ensure a newline after the response is complete
                    if self.is_streaming:
                        print()
                        self.is_streaming = False

            except KeyboardInterrupt:
                print("\nGoodbye!")
                sys.exit(0)
            except Exception as e:
                print(f"\nError: {str(e)}")
                print("Starting new conversation...")
                self.start_new_conversation()


def main():
    parser = argparse.ArgumentParser(description="GoatBrain CLI Chat")
    parser.add_argument("--tts", action="store_true", help="Enable text-to-speech")
    args = parser.parse_args()

    chat = CliChat(use_tts=args.tts)
    chat.run()


if __name__ == "__main__":
    main()
