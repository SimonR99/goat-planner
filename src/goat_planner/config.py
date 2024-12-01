import os

# Get the project root directory (assuming config.py is in src/goat_planner)
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Define common paths
CONVERSATIONS_PATH = os.path.join(PROJECT_ROOT, "data", "conversations.json")

# Create data directory if it doesn't exist
os.makedirs(os.path.dirname(CONVERSATIONS_PATH), exist_ok=True) 