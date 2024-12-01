# Goat Planner: Robot Control with LLM

Goat Planner is a modular project for controlling robots using a language model (LLM). It integrates ROS2 for compatibility while maintaining the ability to run independently, ensuring flexibility for both simulated and real-world environments. The LLM used is a fine-tuned version of LLaMA3 that has been trained to generate behavior trees.

## Features

- Language Model Integration: Uses a fine-tuned Ollama model for task planning.
- Generate plans compatible with BehaviorTree.CPP
- ROS2 integration with distributed nodes
- Real-time pipeline.
- Web Interface: React-based frontend for user query and plan visualization.
- Voice capabilities: Speech-to-text and text-to-speech.

### Standalone installation

```bash
git clone https://github.com/SimonR99/goat-planner.git
cd goat-planner
pip install -r requirements.txt
pip install -e .
```

### Run the web interface

```bash
cd src/frontend
npm install
npm start
```

### Run the CLI chat

```bash
python -m goat_planner.cli_chat --tts
```

### ROS2 integration

Clone the repository in your ROS2 workspace and build the package with colcon:

```bash
colcon build --packages-select goat_behavior
```

Now, you need to launch all the services available so that the behavior tree can be executed.

```bash
source install/setup.bash
ros2 launch goat_behavior goat_behavior.launch.py
```

