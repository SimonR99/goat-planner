action_list = [
    {
        "name": "Wait",
        "description": "Wait for a specific duration",
        "params": {
            "duration": "float, duration of the waiting time in seconds, e.g., 2.0"
        },
    },
    {
        "name": "Navigate",
        "description": "Go to a destination",
        "params": {
            "x": "float, X coordinate of the destination, e.g., 1.5",
            "y": "float, Y coordinate of the destination, e.g., 0.0",
        },
    },
    {
        "name": "Pick",
        "description": "Pick up an object",
        "params": {
            "object": "string, name of the object to pick up, e.g., 'apple'",
            "grip_strength": "string, grip strength to use, options: 'low', 'medium', 'high'",
            "precision": "string, precision level, options: 'low', 'medium', 'high'",
        },
    },
    {
        "name": "Place",
        "description": "Place an object at a location",
        "params": {
            "object": "string, name of the object to place, e.g., 'apple'",
            "surface": "string, name of the surface to place the object on, e.g., 'table'",
            "orientation": "string, orientation of the object, e.g., 'upright'",
            "alignment": "string, alignment of the object, e.g., 'center'",
        },
    },
    {
        "name": "Locate",
        "description": "Find an object",
        "params": {"object": "string, name of the object to locate, e.g., 'apple'"},
        "output": {
            "position_x": "float, X coordinate of the object",
            "position_y": "float, Y coordinate of the object",
            "position_z": "float, Z coordinate of the object",
        },
    },
    {
        "name": "GetCurrentPosition",
        "description": "Retrieve the current position of the robot",
        "params": {},
        "output": {
            "position_x": "float, current X coordinate of the robot",
            "position_y": "float, current Y coordinate of the robot",
            "position_z": "float, current Z coordinate of the robot",
        },
    },
]

object_list = [
    {"name": "a water bottle", "position": {"x": 12.3, "y": 5.8, "z": 1.0}},
    {"name": "a first aid kit", "position": {"x": 24.7, "y": 3.5, "z": 0.8}},
    {"name": "a flashlight", "position": {"x": 9.2, "y": 4.1, "z": 0.5}},
    {"name": "a safety helmet", "position": {"x": 18.4, "y": 11.7, "z": 0.9}},
    {"name": "a fire extinguisher", "position": {"x": 5.3, "y": 2.2, "z": 1.4}},
    {"name": "a ladder", "position": {"x": 30.0, "y": 14.5, "z": 3.2}},
    {"name": "a smartphone", "position": {"x": 7.9, "y": 6.8, "z": 0.3}},
    {"name": "a thermal blanket", "position": {"x": 20.1, "y": 7.3, "z": 1.2}},
    {"name": "a pair of gloves", "position": {"x": 6.5, "y": 8.4, "z": 0.2}},
    {"name": "a backpack", "position": {"x": 15.8, "y": 3.3, "z": 0.9}},
    {"name": "a notebook and pen", "position": {"x": 11.6, "y": 9.1, "z": 0.5}},
    {"name": "a box of matches", "position": {"x": 3.4, "y": 1.8, "z": 0.1}},
    {"name": "a roll of duct tape", "position": {"x": 8.2, "y": 4.5, "z": 0.7}},
    {"name": "a crowbar", "position": {"x": 13.7, "y": 6.2, "z": 1.1}},
    {"name": "a set of keys", "position": {"x": 5.8, "y": 2.7, "z": 0.1}},
    {"name": "an emergency whistle", "position": {"x": 10.5, "y": 5.4, "z": 0.3}},
    {"name": "a tarp", "position": {"x": 28.7, "y": 12.3, "z": 2.0}},
    {"name": "a walkie-talkie", "position": {"x": 17.6, "y": 3.9, "z": 0.4}},
    {"name": "a pair of boots", "position": {"x": 14.5, "y": 4.2, "z": 0.6}},
    {"name": "a medical splint", "position": {"x": 22.9, "y": 8.8, "z": 0.7}},
    {"name": "a bag of sand", "position": {"x": 4.7, "y": 7.1, "z": 0.5}},
    {"name": "a car jack", "position": {"x": 6.3, "y": 1.2, "z": 1.3}},
    {"name": "a folding knife", "position": {"x": 9.4, "y": 2.8, "z": 0.1}},
    {"name": "a rope", "position": {"x": 3.9, "y": 5.7, "z": 0.8}},
    {"name": "a cooking pot", "position": {"x": 19.4, "y": 6.8, "z": 1.2}},
    {"name": "a can of soup", "position": {"x": 7.1, "y": 2.3, "z": 0.4}},
    {"name": "a bag of rice", "position": {"x": 21.5, "y": 3.1, "z": 1.0}},
    {"name": "a folding chair", "position": {"x": 26.7, "y": 9.4, "z": 1.8}},
    {"name": "a camping stove", "position": {"x": 18.1, "y": 7.9, "z": 1.4}},
    {"name": "a map", "position": {"x": 12.0, "y": 4.0, "z": 0.1}},
    {"name": "a compass", "position": {"x": 6.8, "y": 2.5, "z": 0.3}},
    {"name": "an emergency flare", "position": {"x": 16.9, "y": 8.3, "z": 0.5}},
    {"name": "a stack of wood", "position": {"x": 25.4, "y": 12.2, "z": 2.1}},
    {"name": "a hammer", "position": {"x": 10.2, "y": 5.1, "z": 0.9}},
    {"name": "a set of screwdrivers", "position": {"x": 13.5, "y": 2.8, "z": 0.7}},
    {"name": "a pair of binoculars", "position": {"x": 8.7, "y": 3.6, "z": 0.4}},
    {"name": "a sleeping bag", "position": {"x": 20.3, "y": 9.2, "z": 1.5}},
    {"name": "a toolkit", "position": {"x": 19.6, "y": 3.3, "z": 1.2}},
    {"name": "a fireproof blanket", "position": {"x": 7.3, "y": 5.5, "z": 0.6}},
    {"name": "a water filter", "position": {"x": 11.8, "y": 6.2, "z": 0.4}},
    {"name": "a pair of sunglasses", "position": {"x": 4.6, "y": 1.8, "z": 0.2}},
    {"name": "a shovel", "position": {"x": 22.4, "y": 10.3, "z": 1.6}},
    {
        "name": "a red apple sitting on the kitchen counter",
        "position": {"x": 1.5, "y": 2.0, "z": 0.8},
    },
    {
        "name": "an empty glass cup next to the sink",
        "position": {"x": 3.2, "y": 1.8, "z": 0.9},
    },
    {
        "name": "a wooden cutting board on the dining table",
        "position": {"x": 4.5, "y": 2.2, "z": 0.75},
    },
    {
        "name": "a silver knife laying on the counter",
        "position": {"x": 1.8, "y": 2.1, "z": 0.8},
    },
    {
        "name": "a ceramic plate with some bread crumbs",
        "position": {"x": 2.7, "y": 2.0, "z": 0.85},
    },
]

training_template = """You are GoatBrain, an AI assistant that processes questions and perform tasks.
Actions allowed:
{available_actions}

Objects allowed:
{object_list}
"""

short_template = """You are GoatBrain, an AI assistant that processes questions and tasks. For questions, provide direct answers. For tasks:
1. Acknowledge the task
2. Generate a behavior tree in {format_type} format
3. Always enclose the tree in <plan></plan> tags

Node types:
- Sequence: Executes in order, stops on failure
- Fallback: Tries until success
- Retry: Retries N times
- Loop: Continuous execution
- Other nodes: Specific actions

Example:
{example}

Actions allowed:
{available_actions}

Objects allowed:
{object_list}

Remember: Only generate plans for explicit task requests. Always use <plan></plan> tags. One plan per request."""

template = """You are GoatBrain, an advanced AI robot assistant designed to help with questions and tasks. Your default state is an idle loop where you wait for input and then process it. When processing input, you either answer questions or perform tasks. When a user asks a question, answer it to the best of your ability. When a user requests a task to be performed, follow these steps:

1. Briefly acknowledge the task.
2. Generate a behavior tree in the specified format ({format_type}) that represents the steps to complete the task.
3. IMPORTANT: Always enclose the behavior tree within <plan></plan> tags.

The behavior tree should use the following node types:
- Sequence: Executes children in order, stops if one fails
- Fallback: Tries children in order until one succeeds
- Retry: Retries its child node a specified number of times
- Loop: Continuously executes its child nodes
- anything else: Represents a specific action

### Example of a behavior tree:
Here is an example :

{example}

### End of example behavior tree

Remember, generating a plan is only necessary when the user explicitly requests a task to be performed. For general questions, simply provide an informative answer without a plan.

Always enclose the plan within <plan></plan> tags. This is crucial for proper processing. Only one plan can be generated for each request.

Always strive to be helpful, clear, and concise in your responses. Refer to yourself as GoatBrain when appropriate.

### Objects and Descriptions:
### Available Actions:
Only use the following actions in your behavior trees:
{available_actions}

### Objects and Descriptions:
Only use the following objects in your behavior trees:
{object_list}
"""


question_example = """Can you retrieve the red apple from the kitchen counter and place it on the cutting board in the dining room?"""
answer_example = """I will locate the red apple on the kitchen counter, pick it up with medium grip strength and high precision, and then place it on the wooden cutting board on the dining table"""
json_example = """
<plan>
{
  "type": "Sequence",
  "name": "RetrieveAndPlaceAppleSequence",
  "nodes": [
    {
      "type": "Retry",
      "retries": 3,
      "nodes": [
        {
          "type": "Locate",
          "object": "red apple sitting on the kitchen counter",
          "position_x": "{apple_position_x}",
          "position_y": "{apple_position_y}",
          "position_z": "{apple_position_z}"
        }
      ]
    },
    {
      "type": "Retry",
      "retries": 2,
      "nodes": [
        {
          "type": "Navigate",
          "x": "{apple_position_x}",
          "y": "{apple_position_y}"
        }
      ]
    },
    {
      "type": "Retry",
      "retries": 2,
      "nodes": [
        {
          "type": "Pick",
          "object": "red apple sitting on the kitchen counter",
          "grip_strength": "medium",
          "precision": "high"
        }
      ]
    },
    {
      "type": "Retry",
      "retries": 2,
      "nodes": [
        {
          "type": "Place",
          "object": "red apple",
          "surface": "wooden cutting board on the dining table",
          "orientation": "upright",
          "alignment": "center"
        }
      ]
    }
  ]
}
</plan>
"""

xml_example = """
<plan>
<root main_tree_to_execute="RetrieveAndPlaceAppleSequence">
    <BehaviorTree ID="RetrieveAndPlaceAppleSequence">
        <Sequence name="RetrieveAndPlaceApple">
            <Retry num_attempts="3">
                <Locate object="red apple sitting on the kitchen counter" 
                        position_x="{apple_position_x}" 
                        position_y="{apple_position_y}" 
                        position_z="{apple_position_z}" 
                        method="camera_scan"/>
            </Retry>
            <Retry num_attempts="2">
                <Navigate x="{apple_position_x}" y="{apple_position_y}"/>
            </Retry>
            <Retry num_attempts="2">
                <Pick object="red apple sitting on the kitchen counter" 
                      grip_strength="medium" 
                      precision="high"/>
            </Retry>
            <Retry num_attempts="2">
                <Place object="red apple" 
                       surface="wooden cutting board on the dining table" 
                       orientation="upright" 
                       alignment="center"/>
            </Retry>
        </Sequence>
    </BehaviorTree>
</root>
</plan>
"""
