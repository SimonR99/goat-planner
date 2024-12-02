import json

class BehaviorTree:
    def __init__(self):
        self.tree = {
            "type": "Sequence",
            "name": "Root",
            "nodes": []
        }

    def validate_tree(self, tree):
        if not isinstance(tree, dict):
            return False
            
        # Check required fields
        if not all(key in tree for key in ["type", "name", "nodes"]):
            return False
            
        # Validate nodes array
        if not isinstance(tree["nodes"], list):
            return False
            
        # Validate each node in the tree
        return all(self.validate_node(node) for node in tree["nodes"])

    def validate_node(self, node):
        if not isinstance(node, dict):
            return False
            
        # Check required fields for all nodes
        if not all(key in node for key in ["type"]):
            return False
            
        # Validate specific node types
        if node["type"] == "Retry" and "retries" not in node:
            return False
            
        # If node has child nodes, validate them
        if "nodes" in node:
            if not isinstance(node["nodes"], list):
                return False
            return all(self.validate_node(child) for child in node["nodes"])
                
        # Validate action nodes have required parameters
        if node["type"] in ["Locate", "NavigateTo", "Pick", "Place", "AskForHelp"]:
            return self.validate_action_node(node)
                
        return True

    def validate_action_node(self, node):
        """Validate parameters for specific action types"""
        required_params = {
            "Locate": ["object", "location", "method"],
            "NavigateTo": ["location", "mode", "speed"],
            "Pick": ["object", "grip_strength", "precision"],
            "Place": ["object", "surface", "orientation", "alignment"],
            "AskForHelp": ["message"]
        }
        
        if node["type"] in required_params:
            return all(param in node for param in required_params[node["type"]])
        return True

    def update_tree(self, new_tree):
        if self.validate_tree(new_tree):
            self.tree = new_tree
            return True
        return False

    def get_tree(self):
        return self.tree

    def to_json(self):
        return json.dumps(self.tree)