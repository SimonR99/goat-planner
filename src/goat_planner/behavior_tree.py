import json

class BehaviorTree:
    def __init__(self):
        self.tree = {
            "root": {
                "type": "BehaviorTree",
                "id": "MainTree",
                "nodes": []
            }
        }

    def validate_tree(self, tree):
        if not isinstance(tree, dict):
            return False
            
        # Check root structure
        if "root" not in tree:
            return False
            
        root = tree["root"]
        # Validate required fields in root
        if not all(key in root for key in ["type", "id", "nodes"]):
            return False
            
        # Validate root type
        if root["type"] != "BehaviorTree":
            return False
            
        # Validate nodes array
        if not isinstance(root["nodes"], list):
            return False
            
        # Validate each node in the tree
        return all(self.validate_node(node) for node in root["nodes"])

    def validate_node(self, node):
        if not isinstance(node, dict):
            return False
            
        # Check required fields for all nodes
        if not all(key in node for key in ["type", "name"]):
            return False
            
        # If node has child nodes, validate them
        if "nodes" in node:
            if not isinstance(node["nodes"], list):
                return False
            return all(self.validate_node(child) for child in node["nodes"])
            
        # If node has parameters, validate them
        if "parameters" in node:
            if not isinstance(node["parameters"], dict):
                return False
                
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