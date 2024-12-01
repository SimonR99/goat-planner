import json

class BehaviorTree:
    def __init__(self):
        self.tree = {
            "root": {
                "main_tree_to_execute": "MainTree",
                "BehaviorTree": {
                    "ID": "MainTree",
                    "RecoveryNode": {
                        "name": "Root",
                        "number_of_retries": "0",
                        "children": []
                    }
                }
            }
        }

    def validate_tree(self, tree):
        if not isinstance(tree, dict):
            return False
            
        # Check if it's the root structure
        if "root" in tree:
            if "main_tree_to_execute" not in tree["root"] or "BehaviorTree" not in tree["root"]:
                return False
            return self.validate_node(tree["root"]["BehaviorTree"])
            
        # For individual nodes
        return self.validate_node(tree)

    def validate_node(self, node):
        if not isinstance(node, dict):
            return False
            
        # Allow any property in the node
        for key, value in node.items():
            # If the value is a dict, recursively validate it
            if isinstance(value, dict):
                if not self.validate_node(value):
                    return False
            # If the value is a list, validate each dict in the list
            elif isinstance(value, list):
                for item in value:
                    if isinstance(item, dict):
                        if not self.validate_node(item):
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