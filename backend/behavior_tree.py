import json

class BehaviorTree:
    def __init__(self):
        self.tree = {
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

    def update_tree(self, new_tree):
        if self.validate_tree(new_tree):
            self.tree = new_tree
            return True
        return False

    def validate_tree(self, tree):
        if not isinstance(tree, dict):
            return False
        if 'name' not in tree or 'type' not in tree:
            return False
        if tree['type'] not in ['sequence', 'fallback', 'retry', 'action', 'loop']:
            return False
        if 'children' in tree:
            if not isinstance(tree['children'], list):
                return False
            for child in tree['children']:
                if not self.validate_tree(child):
                    return False
        return True

    def get_tree(self):
        return self.tree

    def to_json(self):
        return json.dumps(self.tree)