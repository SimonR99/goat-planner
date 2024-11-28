import os
import sys
from behaviortree.tree import Tree

# Import all actions dynamically from the actions/ folder
def import_actions():
    actions_dir = os.path.join(os.path.dirname(__file__), "actions")
    sys.path.append(actions_dir)

    actions = {}
    for filename in os.listdir(actions_dir):
        if filename.endswith(".py") and filename != "__init__.py":
            module_name = filename[:-3]
            module = __import__(module_name)
            actions[module_name] = module
    return actions

# Main script
if __name__ == "__main__":
    # Path to the XML file
    xmlfile = "test.xml"

    # Create the behavior tree
    bt_tree = Tree()

    # Dynamically import actions
    action_modules = import_actions()
    func_list = bt_tree.get_func_name(xmlfile)
    print("Functions in XML:", func_list)


    # Map the action names in the XML to their corresponding Python functions
    eval_func = []
    for func_name in func_list:
        if func_name.startswith("SaySomething."):
            # Extract module and message from the XML attribute
            module_name, func_name = func_name.split(".")
            eval_func.append(lambda: action_modules[module_name].SaySomething("Task complete!"))
        else:
            # For other actions
            module_name, func_name = func_name.split(".")
            eval_func.append(getattr(action_modules[module_name], func_name))

    # Generate the tree
    bt_tree.gen_tree(xmlfile, eval_func)

    # Print the tree structure
    bt_tree.dump()

    # Run the behavior tree
    bt_tree.run()
