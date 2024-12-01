import xml.etree.ElementTree as ET

def validate_behavior_tree(bt_input):
    """
    Validate a Behavior Tree XML from a string or file.
    :param bt_input: A string containing the BT XML or the path to an XML file.
    :return: (is_valid, errors) where is_valid is a boolean and errors is a list of validation issues.
    """
    errors = []
    try:
        # Parse the input (string or file)
        if bt_input.strip().startswith("<"):
            tree = ET.ElementTree(ET.fromstring(bt_input))  # Parse from string
        else:
            tree = ET.parse(bt_input)  # Parse from file
        root = tree.getroot()
        
        # Check if the root element is `<root>`
        if root.tag != "root":
            errors.append("Root element must be <root>.")
        
        # Check if `main_tree_to_execute` attribute exists in `<root>`
        if "main_tree_to_execute" not in root.attrib:
            errors.append("The <root> element must have a 'main_tree_to_execute' attribute.")
        
        # Validate each <BehaviorTree> element
        for behavior_tree in root.findall("BehaviorTree"):
            # Check if ID attribute exists
            if "ID" not in behavior_tree.attrib:
                errors.append("<BehaviorTree> elements must have an 'ID' attribute.")
            
            # Validate the structure recursively
            def validate_node(node):
                valid_nodes = {"Sequence", "Fallback", "Action"}
                if node.tag not in valid_nodes:
                    errors.append(f"Invalid node <{node.tag}> found.")
                # Validate Action node attributes
                if node.tag == "Action" and "ID" not in node.attrib:
                    errors.append(f"<Action> node is missing 'ID' attribute.")
                # Recursively validate child nodes
                for child in node:
                    validate_node(child)
            
            # Start validating from the top level of the behavior tree
            for child in behavior_tree:
                validate_node(child)
    except ET.ParseError as e:
        errors.append(f"XML Parsing Error: {str(e)}")
    except Exception as e:
        errors.append(f"Unexpected error: {str(e)}")
    
    is_valid = len(errors) == 0
    return is_valid, errors


if __name__ == "__main__":
    # Example Usage
    bt_string = """
    <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Fallback>
        <Sequence>
            <Action ID="LocateObject" object="food_item"/>
            <Action ID="NavigateTo" target="food_item"/>
            <Action ID="Pick" object="food_item"/>
            <Action ID="Place" object="food_item" location="table"/>
        </Sequence>
        <Sequence>
            <Action ID="NavigateTo" target="kitchen"/>
            <Action ID="LocateObject" object="fridge"/>
            <Action ID="OpenDoor" door="fridge"/>
            <Action ID="InspectObject" object="fridge_contents"/>
            <Action ID="IdentifyObject" object="low_sodium_food"/>
            <Fallback>
            <Sequence>
                <Action ID="Pick" object="low_sodium_food"/>
                <Action ID="Place" object="low_sodium_food" location="table"/>
            </Sequence>
            <Action ID="RequestAssistance" task="find_food"/>
            </Fallback>
        </Sequence>
        </Fallback>
    </BehaviorTree>
    </root>
    """

    # Validate BT from string
    is_valid, errors = validate_behavior_tree(bt_string)
    if is_valid:
        print("Behavior Tree is valid.")
    else:
        print("Behavior Tree is invalid. Errors:")
        for error in errors:
            print(f"- {error}")
