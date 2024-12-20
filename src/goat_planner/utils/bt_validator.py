import json
import xml.etree.ElementTree as ET
from typing import List, Literal, Optional


def validate_behavior_tree(
    bt_input,
    type: Literal["json", "xml"] = "json",
    allowed_nodes: Optional[List[str]] = None,
    from_file: bool = False,
):
    """
    Validate a Behavior Tree in JSON or XML format from a string or file.
    :param bt_input: A string containing the BT JSON/XML or the path to a JSON/XML file.
    :param type: The format of the input ("json" or "xml").
    :param allowed_nodes: A list of allowed node types. If None, all nodes are allowed.
    :return: (is_valid, errors) where is_valid is a boolean and errors is a list of validation issues.
    """
    errors = []

    try:
        if type == "xml":
            # Parse the input (string or file)
            if not from_file:
                tree = ET.ElementTree(ET.fromstring(bt_input))  # Parse from string
            else:
                tree = ET.parse(bt_input)  # Parse from file
            root = tree.getroot()

            # Check if the root element is `<root>`
            if root.tag != "root":
                errors.append("Root element must be <root>.")

            # Check if `main_tree_to_execute` attribute exists in `<root>`
            if "main_tree_to_execute" not in root.attrib:
                errors.append(
                    "The <root> element must have a 'main_tree_to_execute' attribute."
                )

            # Validate each <BehaviorTree> element
            for behavior_tree in root.findall("BehaviorTree"):
                # Check if ID attribute exists
                if "ID" not in behavior_tree.attrib:
                    errors.append(
                        "<BehaviorTree> elements must have an 'ID' attribute."
                    )

                # Validate the structure recursively
                def validate_node(node):
                    if allowed_nodes is None:
                        return
                    valid_nodes = set(allowed_nodes)

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

        elif type == "json":
            # Load the JSON input
            if not from_file:
                bt_data = json.loads(bt_input)  # Parse from string
            else:
                with open(bt_input, "r") as file:
                    bt_data = json.load(file)  # Parse from file

            # Validate the structure recursively
            def validate_node(node):
                if allowed_nodes is None:
                    return
                valid_nodes = set(allowed_nodes)

                if "type" not in node or node["type"] not in valid_nodes:
                    errors.append(
                        f"Invalid or missing node type '{node.get('type', None)}' found."
                    )
                # Recursively validate child nodes
                if "nodes" in node:
                    for child in node["nodes"]:
                        validate_node(child)

            validate_node(bt_data)

        else:
            errors.append("Unsupported format. Use 'json' or 'xml'.")

    except ET.ParseError as e:
        errors.append(f"XML Parsing Error: {str(e)}")
    except json.JSONDecodeError as e:
        errors.append(f"JSON Parsing Error: {str(e)}")
    except Exception as e:
        errors.append(f"Unexpected error: {str(e)}")

    is_valid = len(errors) == 0
    return is_valid, errors


if __name__ == "__main__":
    # Example Usage
    bt_string_xml = """
    <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Fallback>
        <Sequence>
            <Action ID="LocateObject" object="food_item"/>
        </Sequence>
        </Fallback>
    </BehaviorTree>
    </root>
    """

    bt_string_json = """
    {
        "type": "Sequence",
        "name": "RetrieveAndPlaceAppleSequence",
        "nodes": [
            {
                "type": "Sequence",
                "name": "PickUpAppleSequence",
                "nodes": [
                    {
                        "type": "Locate",
                        "object": "red apple sitting on the kitchen counter"
                    },
                    {
                        "type": "Pick",
                        "object": "red apple sitting on the kitchen counter",
                        "grip_strength": "medium",
                        "precision": "high"
                    }
                ]
            },
            {
                "type": "Sequence",
                "name": "PlaceAppleOnPlateSequence",
                "nodes": [
                    {
                        "type": "Navigate",
                        "x": 4.5,
                        "y": 2.2
                    },
                    {
                        "type": "Place",
                        "object": "red apple",
                        "surface": "ceramic plate with some bread crumbs",
                        "orientation": "upright",
                        "alignment": "center"
                    }
                ]
            }
        ]
    }
    """

    # Validate XML
    is_valid, errors = validate_behavior_tree(
        bt_string_xml, type="xml", allowed_nodes=["Sequence", "Fallback", "Action"]
    )
    if is_valid:
        print("XML Behavior Tree is valid.")
    else:
        print("XML Behavior Tree is invalid. Errors:")
        for error in errors:
            print(f"- {error}")

    # Validate JSON
    is_valid, errors = validate_behavior_tree(
        bt_string_json, type="json", allowed_nodes=["Sequence", "Navigate", "Place"]
    )
    if is_valid:
        print("JSON Behavior Tree is valid.")
    else:
        print("JSON Behavior Tree is invalid. Errors:")
        for error in errors:
            print(f"- {error}")
