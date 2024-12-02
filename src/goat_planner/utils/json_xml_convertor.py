import json
import xml.etree.ElementTree as ET
from html import unescape
from xml.dom.minidom import parseString


def xml_to_json(xml_string):
    def parse_element(element):
        node = {"type": element.tag}
        if element.attrib:
            node.update(element.attrib)
        children = list(element)
        if children:
            node["nodes"] = [parse_element(child) for child in children]
        elif element.text and element.text.strip():
            node["parameters"] = {"text": element.text.strip()}
        return node

    root = ET.fromstring(xml_string)
    return {root.tag: parse_element(root)}


def json_to_xml(json_obj):
    def build_element(node):
        element = ET.Element(node["type"])
        for key, value in node.items():
            if key == "nodes":
                for child in value:
                    element.append(build_element(child))
            elif key != "type":
                element.set(key, value)
        return element

    root_key = next(iter(json_obj))
    root_data = json_obj[root_key]
    root_element = build_element(root_data)

    # Pretty-print the XML
    rough_string = ET.tostring(root_element, "utf-8")
    return parseString(rough_string).toprettyxml(indent="    ")


# Test Example
if __name__ == "__main__":
    xml_string = """
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <SaySomething name="action_hello" message="Hello"/>
            <OpenGripper name="open_gripper"/>
            <ApproachObject name="approach_object"/>
            <CloseGripper name="close_gripper"/>
        </Sequence>
    </BehaviorTree>
</root>
"""

    print("XML to JSON:")
    json_data = xml_to_json(xml_string)
    print(json.dumps(json_data, indent=2))

    print("JSON to XML:")
    xml_output = json_to_xml(json_data)
    print(xml_output)

    xml_output = xml_output.replace("\n", "").replace('<?xml version="1.0" ?>', "")
    xml_string = xml_string.replace("\n", "")
    print(xml_string == xml_output)
