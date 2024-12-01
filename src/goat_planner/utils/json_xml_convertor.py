import json
import xml.etree.ElementTree as ET
from html import unescape
from xml.dom.minidom import parseString


def json_to_xml(json_obj, root_name="root"):
    def build_xml_element(element, data):
        if isinstance(data, dict):
            for key, value in data.items():
                sub_element = ET.SubElement(element, key)
                build_xml_element(sub_element, value)
        elif isinstance(data, list):
            for item in data:
                sub_element = ET.SubElement(element, "item")
                build_xml_element(sub_element, item)
        else:
            element.text = str(data)

    root = ET.Element(root_name)
    build_xml_element(root, json_obj)
    xml_string = ET.tostring(
        root, encoding="unicode"
    )  # Use unicode to keep raw characters
    pretty_xml = parseString(xml_string).toprettyxml(indent="  ")
    return pretty_xml


def xml_to_json(xml_string):
    def parse_element(element):
        child_elements = list(element)
        if not child_elements:
            return (
                unescape(element.text.strip()) if element.text else None
            )  # Decode entities
        result = {}
        for child in child_elements:
            if child.tag == "item":  # Handle lists
                result.setdefault(child.tag, [])
                result[child.tag].append(parse_element(child))
            else:
                result[child.tag] = parse_element(child)
        return result

    root = ET.fromstring(xml_string)
    return {root.tag: parse_element(root)}


# Test Example
if __name__ == "__main__":
    json_data = {
        "person": {
            "name": 'John "Doe"',
            "age": 30,
            "hobbies": ['reading "books"', "cycling"],
            "address": {"city": "New York", "zipcode": "10001"},
        }
    }

    print("JSON to XML:")
    xml_output = json_to_xml(json_data)
    print(xml_output)

    print("XML to JSON:")
    back_to_json = xml_to_json(xml_output)
    print(json.dumps(back_to_json, indent=2))
