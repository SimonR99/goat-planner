import xml.etree.ElementTree as ET
import json


def xml_to_json(xml_str):
    def parse_element(element):
        tag_data = {}
        # Get attributes
        if element.attrib:
            tag_data.update(element.attrib)
        # Recursively get children
        children = list(element)
        if children:
            child_data = {}
            for child in children:
                child_dict = parse_element(child)
                if child.tag not in child_data:
                    child_data[child.tag] = child_dict
                else:
                    if isinstance(child_data[child.tag], list):
                        child_data[child.tag].append(child_dict)
                    else:
                        child_data[child.tag] = [child_data[child.tag], child_dict]
            tag_data.update(child_data)
        else:
            # If no children, add the text content
            if element.text and element.text.strip():
                tag_data['text'] = element.text.strip()
        return tag_data

    root = ET.fromstring(xml_str)
    return json.dumps({root.tag: parse_element(root)}, indent=4)


def json_to_xml(json_str):
    def build_element(element_name, data):
        element = ET.Element(element_name)
        if isinstance(data, dict):
            for key, value in data.items():
                if key == "text":
                    element.text = value
                elif isinstance(value, list):
                    for item in value:
                        child = build_element(key, item)
                        element.append(child)
                elif isinstance(value, dict):
                    child = build_element(key, value)
                    element.append(child)
                else:
                    element.set(key, value)
        elif isinstance(data, list):
            for item in data:
                child = build_element(element_name, item)
                element.append(child)
        else:
            element.text = str(data)
        return element

    json_data = json.loads(json_str)
    root_name, root_content = next(iter(json_data.items()))
    root_element = build_element(root_name, root_content)
    return ET.tostring(root_element, encoding='unicode')


# Example Usage
xml_input = """
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="0" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="0.333">
          <RecoveryNode number_of_retries="0" name="ComputePathThroughPoses">
            <ReactiveSequence>
              <RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="1.5"/>
              <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/>
            </ReactiveSequence>
            <ReactiveFallback name="ComputePathThroughPosesRecoveryFallback">
              <GoalUpdated/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </ReactiveFallback>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="0" name="FollowPath">
          <FollowPath path="{path}" controller_id="FollowPath"/>
          <ReactiveFallback name="FollowPathRecoveryFallback">
            <GoalUpdated/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
          </ReactiveFallback>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <!-- <Spin spin_dist="1.57"/> -->
          <Wait wait_duration="5"/>
          <BackUp backup_dist="0.15" backup_speed="0.025"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
"""

# Convert XML to JSON
json_output = xml_to_json(xml_input)
print("JSON Output:")
print(json_output)

# Convert JSON back to XML
xml_output = json_to_xml(json_output)
print("\nXML Output:")
print(xml_output)
