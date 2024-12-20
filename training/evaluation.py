import json
import re
from typing import Literal

from tqdm import tqdm

from goat_planner.utils.bt_validator import validate_behavior_tree
from goat_planner.utils.json_xml_convertor import json_to_xml


def evaluate_model(
    model,
    tokenizer,
    formatting_prompt,
    validation_type: Literal["json", "xml"] = "json",
    query_file="./query_dataset.json",
    instruction="",
    action_list=None,
):
    """
    Process queries and validate behavior trees.

    Args:
        model: The model used for generating answers.
        tokenizer: The tokenizer used for preprocessing.
        formatting_prompt: A function to format input prompts.
        validation_type (str): Type of validation ('json' or 'xml'). Default is 'json'.
        query_file (str): Path to the JSON file containing the queries.
        action_list (list): List of action nodes with 'name' fields.

    Returns:
        dict: A dictionary containing the score and categorized plans.
    """
    number_of_valid_json = 0
    categorized_plans = {"no plan": [], "not valid": [], "valid": []}

    # Load query dataset
    with open(query_file, "r") as f:
        query_json = json.load(f)

    query_list = [query["task"] for query in query_json]

    base_node = ["Sequence", "Fallback", "Retry"]
    action_node = [action["name"] for action in action_list] if action_list else []

    for i, query in enumerate(tqdm(query_list)):
        examples = {"instruction": [instruction], "input": [query], "output": [""]}

        formatted_data = formatting_prompt(examples)
        inputs = tokenizer(
            [formatted_data["text"][0]],
            return_tensors="pt",
            padding=True,
            truncation=True,
        ).to("cuda")

        outputs = model.generate(
            **inputs,
            max_new_tokens=700,
            use_cache=True,
            pad_token_id=tokenizer.pad_token_id,
        )

        answer = tokenizer.batch_decode(outputs)
        answer = answer[0].split("<|start_header_id|>assistant<|end_header_id|>")[-1]

        plan = re.findall(r"<plan>(.*?)</plan>", answer, re.DOTALL)

        if not plan:
            categorized_plans["no plan"].append([query, answer])
            continue

        plan = plan[0]
        is_xml_valid = validate_behavior_tree(
            plan, validation_type, allowed_nodes=base_node + action_node
        )

        if is_xml_valid[0]:
            categorized_plans["valid"].append(plan)
            number_of_valid_json += 1
        else:
            categorized_plans["not valid"].append([plan, is_xml_valid[1]])

    score = number_of_valid_json / len(query_list) if query_list else 0

    return {"score": score, "plans": categorized_plans}
