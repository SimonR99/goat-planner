#!/usr/bin/env python3

from goat_planner.goat_state import GoatState
import json
from tabulate import tabulate


def view_objects():
    # Get singleton instance of GoatState
    state = GoatState()

    # Get all objects
    objects = state.get_objects()

    if not objects:
        print("No objects found in database.")
        return

    # Prepare data for tabulate
    table_data = []
    for obj_id, obj in objects.items():
        # Format position for display
        pos = obj.position
        position_str = f"x:{pos['x']:.2f}, y:{pos['y']:.2f}, z:{pos['z']:.2f}"

        # Get caption from properties
        caption = obj.properties.get("caption", "No caption")

        # Get confidence from properties
        confidence = obj.properties.get("confidence", "N/A")
        if isinstance(confidence, (int, float)):
            confidence = f"{confidence:.2f}"

        table_data.append(
            [obj_id, obj.type, position_str, caption, confidence, obj.last_updated]
        )

    # Print table
    headers = ["ID", "Type", "Position", "Caption", "Confidence", "Last Updated"]
    print("\nObjects in Database:")
    print(tabulate(table_data, headers=headers, tablefmt="grid"))


def view_state():
    # Get singleton instance of GoatState
    state = GoatState()

    print("\n=== Objects in Database ===")
    view_objects()

    print("\n=== Current Behavior Tree ===")
    tree = state.get_behavior_tree()
    if tree:
        print(json.dumps(tree, indent=2))
    else:
        print("No behavior tree found")


def main():
    try:
        view_state()
    except Exception as e:
        print(f"Error viewing objects: {e}")


if __name__ == "__main__":
    main()
