import cv2
import os
import json
from ultralytics import YOLO as UltralyticsYOLO
from transformers import pipeline
from PIL import Image
from typing import List, Dict
import numpy as np


def load_yolo_model(model_path: str, device: str = "cpu"):
    """Load YOLO model."""
    model = UltralyticsYOLO(model_path)
    model.to(device)
    return model


def detect_objects(model, image: np.ndarray, confidence_threshold: float = 0.25, nms_threshold: float = 0.45) -> List[Dict]:
    """Run YOLO detection on an image."""
    results = model(image, conf=confidence_threshold, iou=nms_threshold)
    return postprocess_yolo_results(results)


def postprocess_yolo_results(results) -> List[Dict]:
    """Convert YOLO output to a list of detections."""
    detections = []
    for result in results:
        boxes = result.boxes
        for box in boxes:
            detection = {
                'bbox': box.xyxy[0].cpu().numpy().astype(int).tolist(),  # Convert to list for JSON compatibility
            }
            detections.append(detection)
    return detections


def process_image_dataset(
    dataset_path: str,
    output_path: str,
    yolo_model,
    blip_pipeline,
    min_width: int = 40,
    min_height: int = 40,
    max_captions: int = 1000,
):
    """Process a folder of images and generate unique captions."""
    os.makedirs(output_path, exist_ok=True)

    unique_captions = set()
    total_captions = 0

    for image_file in os.listdir(dataset_path):
        if total_captions >= max_captions:
            print(f"Reached maximum caption limit of {max_captions}. Stopping processing.")
            break

        image_path = os.path.join(dataset_path, image_file)
        if not image_file.lower().endswith(('.png', '.jpg', '.jpeg')):
            continue  # Skip non-image files

        # Load image
        image = cv2.imread(image_path)
        if image is None:
            print(f"Failed to load image: {image_file}")
            continue

        # Detect objects with YOLO
        detections = detect_objects(yolo_model, image)

        for i, det in enumerate(detections):
            if total_captions >= max_captions:
                break

            x1, y1, x2, y2 = det['bbox']
            padding = 0.2  # 20% padding
            x1 = max(0, x1 - int(padding * (x2 - x1)))
            x2 = min(image.shape[1], x2 + int(padding * (x2 - x1)))
            y1 = max(0, y1 - int(padding * (y2 - y1)))
            y2 = min(image.shape[0], y2 + int(padding * (y2 - y1)))

            cropped_width = x2 - x1
            cropped_height = y2 - y1

            if cropped_width < min_width or cropped_height < min_height:
                print(f"Skipping small region: {cropped_width}x{cropped_height}")
                continue

            # Crop and prepare for captioning
            cropped_image = image[y1:y2, x1:x2]
            cropped_image_rgb = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(cropped_image_rgb)

            # Generate caption
            caption = blip_pipeline(pil_image)[0]['generated_text']

            if caption in unique_captions:
                print(f"Skipping duplicate caption: {caption}")
                continue

            unique_captions.add(caption)
            total_captions += 1
            print(f"Caption added: {caption}")

    # Save all unique captions to a JSON file
    captions_file_path = os.path.join(output_path, "captions.json")
    with open(captions_file_path, "w") as json_file:
        json.dump(list(unique_captions), json_file, indent=4)

    print(f"Captions saved to {captions_file_path}")


# Usage Example
if __name__ == "__main__":
    # Path to YOLO model and dataset
    yolo_model_path = "../models/yolov8s-world.pt"  # Replace with your YOLO model path
    dataset_path = "../data/coco_dataset/val2017"  # Replace with your dataset folder path
    output_path = "../data"  # Replace with your output folder path

    # Load YOLO model
    yolo_model = load_yolo_model(yolo_model_path, device="cuda")

    # Initialize BLIP Captioning Pipeline
    blip_pipeline = pipeline("image-to-text", model="Salesforce/blip-image-captioning-base", device=0)

    # Process dataset
    process_image_dataset(dataset_path, output_path, yolo_model, blip_pipeline)
