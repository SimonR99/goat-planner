import os
import requests
import zipfile
from tqdm import tqdm


def download_and_extract_coco(dataset_path="coco_dataset"):
    """
    Download and extract the COCO 2017 dataset.
    """
    # Ensure dataset directory exists
    os.makedirs(dataset_path, exist_ok=True)

    # Define URLs for images and annotations
    coco_urls = {
        "val_images": "http://images.cocodataset.org/zips/val2017.zip",
    }

    for key, url in coco_urls.items():
        # Define the output zip file path
        zip_path = os.path.join(dataset_path, f"{key}.zip")
        extracted_path = os.path.join(dataset_path, key)

        # Skip if already downloaded
        if os.path.exists(extracted_path):
            print(f"{key} already downloaded and extracted.")
            continue

        # Download the file with progress bar
        print(f"Downloading {key}...")
        response = requests.get(url, stream=True)
        total_size = int(response.headers.get("content-length", 0))
        with open(zip_path, "wb") as f, tqdm(
            desc=key, total=total_size, unit="B", unit_scale=True
        ) as bar:
            for chunk in response.iter_content(chunk_size=1024):
                f.write(chunk)
                bar.update(len(chunk))

        # Extract the zip file
        print(f"Extracting {key}...")
        with zipfile.ZipFile(zip_path, "r") as zip_ref:
            zip_ref.extractall(dataset_path)
        os.remove(zip_path)  # Remove the zip file to save space

    print("COCO dataset downloaded and extracted.")


# Usage Example
if __name__ == "__main__":
    # Path to save the COCO dataset
    coco_dataset_path = "../data/coco_dataset"

    # Download and prepare COCO dataset
    download_and_extract_coco(coco_dataset_path)

    # Specify dataset folders for processing
    train_images_path = os.path.join(coco_dataset_path, "train_images")
    val_images_path = os.path.join(coco_dataset_path, "val_images")
    print(f"Train images directory: {train_images_path}")
    print(f"Validation images directory: {val_images_path}")
