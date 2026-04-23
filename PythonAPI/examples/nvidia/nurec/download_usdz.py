import argparse
from pathlib import Path
import json
import os

from huggingface_hub import login, snapshot_download


def string_to_boolean(s):
    s = s.strip().lower()  # Normalize the string
    if s in ('true', '1', 'yes', 'on'):
        return True

    return False

def main():
    valid_categories = ["behavior", "layout", "lighting", "road_types", "surface_conditions", "traffic_density", "vrus", "weather"]

    parser = argparse.ArgumentParser(
        description="Downloads usdz clips based upon criteria specified in the labels.json"
    )
    parser.add_argument(
        "--local-dir", type=str, required=True, help="The path to store the usdz"
    )
    parser.add_argument(
        "--category",
        type=str,
        required=True,
        choices=valid_categories,
        help="The specified category in the labels.json. Must be one of: %(choices)",
    )
    parser.add_argument(
        "--value",
        type=str,
        required=True,
        help="The specified value in the category",
    )

    args = parser.parse_args()

    hf_api_token = os.getenv("HF_TOKEN")

    login(token=hf_api_token)

    # First download all the labels.json files
    print(f"Downloading dataset labels.json to {args.local_dir}.")

    snapshot_download(repo_id="nvidia/PhysicalAI-Autonomous-Vehicles-NuRec", repo_type="dataset", allow_patterns="*.json", local_dir=args.local_dir)

    category = args.category
    value = args.value

    # Find all of the labels.json files that have been downloaded
    local_dir = Path(args.local_dir)
    label_paths = local_dir.rglob("labels.json")

    # Filter through the labels.json and find all usdz that match our criteria
    paths_to_download = {}

    print(f"Filtering usdz downloads based upon labels.json downloaded with criteria {category} and {value}.")

    for label_path in label_paths:
        with open(label_path, "r", encoding="utf-8") as f:
            metadata = json.load(f)

            if category in metadata:
                if category == "vrus":
                    if string_to_boolean(value) == metadata["vrus"]:
                        paths_to_download[label_path.parent] = True
                else:
                    if value in metadata[args.category]:
                        paths_to_download[label_path.parent] = True

    print(f"Found {len(paths_to_download)} that matched criteria.")

    # Download the selected usdz and front camera mp4
    for path in paths_to_download.keys():
        relative_path = path.relative_to(local_dir)
        if "Batch0012/f" in str(path) or "Batch0012/e" in str(path) or "Batch0012/d" in str(path) or "Batch0012/c" in str(path) or "Batch0012/b" in str(path):
            continue  # skip Batch0012 which has some corrupted usdz files

        print(f"Downloading usdz and front camera at path {relative_path}")
        snapshot_download(repo_id="nvidia/PhysicalAI-Autonomous-Vehicles-NuRec", repo_type="dataset", allow_patterns=f"{relative_path}/*", local_dir=args.local_dir)


if __name__ == "__main__":
    main()