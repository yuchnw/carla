import zipfile
import os
import shutil
import tempfile

usdz_path = "HIGHWAY/Batch0012/eabf93a3-62e8-4302-b989-354e7422a644/eabf93a3-62e8-4302-b989-354e7422a644.usdz"
output_dir = "extracted_json/eabf"

os.makedirs(output_dir, exist_ok=True)

with zipfile.ZipFile(usdz_path, 'r') as zip_ref:
    # Create a temp directory to extract
    with tempfile.TemporaryDirectory() as tmpdir:
        zip_ref.extractall(tmpdir)

        for root, _, files in os.walk(tmpdir):
            for file in files:
                if file.lower().endswith(".ply") or file.lower().endswith(".json") or file.lower().endswith(".xodr"):
                # print(file)
                # if "mesh_ground.usd" in file:
                #     #src = os.path.join(root, file)
                #     #dst = os.path.join(output_dir, file)
                    # shutil.copy2(root+"/"+file, '/home/yuchen.wang/Downloads/')
                    shutil.copy2(root+"/"+file, output_dir)
