from PIL import Image
import os
from pluspy import ffmpeg_utils
import re

def generate_video(srcdir, ext='.png', prefix='collage_outputframe'):
    # Generate video...
    PICT_EXT = ext
    FRAME_RATE = 10
    VID_FILE = srcdir+prefix+'vid.mp4'
    vid_output = os.path.join(srcdir, VID_FILE)
    short_pict = PICT_EXT.strip(".")
    print("Building video file %s from %s files in %s" % (vid_output, PICT_EXT, srcdir))
    am_files, _ = ffmpeg_utils.generate_video_from_files(
        ffmpeg_utils.file_lister(srcdir, prefix, PICT_EXT), FRAME_RATE, vid_output,
        threads=1, gpu_accel=True)
    if am_files == 0:
        raise RuntimeError("Couldn't find any %s files to generate video." % short_pict)

parent_dir = "/home/yuchen.wang/Documents/_out"  # this folder contains 9 subfolders

def collage():
    for i in range(102, 251):
        skip = False
        # --- CONFIG ---
        id = 'frame030{}'.format(i)
        output_path = "/home/yuchen.wang/Documents/_out/collage3/collage_output{}.jpg".format(id)
        rows, cols = 1, 3
        single_width, single_height = 3840, 2160
        final_size = (3840, 720)

        # --- Get 1 image per subfolder ---
        # ordered_name = ["front_left_camera", "front", "front_right_camera", "left_rear_camera", "left_side_camera", "left_front_camera", "right_front_camera", "right_side_camera", "right_rear_camera"]
        ordered_name = ["front_left_camera", "front", "front_right_camera"]
        subfolders = [os.path.join(parent_dir, d) for d in ordered_name if os.path.isdir(os.path.join(parent_dir, d))]
        # if len(subfolders) < 9:
        #     raise ValueError("Need at least 9 subfolders, each containing one JPG image.")

        image_paths = []
        for folder in subfolders[:9]:
            jpg = id+".jpg"
            if not os.path.exists(os.path.join(folder, jpg)):
                skip = True
                break
            image_paths.append(os.path.join(folder, jpg))  # pick the first JPG

        if skip:
            continue

        # --- Load and resize images ---
        images = [Image.open(p).resize((single_width, single_height)) for p in image_paths]

        # --- Create large canvas ---
        # collage = Image.new("RGB", (cols * single_width, 1 * single_height + 2 * single_width))
        collage = Image.new("RGB", (cols * single_width, rows * single_height))

        margin = int((single_width - single_height) * 3 / 2)

        for idx, img in enumerate(images):
            row = idx // cols
            col = idx % cols
            if row == 0:
                collage.paste(img, (col * single_width, row * single_height))
            # else:
            #     paste_img = img.rotate(90, expand=True)
            #     x = col * single_height + margin
            #     y = single_height + (row-1) * single_width
            #     collage.paste(paste_img, (x, y))

        # --- Resize to final output ---
        collage = collage.resize(final_size, Image.LANCZOS)

        # --- Save result ---
        collage.save(output_path)
        print(f"Saved collage to {output_path}")

# collage()
# generate_video("/home/yuchen.wang/Documents/_out/right_rear_camera/", '.jpg', 'frame')
# generate_video("/home/yuchen.wang/Documents/_out/right_rear_camera/", '.jpg', 'seg')
# generate_video("/home/yuchen.wang/Documents/_out/right_rear_camera/", '.jpg', 'dep')

def collage_stitch():

    # Set your folder paths
    folder_a = "/home/yuchen.wang/Documents/_out/front_left_camera/"  # contains 1.jpg to 150.jpg
    folder_b = "/home/yuchen.wang/Documents/_out/STITCH_CAMERA_DONUT_BOWL/"  # contains rostime-named .jpgs

    # Sort Folder A numerically by frame number
    def extract_frame_number(filename):
        match = re.search(r"frame(\d+)", filename)
        return int(match.group(1)) if match else float('inf')

    images_a = sorted(
        [f for f in os.listdir(folder_a) if f.endswith(".jpg")],
        key=extract_frame_number
    )

    # Get and sort B: sort by float timestamp
    images_b = sorted(
        [f for f in os.listdir(folder_b) if f.endswith(".jpg")],
        key=lambda x: float(os.path.splitext(x)[0])
    )

    # Sanity check
    if len(images_a) != len(images_b):
        raise ValueError(f"Image count mismatch: {len(images_a)} vs {len(images_b)}")

    # Pairing
    paired = list(zip(images_a, images_b))

    # Print or save the pairs
    for a, b in paired:
        a_dir = os.path.join(folder_a, a)
        imgA = Image.open(a_dir).resize((1440, 810))
        b_dir = os.path.join(folder_b, b)
        imgB = Image.open(b_dir)
        # print(imgs)
        # print(f"{a} <--> {b}")
        collage = Image.new("RGB", (1440, 1170))
        collage.paste(imgA, (0, 0))
        collage.paste(imgB, (0, 810))

        output_path = "/home/yuchen.wang/Documents/_out/collage_output{}".format(a)

        # --- Save result ---
        collage.save(output_path)
        print(f"Saved collage to {output_path}")

collage_stitch()
generate_video("/home/yuchen.wang/Documents/_out/", '.jpg', 'collage_outputframe')
