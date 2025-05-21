import cv2
import os
from glob import glob

# Path to your directory containing images
image_dir = "/home/makemorerobot/images"

# Load all image paths
image_paths = sorted(glob(os.path.join(image_dir, "*.*")))  # Use specific extensions if needed

# Read images
images = []
for path in image_paths:
    img = cv2.imread(path)
    if img is not None:
        images.append(img)

# Make sure we have enough images
if len(images) < 2:
    print("Need at least two images to stitch.")
    exit()

# Create a Stitcher object (OpenCV 4.x)
stitcher = cv2.Stitcher_create()
print("Stitching images...")
# Stitch the images
status, stitched = stitcher.stitch(images)

if status == cv2.Stitcher_OK:
    print("Stitching completed successfully.")
    cv2.imwrite("stitched_output.jpg", stitched)
else:
    print(f"Stitching failed with status code {status}")
