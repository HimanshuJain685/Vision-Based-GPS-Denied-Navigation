import rasterio
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import rotate
import random
from math import radians, cos, sin
import cv2
import os

#-----------------------------------------------------------------------------------#
def reverse_rotate(center_x, center_y, angle, ref_center_x, ref_center_y):
    # Convert angle to radians
    angle_rad = radians(angle)
    
    # Compute the center relative to the reference image center
    dx = center_x - ref_center_x
    dy = center_y - ref_center_y
    
    # Reverse rotate the center
    new_dx = cos(-angle_rad) * dx - sin(-angle_rad) * dy
    new_dy = sin(-angle_rad) * dx + cos(-angle_rad) * dy
    
    # Calculate the new position in the original reference image coordinates
    new_center_x = ref_center_x + new_dx
    new_center_y = ref_center_y + new_dy
    
    return round(new_center_x, 2), round(new_center_y, 2)


def generate_image_pairs(tif_path1, tif_path2, location, ref_dir, cam_dir, n, ref_size):
    with rasterio.open(tif_path1) as src1, rasterio.open(tif_path2) as src2:
        image1_rgb = src1.read()  # Shape: (3, H, W)
        image2_rgb = src2.read()  # Shape: (3, H, W)
        height, width = image1_rgb.shape[1:]

    os.makedirs(ref_dir, exist_ok=True)
    os.makedirs(cam_dir, exist_ok=True)

    for i in range(n):
        # Randomly crop from both images using same coordinates
        ref_x = random.randint(0, width - ref_size)
        ref_y = random.randint(0, height - ref_size)
        
        reference_rgb = image1_rgb[:, ref_y:ref_y+ref_size, ref_x:ref_x+ref_size]
        reference_rgb2 = image2_rgb[:, ref_y:ref_y+ref_size, ref_x:ref_x+ref_size]

        map_center_x = ref_x + ref_size // 2
        map_center_y = ref_y + ref_size // 2

        # Rotate the crop from the second image
        angle = random.uniform(0, 360)
        rotated_rgb = np.stack([
            rotate(reference_rgb2[c], angle, reshape=False, order=1)
            for c in range(3)
        ], axis=0)

        #Define camera image size
        cam_width = int(ref_size / 3)
        cam_height = int(cam_width * 2 / 3)

        # Select random patch from rotated image with valid corners
        while True:
            cam_x = random.randint(0, ref_size - cam_width)
            cam_y = random.randint(0, ref_size - cam_height)

            corners = [
                (cam_y, cam_x),
                (cam_y, cam_x + cam_width - 1),
                (cam_y + cam_height - 1, cam_x),
                (cam_y + cam_height - 1, cam_x + cam_width - 1)
            ]

            valid = True
            for y, x in corners:
                pixel = rotated_rgb[:, y, x]
                if np.all(pixel == 0):
                    valid = False
                    break

            if valid:
                break

        camera_rgb = rotated_rgb[:, cam_y:cam_y+cam_height, cam_x:cam_x+cam_width]

        # Calculate center of camera image in reference image coords
        center_x = cam_x + cam_width // 2
        center_y = cam_y + cam_height // 2

        ref_center_x = ref_size // 2
        ref_center_y = ref_size // 2

        # Reverse rotate the center
        actual_center_x, actual_center_y = reverse_rotate(center_x, center_y, -angle, ref_center_x, ref_center_y)

        # Generate filename
        filename = f"({map_center_y},{map_center_x})_({round(actual_center_y - ref_size//2, 2)},{round(actual_center_x - ref_size//2, 2)})_{angle:.2f}_{location}.png"
        print(filename)

        # Save Images
        ref_img_path = os.path.join(ref_dir, filename)
        cam_img_path = os.path.join(cam_dir, filename)

        # Convert from (C, H, W) to (H, W, C) and clip to 0–255 uint8
        ref_img_bgr = np.transpose(reference_rgb, (1, 2, 0)).astype(np.uint8)
        cam_img_bgr = np.transpose(camera_rgb, (1, 2, 0)).astype(np.uint8)

        # Save using OpenCV if needed
        # cv2.imwrite(ref_img_path, cv2.cvtColor(ref_img_bgr, cv2.COLOR_RGB2BGR))
        # cv2.imwrite(cam_img_path, cv2.cvtColor(cam_img_bgr, cv2.COLOR_RGB2BGR))

        # DISPLAY PLOTS
        fig, axs = plt.subplots(1, 3, figsize=(15, 5))

        axs[0].imshow(ref_img_bgr)
        axs[0].plot(actual_center_x, actual_center_y, 'ro')
        axs[0].set_title("Reference Image (from image1)\n(Red dot = Actual Center after Reverse Rotation)")
        axs[0].axis('off')

        axs[1].imshow(np.transpose(rotated_rgb, (1, 2, 0)).astype(np.uint8))
        axs[1].plot(center_x, center_y, 'ro')
        axs[1].set_title(f"Rotated Image (from image2)\n(Rotation: {angle:.2f}°)")
        axs[1].axis('off')

        axs[2].imshow(cam_img_bgr)
        axs[2].set_title("Camera Image (from rotated image2)")
        axs[2].axis('off')

        plt.tight_layout()
        plt.show()


#-----------------------------------------------------------------------------------#
# Output directories
ref_dir = "dataset/Reference Image"
cam_dir = "dataset/Camera Image"

generate_image_pairs("US Google.tif", "US Bing.tif", "us", ref_dir, cam_dir, n=500, ref_size=1000)

