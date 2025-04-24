import rasterio
import numpy as np
import cv2
#-----------------------------------------------------------------------------------#
def geotiff_to_png_opencv(geotiff_path, output_png_path):
    # Open the GeoTIFF file
    with rasterio.open(geotiff_path) as src:
        # Read the first 3 bands as R, G, B
        r = src.read(1)
        g = src.read(2)
        b = src.read(3)

        # Stack into RGB
        rgb = np.stack([r, g, b], axis=-1)

        # Normalize to 0–255 if needed
        if rgb.dtype != np.uint8:
            rgb_min = rgb.min()
            rgb_max = rgb.max()
            rgb = 255 * (rgb - rgb_min) / (rgb_max - rgb_min)
            rgb = rgb.astype(np.uint8)

        # Convert RGB to BGR for OpenCV
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # Save using OpenCV
        cv2.imwrite(output_png_path, bgr)
        print(f"Saved PNG to {output_png_path}")

#-----------------------------------------------------------------------------------#
# Example usage
geotiff_to_png_opencv('Mumbai 7500.tif', 'Mumbai 7500 image.png')
