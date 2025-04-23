import rasterio
from rasterio.transform import xy
from math import radians, sin, cos, sqrt, atan2

def haversine(coord1, coord2):
    """Calculate the Haversine distance between two (lat, lon) points."""
    R = 6371000  # Earth radius in meters

    lat1, lon1 = radians(coord1[0]), radians(coord1[1])
    lat2, lon2 = radians(coord2[0]), radians(coord2[1])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    return R * c

# === CONFIGURATION ===
tiff_file = "Mumbai 7500.tif"

# === PROCESS TIFF ===
with rasterio.open(tiff_file) as dataset:
    image = dataset.read()  # shape: (bands, height, width)
    _, height, width = image.shape

    print(f"Image dimensions Width: {width} pixels, Height: {height} pixels")

    # Get GPS coordinates of key corners using (row, col)
    top_left_latlon = xy(dataset.transform, 0, 0)
    bottom_left_latlon = xy(dataset.transform, height - 1, 0)
    bottom_right_latlon = xy(dataset.transform, height - 1, width - 1)
    center_latlon = xy(dataset.transform, height//2, width//2)

    # xy() returns (lon, lat) — rearranged to (lat, lon)
    top_left = (top_left_latlon[1], top_left_latlon[0])
    bottom_left = (bottom_left_latlon[1], bottom_left_latlon[0])
    bottom_right = (bottom_right_latlon[1], bottom_right_latlon[0])
    center = (center_latlon[1], center_latlon[0])

    # print(f"Top Left GPS: {top_left}")
    # print(f"Bottom Left GPS: {bottom_left}")
    # print(f"Bottom Right GPS: {bottom_right}")
    print(f"Center GPS: {center}")

    # Calculate distances using Haversine formula
    height_m = haversine(top_left, bottom_left)
    width_m = haversine(bottom_left, bottom_right)

    print(f"Image Height (meters): {height_m:.2f}")
    print(f"Image Width (meters): {width_m:.2f}")
    print(width_m/width)
    print(height_m/height)
