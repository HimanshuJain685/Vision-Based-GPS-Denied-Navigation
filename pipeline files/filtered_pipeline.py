import rospy
import socket
import json
import rasterio
import cv2
import numpy as np
from math import radians, sin, cos, sqrt, atan2
import time
from rasterio.transform import rowcol, xy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil

#------------------------------------------------------------------------------------------------------#
# Configuration Variables
TIFF_PATH = "Mumbai 7500.tif"
INITIAL_GPS = (18.992394752836383, 72.83529905339414)
REFERENCE_SIZE_DEFAULT = 500
REFERENCE_SIZE_FIRST = 1000
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
out_addr = ("127.0.0.1", 25100)
waypoint_sent = True

connection_string = 'udp:127.0.0.1:14550'
print("Connecting to vehicle on:", connection_string)
vehicle = connect(connection_string, wait_ready=True)

#------------------------------------------------------------------------------------------------------#
# Function to get distance between two GPS coordinates 
def haversine(coord1, coord2):
    R = 6371000
    lat1, lon1 = radians(coord1[0]), radians(coord1[1])
    lat2, lon2 = radians(coord2[0]), radians(coord2[1])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

# Fucntion to set waypoint in the vehicle
def set_waypoint(lat, lon):
    cmds = vehicle.commands
    cmds.clear()
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, 0, 0,
        lat, lon, 40
    ))
    cmds.upload()
    print("Waypoint uploaded to:", lat, lon)

# Function to send GPS data over UDP
def send_gps_udp(lat, lon, angle):
    global waypoint_sent
    gps_msg = {
        'time_usec': 0,
        'gps_id': 0,
        'ignore_flags': 254,      # ignore alt, hdop, vdop, vn, ve, vd, speed_accuracy, vert_accuracy
        'time_week_ms': 0,
        'time_week': 0,
        'fix_type': 3,
        'lat': int(lat * 1E7),
        'lon': int(lon * 1E7),
        'alt': 0,
        'hdop': 1,
        'vdop': 1,
        'vn': 0,
        've': 0,
        'vd': 0,
        'speed_accuracy': 0,
        'horiz_accuracy': 1,
        'vert_accuracy': 0,
        'satellites_visible': 13,
        'yaw': angle
    }
    out_data = json.dumps(gps_msg)
    s.sendto(out_data.encode(), out_addr)

    # Send initial GPS coordinates as waypoint
    # to ensure the vehicle remains at that position 
    if not waypoint_sent:
        set_waypoint(INITIAL_GPS[0], INITIAL_GPS[1])
        waypoint_sent = True

# Function to arm the vehicle and take off to a specified altitude
# It keeps sending the initial GPS coordinates to the vehicle until takeoff
def arm_and_takeoff(target_altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise... ")
        send_gps_udp(*INITIAL_GPS, 0)
        time.sleep(0.05)

    start_time = time.time()
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming... ")
        if time.time() - start_time >= 0.05:
            send_gps_udp(*INITIAL_GPS, 0)
            start_time = time.time()
        time.sleep(0.05)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        if time.time() - start_time >= 0.05:
            send_gps_udp(*INITIAL_GPS, 0)
            start_time = time.time()
        altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude:", altitude)
        if altitude >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(0.05)

    global waypoint_sent

    # Set waypoint_sent to False
    # to ensure the vehicle receives the initial GPS coordinates 
    # when the next iteration begins
    waypoint_sent = False

# Class for low-pass filter
# to smooth out the GPS data
class LowPassFilter:
    def __init__(self, alpha, initial_value):
        self.alpha = alpha
        self.filtered = initial_value

    def update(self, new_value):
        self.filtered = self.alpha * new_value + (1 - self.alpha) * self.filtered
        return self.filtered

# Camera class to handle image capture
class Camera:
    def __init__(self, topic="/webcam/image_raw"):
        self.bridge = CvBridge()
        self.image = None

        # Mutex lock to prevent conflict between Image saving and retrieval
        self.lock = threading.Lock()
        rospy.Subscriber(topic, Image, self.callback)
        rospy.loginfo(f"Subscribed to camera topic: {topic}")

    def callback(self, msg):
        try:
            # Save grayscale image
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            with self.lock:
                self.image = img.copy()
        except Exception as e:
            rospy.logwarn(f"Image conversion failed: {e}")

    def get_image(self):
        with self.lock:
            return self.image.copy() if self.image is not None else None

#------------------------------------------------------------------------------------------------------#
# Main function to takeoff and process images and send GPS data
def main():
    rospy.init_node('gps_image_matcher')

    # Load GeoTIFF raster image
    with rasterio.open(TIFF_PATH) as dataset:
        print("GeoTIFF loaded.")
        # arm_and_takeoff(40)

        sift = cv2.SIFT_create()
        camera = Camera()
        current_gps = INITIAL_GPS

        image = dataset.read()
        _, height, width = image.shape

        # Get GPS coordinates of key corners using (row, col)
        top_left_latlon = xy(dataset.transform, 0, 0)
        bottom_left_latlon = xy(dataset.transform, height - 1, 0)
        bottom_right_latlon = xy(dataset.transform, height - 1, width - 1)

        # Calculate distances using Haversine formula
        height_m = haversine((top_left_latlon[1], top_left_latlon[0]), (bottom_left_latlon[1], bottom_left_latlon[0]))
        width_m = haversine((bottom_left_latlon[1], bottom_left_latlon[0]), (bottom_right_latlon[1], bottom_right_latlon[0]))

        # Calculate average meters per pixel accuracy
        h_acc = 0.5 * ((width_m / width) + (height_m / height))

        # Initialize Filters
        lat_filter = LowPassFilter(alpha=0.2, initial_value=INITIAL_GPS[0])
        lon_filter = LowPassFilter(alpha=0.2, initial_value=INITIAL_GPS[1])
        yaw_filter = LowPassFilter(alpha=0.2, initial_value=0.0)

        count = 0
        iteration = 0
        global waypoint_sent

        while not rospy.is_shutdown():
            if iteration == 1:
                waypoint_sent = False

            if count < 5:
                start_time = time.time()

                # Get the current camera image
                cam_gray = camera.get_image()
                if cam_gray is None:
                    rospy.logwarn("Waiting for camera image...")
                    rospy.sleep(0.1)
                    continue

                ref_row, ref_col = rowcol(dataset.transform, current_gps[1], current_gps[0])

                # Set larger window size for 1st iteration 
                # to cater for drift
                ref_size = REFERENCE_SIZE_FIRST if iteration == 0 else REFERENCE_SIZE_DEFAULT

                # Define centered window
                row_start = max(0, ref_row - ref_size // 2)
                row_end = min(dataset.height, ref_row + ref_size // 2)
                col_start = max(0, ref_col - ref_size // 2)
                col_end = min(dataset.width, ref_col + ref_size // 2)

                # Read RGB bands from the raster image
                r = dataset.read(1, window=((row_start, row_end), (col_start, col_end)))
                g = dataset.read(2, window=((row_start, row_end), (col_start, col_end)))
                b = dataset.read(3, window=((row_start, row_end), (col_start, col_end)))

                # Stack and convert to BGR 
                ref_image = np.dstack((b, g, r))
                ref_center = ((row_start + row_end) // 2, (col_start + col_end) // 2)

                ref_gray = cv2.cvtColor(ref_image, cv2.COLOR_BGR2GRAY)

                # Find keypoints and descriptors
                kp_ref, desc_ref = sift.detectAndCompute(ref_gray, None)
                kp_cam, desc_cam = sift.detectAndCompute(cam_gray, None)

                if desc_ref is None or desc_cam is None:
                    print("Insufficient Descriptors.")
                    count += 1
                    rospy.sleep(0.1)
                    continue

                # FLANN matcher    
                index_params = dict(algorithm=1, trees=5)
                search_params = dict(checks=50)
                flann = cv2.FlannBasedMatcher(index_params, search_params)

                # Match descriptor
                matches = flann.knnMatch(desc_cam, desc_ref, k=2)

                # Apply ratio test
                good_matches = [m for m, n in matches if m.distance < 0.7 * n.distance]
                if len(good_matches) < 10:
                    print("Insufficient good matches.")
                    count += 1
                    rospy.sleep(0.1)
                    continue

                # Reset count of failed image matching attempts    
                count = 0

                # Get matched points
                src_pts = np.float32([kp_cam[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                dst_pts = np.float32([kp_ref[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                # Find homography with RANSAC
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

                if M is None:
                    print("Homography matrix could not be computed.")
                    continue

                # Get template dimensions        
                h, w = cam_gray.shape

                # Create template corners
                template_corners = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)

                # Transform corners to reference image space
                ref_corners = cv2.perspectiveTransform(template_corners, M)

                # Calculate center coordinates relative to reference center
                mean_x = np.mean(ref_corners[:, :, 0])
                mean_y = np.mean(ref_corners[:, :, 1])

                # Center pixel coordinates of camera image 
                # wrt Raster map image
                pixel_row = mean_y + row_start
                pixel_col = mean_x + col_start

                # Access GPS coordinates of the center pixel 
                lon, lat = xy(dataset.transform, pixel_row, pixel_col)
                current_gps = (lat, lon)

                # Calculate yaw angle
                tl_img = ref_corners[0][0]
                tr_img = ref_corners[3][0]
                deltaY = tr_img[0] - tl_img[0]
                deltaX = tr_img[1] - tl_img[1]
                angle_rad = np.arctan2(deltaX, deltaY)
                angle_deg = 360 - np.mod(np.degrees(angle_rad), 360)

                # Filter outputs
                filtered_lat = lat_filter.update(lat)
                filtered_lon = lon_filter.update(lon)
                filtered_yaw = yaw_filter.update(angle_deg)

                print("Latitude:", filtered_lat, "Longitude:", filtered_lon, "Yaw:", filtered_yaw)
                send_gps_udp(filtered_lat, filtered_lon, filtered_yaw)

                frequency = 1 / (time.time() - start_time)
                print("Frequency:", frequency)
                iteration += 1

                
            else:
                print("Unable to get enough features.")


#------------------------------------------------------------------------------------------------------#
if __name__ == "__main__":
    main()
