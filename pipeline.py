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

# ========== CONFIGURATION ==========
TIFF_PATH = "Mumbai 7500.tif"
REFERENCE_SIZE = 500
INITIAL_GPS = (18.992394752836383, 72.83529905339414)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPV4, UDP
out_addr = ("127.0.0.1", 25100)

# Connect to SITL
# vehicle = connect('127.0.0.1:14550', wait_ready=True)
# ========== CAMERA CLASS ==========
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


class Camera:
    def __init__(self, topic="/webcam/image_raw"):
        self.bridge = CvBridge()
        self.image = None
        self.lock = threading.Lock()
        rospy.Subscriber(topic, Image, self.callback)
        rospy.loginfo(f"Subscribed to camera topic: {topic}")

    def callback(self, msg):
        try:
            # Convert to BGR color image
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            with self.lock:
                self.image = img.copy()
        except Exception as e:
            rospy.logwarn(f"Image conversion failed: {e}")

    def get_image(self):
        with self.lock:
            return self.image.copy() if self.image is not None else None
        

# ========== MAIN LOOP ==========
def main():
    rospy.init_node('gps_image_matcher')
    sift = cv2.SIFT_create()
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera = Camera()
    current_gps = INITIAL_GPS

    with rasterio.open(TIFF_PATH) as dataset:

        image = dataset.read()
        _, height, width = image.shape

        # Get GPS coordinates of key corners using (row, col)
        top_left_latlon = xy(dataset.transform, 0, 0)
        bottom_left_latlon = xy(dataset.transform, height - 1, 0)
        bottom_right_latlon = xy(dataset.transform, height - 1, width - 1)

        # Calculate distances using Haversine formula
        height_m = haversine((top_left_latlon[1], top_left_latlon[0]), (bottom_left_latlon[1], bottom_left_latlon[0]))
        width_m = haversine((bottom_left_latlon[1], bottom_left_latlon[0]), (bottom_right_latlon[1], bottom_right_latlon[0]))

        h_acc = 0.5 * ((width_m/width) + (height_m/height)) 

        count = 0
        while not rospy.is_shutdown():

            if count < 5:
                start_time = time.time()
                # Get the current camera image
                cam_gray = camera.get_image()
                if cam_gray is None:
                    rospy.logwarn("Waiting for camera image...")
                    rospy.sleep(0.1)
                    continue

                # cv2.imshow("Camera Image", cam_img)
                # cv2.waitKey(1)

                ref_row, ref_col = rowcol(dataset.transform, current_gps[1], current_gps[0])

                # Define centered window
                row_start = max(0, ref_row - REFERENCE_SIZE // 2)
                row_end = min(dataset.height, ref_row + REFERENCE_SIZE // 2)
                col_start = max(0, ref_col - REFERENCE_SIZE // 2)
                col_end = min(dataset.width, ref_col + REFERENCE_SIZE // 2)

                # Read RGB bands (assuming bands 1, 2, 3 are R, G, B)
                r = dataset.read(1, window=((row_start, row_end), (col_start, col_end)))
                g = dataset.read(2, window=((row_start, row_end), (col_start, col_end)))
                b = dataset.read(3, window=((row_start, row_end), (col_start, col_end)))

                # Stack and convert to BGR (OpenCV expects BGR)
                ref_image = np.dstack((b, g, r))
                ref_center = ((row_start+row_end)//2, (col_start+col_end)//2)

                ref_gray = cv2.cvtColor(ref_image, cv2.COLOR_BGR2GRAY)

                # Find keypoints and descriptors
                kp_ref, desc_ref = sift.detectAndCompute(ref_gray, None)
                kp_cam, desc_cam = sift.detectAndCompute(cam_gray, None)
        
                if desc_ref is None or desc_cam is None:
                    print("Insufficient Descriptors.")
                    count+=1
                    rospy.sleep(0.1)
                    continue

                # FLANN matcher
                index_params = dict(algorithm=1, trees=5)
                search_params = dict(checks=50)
                flann = cv2.FlannBasedMatcher(index_params, search_params)
            
                # Match descriptors
                matches = flann.knnMatch(desc_cam, desc_ref, k=2)
                
                # Apply ratio test
                good_matches = []
                for m, n in matches:
                    if m.distance < 0.7 * n.distance:
                        good_matches.append(m)

                if len(good_matches) < 10:
                    print("Insufficient good matches.")
                    count+=1
                    rospy.sleep(0.1)
                    continue

                count=0
                
                # Get matched points
                src_pts = np.float32([kp_cam[m.queryIdx].pt for m in good_matches]).reshape(-1,1,2)
                dst_pts = np.float32([kp_ref[m.trainIdx].pt for m in good_matches]).reshape(-1,1,2)
                
                # Find homography with RANSAC
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

                if M is None:
                    print("Homography matrix could not be computed.")
                    continue

                # Get template dimensions
                h, w = cam_gray.shape
                
                # Create template corners
                template_corners = np.float32([[0,0], [0,h-1], [w-1,h-1], [w-1,0]]).reshape(-1,1,2)
                
                # Transform corners to reference image space
                ref_corners = cv2.perspectiveTransform(template_corners, M)
                
                # Calculate center coordinates relative to reference center
                mean_x = np.mean(ref_corners[:,:,0])
                mean_y = np.mean(ref_corners[:,:,1])
                center_x = mean_y - ref_center[1]  # X = y - center_y (down +)
                center_y = mean_x - ref_center[0]  # Y = x - center_x (right +)

                pixel_row = mean_y + row_start
                pixel_col = mean_x + col_start
                            
                lon, lat = xy(dataset.transform, pixel_row, pixel_col)
                current_gps = (lat, lon)
                print("Latitude:", lat, "Longitude:", lon)

                # Calculate rotation angle using top edge vector
                tl_img = ref_corners[0][0]  # Top-left in image coordinates (x, y)
                tr_img = ref_corners[3][0]  # Top-right in image coordinates (x, y)
                
                deltaY = tr_img[0] - tl_img[0]  # Change in image x (Y in user's system)
                deltaX = tr_img[1] - tl_img[1]  # Change in image y (X in user's system)
                angle_rad = np.arctan2(deltaX, deltaY)
                angle_deg = np.degrees(angle_rad)
                angle_deg = np.mod(angle_deg, 360)  # Normalize to 0-360 degrees

        
                # Send GPS over UDP
                gps_msg = {
                        'time_usec' : 0,                        # (uint64_t) Timestamp (micros since boot or Unix epoch)
                        'gps_id' : 0,                           # (uint8_t) ID of the GPS for multiple GPS inputs
                        'ignore_flags' : 8,                     # (uint16_t) Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
                        'time_week_ms' : 0,                     # (uint32_t) GPS time (milliseconds from start of GPS week)
                        'time_week' : 0,                        # (uint16_t) GPS week number
                        'fix_type' : 3,                         # (uint8_t) 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
                        'lat' : int(lat * 1E7),                      # (int32_t) Latitude (WGS84), in degrees * 1E7
                        'lon' : int(lon * 1E7),                      # (int32_t) Longitude (WGS84), in degrees * 1E7
                        'alt' : 0,                              # (float) Altitude (AMSL, not WGS84), in m (positive for up)
                        'hdop' : 1,                             # (float) GPS HDOP horizontal dilution of position in m
                        'vdop' : 1,                             # (float) GPS VDOP vertical dilution of position in m
                        'vn' : 0,                               # (float) GPS velocity in m/s in NORTH direction in earth-fixed NED frame
                        've' : 0,                               # (float) GPS velocity in m/s in EAST direction in earth-fixed NED frame
                        'vd' : 0,                               # (float) GPS velocity in m/s in DOWN direction in earth-fixed NED frame
                        'speed_accuracy' : 0,                   # (float) GPS speed accuracy in m/s
                        'horiz_accuracy' : 0,                   # (float) GPS horizontal accuracy in m
                        'vert_accuracy' : 0,                    # (float) GPS vertical accuracy in m
                        'satellites_visible' : 13                # (uint8_t) Number of satellites visible.
                }

                out_data = json.dumps(gps_msg)
                # print('out:',out_data)
                s.sendto(out_data.encode(), out_addr)

                frequency = 1 / (time.time() - start_time)
                print("Frequency:", frequency)

                # === Display Reference Image with Matched Center ===
                # Draw a circle at the matched position
                marker_x = int(mean_x)
                marker_y = int(mean_y)
                cv2.circle(ref_image, (marker_x, marker_y), 10, (0, 0, 255), 2)

                # Resize to 640x480
                annotated_resized = cv2.resize(ref_image, (480, 480))

                # cv2.imshow("Reference Image Match", annotated_resized)
                cv2.imshow("Reference Image Match", ref_image)
                cv2.waitKey(1)


            else:
                print("Unable to get enough features.")

if __name__ == "__main__":
    main()
