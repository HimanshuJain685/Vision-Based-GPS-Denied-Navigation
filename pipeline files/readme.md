# Components
- Georeferenced Raster image- Mumbai 7500.tif [https://drive.google.com/file/d/1XWH18RVO8AXFv1XUFBlcc0Y97nSwY1Gd/view?usp=drive_link]
- Locations file for Ardupilot SITL- locations.txt 
- Code to visualize predictions given by pipeline: pipeline.py
- Code to feed a constant GPS coordinate to SITL: takeoff_gps_feed.py
- Code to use for Vision based navigation: filtered_pipeline.py
- Code to check dimensions and Center GPS coordinate of given GeoTiff Image: dimensions.py

# Instructions:
0. Make sure the camera topic and the path to Georeferenced path and the UDP address for GPS feed given in pipeline scripts are correct.
 
1. Get the Georeferenced raster map image from the link.

2. Copy the last location from locations.txt file: mum=18.992394752836383,72.83529905339414,11,0 to your location.txt file.

3. Initialize Gazebo world using 'roslaunch <pkg> custom_ground.world', and initialize Ardupilot SITL instance using ./gps.sh
 
4. To visualize the pipeline predictions: Make pipeline.py executable (chmod +x pipeline.py), set mode to guided (mode guided), arm the drone (arm throttle), takeoff to a suitable altitude (takeoff 50) and run after reaching the altitude: python3 pipeline.py

5. For vision based navigation: 
				1. Set parameters: GPS_TYPE=14 and SIM_GPS_DISABLE 1. Adjust BLENDING_ITERATIONS in filtered_pipeline.py as required.
				2. Make takeoff_gps_feed.py and filtered_pipeline.py executable  
				3. Type 'module load GPSInput' and run: python3 takeoff_gps_feed.py
				4. Set mode to guided (mode guided), arm the drone (arm throttle), takeoff to a suitable altitude (takeoff 50).
				5. Run after reaching the altitude: python3 filtered_pipeline.py and once GPS coordinates are being predicted terminate the process takeoff_gps_feed.py.
				 


