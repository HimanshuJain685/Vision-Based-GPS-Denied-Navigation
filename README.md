# Multi Sensor Fusion for GPS Denied Navigation

## Contents:

- "cv_models" -> jupyter notebooks for all models. Cell outputs within the notebooks contain visualization of matches and performance metrics.
- "dataset" -> Images used to train and test the cv models. Folder contains some sample images. Entire dataset may be accessed from the drive links in the read me file.
- "gazebo files" -> Files and plugins used for the gazebo simulation.
- "pipeline files" -> Code used to execute the image localization models in conjunction with the gazebo simulation.
 

## Instructions: 

1. Clone the repository

2. You may test any of the image localization models from 'cv_models' folder.

3. Sample dataset and codes for generating Reference Image and Simulated Camera Image pairs from Raster map images are provided in 'dataset' folder.

4. The models, world file and launch file required for Gazebo simulation are provided in 'gazebo files' folder. 

5. 'pipeline files' folder contains the main pipeline for Image Matching and Visual Navigation in UAVs (in Gazebo + Ardupilot SITL) and the instructions to run the scripts.

6. The theory presentation file 'Multi_Sensor_Fusion_for_UAVs_in_GPS_Denied_Env.pdf' and the implementation videos ('Visualizing reference image matches.mp4' and 'EKF variance failure in Visual Navigation mode.mp4') within it are uploaded as well.
