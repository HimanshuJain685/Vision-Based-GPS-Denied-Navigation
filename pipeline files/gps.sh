#!/bin/bash



BASE_COMMAND="cd && cd ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter"

# Launch ArduCopter simulation with map and console and fixed MAVLink out address
gnome-terminal --tab --title="drone1" --command="bash -c '$BASE_COMMAND -f gazebo-drone1 --map --console -L mum; exec bash'"


