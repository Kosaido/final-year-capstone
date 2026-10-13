#!/bin/bash
# Everything this project needs at boot, in one place: start the
# MicroXRCEAgent bridge, give it a moment to come up, then run the
# GPS check. This is the modern equivalent of the old
# start_micro_ros.sh -- Ubuntu's systemd is what actually calls this
# file at power-on (see project.service), there's no separate "boot
# script slot" this drops into by itself.

# EDIT this line to match your real agent command/device if it changes:
/usr/local/bin/MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 &
AGENT_PID=$!

# Give the agent a few seconds to open the port and come up before
# the GPS check starts looking for topics.
sleep 5

source /opt/ros/humble/setup.bash #Change to Jazzy if the drone uses Jazzy
source /home/car/ros2_ws/install/setup.bash #change the username accordingly
python3 /home/car/gps_position_check.py --own 0 --topic /fmu/out/vehicle_global_position

# If the GPS check exits for any reason, stop the agent too instead
# of leaving an orphaned background process running.
kill "$AGENT_PID" 2>/dev/null
