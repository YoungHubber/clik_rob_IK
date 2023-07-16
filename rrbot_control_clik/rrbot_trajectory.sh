#!/bin/bash

# Pose 1
rostopic pub /rrbot/pose geometry_msgs/Pose "position:
  x: 3.0
  y: 5.0
  z: 3.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0" -1

# Delay to allow robot to reach pose
sleep 5

# Pose 2
rostopic pub /rrbot/pose geometry_msgs/Pose "position:
  x: 0.5
  y: 0.0
  z: 1.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0" -1

# Delay to allow robot to reach pose
sleep 5

# Pose 3
rostopic pub /rrbot/pose geometry_msgs/Pose "position:
  x: 0.5
  y: 0.0
  z: 0.5
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0" -1

