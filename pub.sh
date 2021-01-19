#!/usr/bin/env bash

rostopic pub /sphere mrs_msgs/Sphere "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'uav91/aloam_origin'
position:
  x: 5.0
  y: 2.0
  z: 2.0
radius: 1.0" 

# rostopic pub /pt geometry_msgs/PointStamped "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: 'uav91/aloam_origin'
# point:
#   x: 0.0
#   y: 0.0
#   z: 0.0" 
