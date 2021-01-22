#!/usr/bin/env bash

# rostopic pub /sphere mrs_msgs/Sphere "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: 'uav91/aloam_origin'
# position:
#   x: 1.0
#   y: 1.0
#   z: 10.0
# radius: 10.0" 

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


rostopic pub /posearr mrs_msgs/PoseWithCovarianceArrayStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'uav91/aloam_origin'
poses:
- id: 0
  pose:
    position: {x: 0.0, y: 1.0, z: 3.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [
  1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 3.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.3, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.3, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.3]
- id: 1
  pose:
    position: {x: 5.0, y: 1.2, z: 1.5}
    orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}
  covariance: [
  1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 7.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.1]"
  
