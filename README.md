# MRS Rviz Plugins

![](.fig/thumbnail.jpg)

| Build status | [![Build Status](https://github.com/ctu-mrs/mrs_rviz_plugins/workflows/Melodic/badge.svg)](https://github.com/ctu-mrs/mrs_rviz_plugins/actions) | [![Build Status](https://github.com/ctu-mrs/mrs_rviz_plugins/workflows/Noetic/badge.svg)](https://github.com/ctu-mrs/mrs_rviz_plugins/actions) |
|--------------|-------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------|

## Plugins

#### mrs_msgs/ObstacleSectors vizualization

"Bumper" vizualizations, integrates seamlessly.

#### mrs_msgs/PoseWithCovarianceStamped vizualization

Integrates seamlessly.

## ROS Nodes - the ROS-Rviz interface

#### RvizNavGoal

Allows giving a reference to a UAV using the "2D Nav Goal" button in Rviz.
The click-and-pull input supplies a 2D position with heading.

#### RvizPoseEstimate

Allows obtaining a coordinates from Rviz by using the "2D Pose Estimate" button in Rviz.
The coordinates will appear as a standard output

## Utils

#### UAV Airframe vizualization

```bash
roslaunch mrs_rviz_plugins load_robot.launch
```
