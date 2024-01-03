# MRS Rviz Plugins

![](.fig/thumbnail.jpg)

> :warning: **Attention please: This README is outdated.**
>
> The MRS UAV System 1.5 is being released and this page needs updating. Please, keep in mind that the information on this page might not be valid.

## Plugins

#### mrs_msgs/ObstacleSectors vizualization

"Bumper" vizualizations, integrates seamlessly.

#### mrs_msgs/PoseWithCovarianceStamped vizualization

Integrates seamlessly.

#### mrs_msgs/TrackArrayStamped vizualization

Integrates seamlessly.

#### mrs_msgs/UavStatus vizualization

Displays useful information about the UAV state and sensors, integrates seamlessly.
Use `mrs_rviz_plugins/UAV Status` display type.

#### nav_msgs/Odometry vizualization

Includes visualization of velocity and its covariance, integrates seamlessly.
Use `mrs_rviz_plugins/OdometryWithVelocity` display type.

#### NamedSetGoal

RViz goal tool with modifiable label.

#### World Manager

Has full InteractionTool functionality + allows adding new obstacles to the world and saving the world configuration.  
 - To add new obstacle, right-click on the desired place and select "Add obstacle".   
 - To load config, right-click, select file with world configuration.
 - To save current world configuration, right-click, select "Save world config" and choose file to save the configuration. If more than 1 drone is active in the tool properties, postfix "uav_name" will be added to filenames in order to avoid conflicts.  

Any of this will not work in case you clicked on an interactive marker.

#### Control tool

Integrates [mrs_uav_status](https://github.com/ctu-mrs/mrs_uav_status/tree/master) and Selection tool functionality into one tool.  
Shortcut key for the tool is 'c'.

##### Mouse control

|        Mouse event        |            Action            |
|:-------------------------:|:----------------------------:|
|      Click and drag       | Select objects on the screen |
|  Shift + Click and drag   |  Move 'UAV Status' display   |
|  Right-click (on drone)   |  Show services to be called  |
|     Alt + Left-click      |            Rotate            |
|    Alt + Middle-click     |           Move X/Y           |
| Alt + Shift + Left-click  |           Move X/Y           |
|     Alt + Right-click     |            Move Z            |
| Alt + Shift + Right-click |            Move Z            |

##### Controlling the UAV through RVIZ  

Press the 'R' key to enter the "remote" mode. While in this mode, you can fly the UAV with your keyboard.
While in remote mode, press 'G' to switch to global frame.
Only one key at a time is registered, multiple key inputs are not supported.

|        Key       |        Action        |
|:----------------:|:--------------------:|
| 'wasd' or 'hjkl' |     Fly laterally    |
|       'qe'       | Change UAV's heading |
|       'rf'       |    Fly up and down   |

##### Custom services

You can add your own services to drone's menu.
 * To add a service to the menu, publish a message to the topic ```mrs_uav_status/set_trigger_service```
 * Only services of the [std_srvs/Trigger](http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html) type are supported
 * The message is a [std_msgs/String](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html), and has to consist of two entries separated by spaces:
   * Service name (```uav_manager/land_home```)
   * Name to be displayed in the menu (```Land Home```) - this name can contain additional spaces
 * The namespace of the UAV will be added automatically (```uav_manager/land_home``` -> ```/uav1/uav_managerland_home```)
 * To a service outside of the namespace, use "/" as the first character (```/uav_manager/land_home```)

Press the 'm' key (as menu) to show services on selected drones. Custom services will not be shown, only the default ones.

#### WaypointPlanner

Allows sending a sequence of waypoints to drone.
The click-and-pull input supplies a 2D position with heading (a waypoint). Tool properties allow customization (height change, fly now, loop, use heading).

|  Key  |         Action         |
|:-----:|:----------------------:|
|   w   | Turn the plugin on/off |
|  del  |  Delete last waypoint  |
| enter |   Trigger behaviour    |

![Demonstration](icons/classes/Waypoint_planner_demonstration.gif)

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
