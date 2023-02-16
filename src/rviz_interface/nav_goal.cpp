/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>

#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/ReferenceStampedSrv.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

//}

namespace mrs_rviz_plugins
{

namespace rviz_interface
{

/* class NavGoal //{ */

class NavGoal : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  // | ---------------------- msg callbacks --------------------- |

private:
  void                       callbackRvizNavGoal(const geometry_msgs::PoseStampedConstPtr& msg);
  ros::Subscriber            sub_rviz_nav_goal_;
  geometry_msgs::PoseStamped rviz_nav_goal_;
  std::mutex                 mutex_rviz_nav_goal_;
  int                        seq_ = 0;

  void               callbackOdomUav(const nav_msgs::OdometryConstPtr& msg);
  nav_msgs::Odometry odom_uav_;
  bool               got_odom_uav_ = false;
  std::mutex         mutex_odom_uav_;

  void               callbackTrackerSetpoint(const nav_msgs::OdometryConstPtr& msg);
  nav_msgs::Odometry tracker_setpoint_;
  bool               got_tracker_setpoint_ = false;
  std::mutex         mutex_tracker_setpoint_;

private:
  mrs_lib::Profiler* profiler_;
  bool               _profiler_enabled_ = false;
};

//}

/* onInit() //{ */

void NavGoal::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  mrs_lib::ParamLoader param_loader(nh_, "NavGoal");

  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  // | ------------------ initialize subscribers ----------------- |
  sub_rviz_nav_goal_ = nh_.subscribe("rviz_nav_goal_in", 1, &NavGoal::callbackRvizNavGoal, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- finish init ---------------------- |
  is_initialized_ = true;

  ROS_INFO_ONCE("[NavGoal]: initialized");

  ROS_INFO("[RvizPoseEstimate]: Waiting for user input in rviz...");
}
//}

// | ---------------------- msg callbacks --------------------- |

/* callbackRvizNavGoal() //{ */

void NavGoal::callbackRvizNavGoal(const geometry_msgs::PoseStampedConstPtr& msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_)
    return;

  got_odom_uav_ = false;

  geometry_msgs::PoseStamped rviz_nav_goal_tmp = *msg;

  ROS_INFO("[NavGoal]: Received rviz nav goal msg #%d", seq_ + 1);

  // Get uav name of the pose estimate frame
  size_t      idx_slash = msg->header.frame_id.find("/");
  std::string uav_name  = msg->header.frame_id.substr(0, idx_slash);

  ROS_INFO("[NavGoal]: Resolved UAV name to: %s", uav_name.c_str());

  ros::Subscriber sub_odom_uav =
      nh_.subscribe("/" + uav_name + "/control_manager/control_reference", 1, &NavGoal::callbackOdomUav, this, ros::TransportHints().tcpNoDelay());

  // Wait 1s for UAV odometry
  ros::Duration timeout(1.0);
  ros::Duration total_sleep_time(0.0);
  while (!got_odom_uav_ && total_sleep_time < timeout) {
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
    total_sleep_time += sleep_time;
  }

  if (total_sleep_time >= timeout) {
    ROS_INFO("[NavGoal]: UAV odometry %f s timeout.", total_sleep_time.toSec());
    got_odom_uav_         = false;
    got_tracker_setpoint_ = false;
    return;
  }
  ROS_INFO("[NavGoal]: Waited %f s for UAV odometry.", total_sleep_time.toSec());

  double setpoint_z;

  // Set setpoint height to current height for global frames (ending with origin)
  // Set setpoint height to 0 for body-fixed frames
  std::size_t found = msg->header.frame_id.find("_origin");
  if (found != std::string::npos) {
    std::scoped_lock lock(mutex_odom_uav_);

    setpoint_z = odom_uav_.pose.pose.position.z;
  } else {
    setpoint_z = 0.0;
  }

  // Create new waypoint msg
  mrs_msgs::ReferenceStampedSrv new_waypoint;
  new_waypoint.request.header.seq           = seq_;
  new_waypoint.request.header.frame_id      = rviz_nav_goal_tmp.header.frame_id;
  new_waypoint.request.header.stamp         = ros::Time::now();
  new_waypoint.request.reference.position.x = rviz_nav_goal_tmp.pose.position.x;
  new_waypoint.request.reference.position.y = rviz_nav_goal_tmp.pose.position.y;
  new_waypoint.request.reference.position.z = setpoint_z;

  try {
    new_waypoint.request.reference.heading = mrs_lib::AttitudeConverter(rviz_nav_goal_tmp.pose.orientation).getHeading();
  }
  catch (mrs_lib::AttitudeConverter::GetHeadingException& e) {
    new_waypoint.request.reference.heading = 0;
  }

  ros::ServiceClient srv_client_reference = nh_.serviceClient<mrs_msgs::ReferenceStampedSrv>("/" + uav_name + "/control_manager/reference");

  ROS_INFO("[NavGoal]: Calling reference service: %s", ("/" + uav_name + "/control_manager/reference").c_str());
  srv_client_reference.call(new_waypoint);
  ROS_INFO("[NavGoal]: Reference service response: %s", new_waypoint.response.message.c_str());

  if (!new_waypoint.response.success) {
    ROS_ERROR("[NavGoal]: Could not set reference.");
    got_odom_uav_         = false;
    got_tracker_setpoint_ = false;
    return;
  }

  ros::Subscriber sub_setpoint_odom = nh_.subscribe("/" + uav_name + "/control_manager/mpc_tracker/setpoint_odom", 1, &NavGoal::callbackTrackerSetpoint, this,
                                                    ros::TransportHints().tcpNoDelay());

  // Wait 1s for setpoint odometry
  ros::Duration total_setpoint_sleep_time(0.0);
  while (!got_tracker_setpoint_ && total_setpoint_sleep_time < ros::Duration(1.0)) {
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
    total_setpoint_sleep_time += sleep_time;
  }
  ROS_INFO("[NavGoal]: Waited %f s for setpoint odometry.", total_setpoint_sleep_time.toSec());

  // Wait 1s for tracking start
  ros::Duration total_diag_sleep_time(0.0);
  while (total_diag_sleep_time < ros::Duration(1.0)) {

    geometry_msgs::Pose tracker_pose_tmp;
    {
      std::scoped_lock lock(mutex_tracker_setpoint_);

      tracker_pose_tmp = tracker_setpoint_.pose.pose;
    }

    double          diff_x = std::abs(tracker_pose_tmp.position.x - rviz_nav_goal_tmp.pose.position.x);
    double          diff_y = std::abs(tracker_pose_tmp.position.y - rviz_nav_goal_tmp.pose.position.y);
    tf2::Quaternion q1, q2;
    tf2::fromMsg(tracker_pose_tmp.orientation, q1);
    tf2::fromMsg(rviz_nav_goal_tmp.pose.orientation, q2);
    tf2::Quaternion rot_diff = q2 * q1.inverse();

    double diff_hdg = 0;

    try {
      diff_hdg = std::abs(mrs_lib::AttitudeConverter(rot_diff).getHeading());
    }
    catch (mrs_lib::AttitudeConverter::GetHeadingException& e) {
      diff_hdg = std::numeric_limits<double>::max();
    }

    /* ROS_INFO("diff: x: %2.4f y: %2.4f yaw: %2.4f\n", diff_x, diff_y, diff_yaw); */
    if (diff_x < 0.1 && diff_y < 0.1 && diff_hdg < 0.1) {
      break;
    }

    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
    total_diag_sleep_time += sleep_time;
  }

  ROS_INFO("[NavGoal]: Waited %f s for tracker setpoint.", total_diag_sleep_time.toSec());

  ROS_INFO("[NavGoal]: Flying to rviz nav goal:");
  ROS_INFO("frame_id: %s\tx: %2.2f y: %2.2f z: %2.2f heading: %2.2f\n", new_waypoint.request.header.frame_id.c_str(), new_waypoint.request.reference.position.x,
           new_waypoint.request.reference.position.y, new_waypoint.request.reference.position.z, new_waypoint.request.reference.heading);

  got_odom_uav_         = false;
  got_tracker_setpoint_ = false;
}

//}

/* callbackOdomUav() //{ */

void NavGoal::callbackOdomUav(const nav_msgs::OdometryConstPtr& msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_)
    return;

  {
    std::scoped_lock lock(mutex_odom_uav_);
    odom_uav_ = *msg;
  }

  if (!got_odom_uav_) {
    got_odom_uav_ = true;
  }
}

//}

/* callbackTrackerSetpoint() //{ */

void NavGoal::callbackTrackerSetpoint(const nav_msgs::OdometryConstPtr& msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_)
    return;

  {
    std::scoped_lock lock(mutex_tracker_setpoint_);

    tracker_setpoint_ = *msg;
  }
  got_tracker_setpoint_ = true;
}

//}

}  // namespace rviz_interface

}  // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::rviz_interface::NavGoal, nodelet::Nodelet);
