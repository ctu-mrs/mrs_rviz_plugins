/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <sensor_msgs/NavSatFix.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

#include <mrs_msgs/TransformPoseSrv.h>

#include <std_srvs/Trigger.h>

//}

namespace mrs_rviz_plugins
{

namespace rviz_interface
{

/* class PoseEstimate //{ */

class PoseEstimate : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

private:
  // subscribers and publishers
  ros::Subscriber sub_rviz_pose_estimate_;
  ros::Publisher  pub_pose_array_;
  ros::Publisher  pub_marker_array_;

  ros::Timer timer_pub_;

  // publisher rate
  int _pub_rate_;

  // mutex for locking the position info
  std::mutex mutex_rviz_pose_estimate_;
  std::mutex mutex_pose_array_;
  std::mutex mutex_utm_pose_array_;
  std::mutex mutex_marker_array_;
  std::mutex mutex_latlon_pose_array_;

  bool got_rviz_pose_estimate_ = false;

  // count of pose estimates
  int count_;

  // republished pose message
  geometry_msgs::PoseWithCovarianceStamped rviz_pose_estimate_;
  geometry_msgs::PoseArray                 pose_array_;
  geometry_msgs::PoseArray                 utm_pose_array_;
  geometry_msgs::PoseArray                 latlon_pose_array_;
  visualization_msgs::MarkerArray          marker_array_;

  bool               callbackDump(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  ros::ServiceServer srv_server_dump_;

private:
  void callbackRvizPoseEstimate(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void timerPub(const ros::TimerEvent &event);

private:
  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;
};

//}

// --------------------------------------------------------------
// |                      internal routines                     |
// --------------------------------------------------------------

/* onInit() //{ */

void PoseEstimate::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  mrs_lib::ParamLoader param_loader(nh_, "PoseEstimate");

  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  param_loader.loadParam("publish_rate", _pub_rate_);

  // SUBSCRIBERS
  sub_rviz_pose_estimate_ = nh_.subscribe("rviz_pose_estimate_in", 1, &PoseEstimate::callbackRvizPoseEstimate, this, ros::TransportHints().tcpNoDelay());

  // PUBLISHERS
  pub_pose_array_   = nh_.advertise<geometry_msgs::PoseArray>("pose_array_out", 1);
  pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_array_out", 1);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  timer_pub_ = nh_.createTimer(ros::Rate(_pub_rate_), &PoseEstimate::timerPub, this);

  // --------------------------------------------------------------
  // |                          services                          |
  // --------------------------------------------------------------

  srv_server_dump_ = nh_.advertiseService("dump_in", &PoseEstimate::callbackDump, this);

  // --------------------------------------------------------------
  // |                          profiler                          |
  // --------------------------------------------------------------

  profiler_ = mrs_lib::Profiler(nh_, "PoseEstimate", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[PoseEstimate]: Could not load all parameters!");
    ros::shutdown();
  }

  count_ = 0;

  is_initialized_ = true;

  ROS_INFO("[PoseEstimate]: initialized");

  ROS_INFO("[PoseEstimate]: Waiting for user input in rviz...");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackRvizPoseEstimate() //{ */

void PoseEstimate::callbackRvizPoseEstimate(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("callbackRvizPoseEstimate");

  got_rviz_pose_estimate_ = true;

  count_++;

  {
    std::scoped_lock lock(mutex_rviz_pose_estimate_);

    rviz_pose_estimate_ = *msg;
  }

  // Get uav name of the pose estimate frame
  size_t      idx_slash = msg->header.frame_id.find("/");
  std::string uav_name  = msg->header.frame_id.substr(0, idx_slash);

  // Prepare transformer
  ros::ServiceClient         srv_client_transform_pose = nh_.serviceClient<mrs_msgs::TransformPoseSrv>("/" + uav_name + "/control_manager/transform_pose");
  mrs_msgs::TransformPoseSrv transform_pose_srv;

  geometry_msgs::Pose new_pose;
  new_pose.position    = msg->pose.pose.position;
  new_pose.orientation = msg->pose.pose.orientation;

  {
    std::scoped_lock lock(mutex_pose_array_);

    pose_array_.poses.push_back(new_pose);
    pose_array_.header.stamp    = ros::Time::now();
    pose_array_.header.frame_id = msg->header.frame_id;
  }

  // rviz pose estimate marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = uav_name + "/gps_origin";
  marker.type            = visualization_msgs::Marker::CUBE_LIST;
  marker.ns              = "2d_rviz_pose_estimate";
  marker.id              = count_;
  marker.color.a         = 1;
  marker.scale.x         = 0.2;
  marker.scale.y         = 0.2;
  marker.scale.z         = 0.2;
  marker.color.r         = 1;
  marker.color.g         = 0;
  marker.color.b         = 0;

  geometry_msgs::Point pt_tmp;
  pt_tmp.x = new_pose.position.x;
  pt_tmp.y = new_pose.position.y;
  pt_tmp.z = new_pose.position.z;
  marker.points.push_back(pt_tmp);

  marker_array_.markers.push_back(marker);

  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = uav_name + "/gps_origin";
  text_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.ns              = "2d_rviz_pose_estimate_text";
  text_marker.id              = count_;
  text_marker.pose            = new_pose;
  text_marker.pose.position.z += 0.5;
  text_marker.color.a = 1;
  text_marker.scale.z = 0.2;
  text_marker.color.r = 1;
  text_marker.color.g = 0;
  text_marker.color.b = 0;
  std::stringstream ss;
  ss << "id: " << count_ << "\nx: " << pt_tmp.x << "\ny: " << pt_tmp.y;
  text_marker.text = ss.str();

  marker_array_.markers.push_back(text_marker);

  geometry_msgs::Quaternion orientation_tmp;
  double                    heading_tmp;

  // Current rviz frame
  orientation_tmp = msg->pose.pose.orientation;

  try {
    heading_tmp = mrs_lib::AttitudeConverter(orientation_tmp).getHeading();
  }
  catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
    heading_tmp = 0;
  }

  ROS_INFO("[PoseEstimate]: New rviz 2D pose estimate:");
  ROS_INFO("[PoseEstimate]: frame_id: %s\nx: %f y: %f yaw: %f", msg->header.frame_id.c_str(), msg->pose.pose.position.x, msg->pose.pose.position.y,
           heading_tmp);

  // GPS frame
  if (msg->header.frame_id != uav_name + "/gps_origin") {

    geometry_msgs::PoseStamped gps_pose;
    gps_pose.pose            = new_pose;
    gps_pose.header.frame_id = msg->header.frame_id;
    gps_pose.header.stamp    = ros::Time::now();

    transform_pose_srv.request.frame_id = "gps_origin";
    transform_pose_srv.request.pose     = gps_pose;

    /* ROS_INFO("[PoseEstimate]: Calling transformer service: %s", ("/" + uav_name + "/control_manager/transform_pose").c_str()); */

    srv_client_transform_pose.call(transform_pose_srv);

    /* ROS_INFO("[PoseEstimate]: Transformer service response: %s", transform_pose_srv.response.message.c_str()); */

    if (transform_pose_srv.response.success) {
      gps_pose = transform_pose_srv.response.pose;
    } else {
      ROS_ERROR("[PoseEstimate]: Could not transform to gps_origin.");
      return;
    }

    orientation_tmp = gps_pose.pose.orientation;
    try {
      heading_tmp = mrs_lib::AttitudeConverter(orientation_tmp).getHeading();
    }
    catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
      heading_tmp = 0;
    }

    ROS_INFO("[PoseEstimate]: frame_id: %s\nx: %f y: %f yaw: %f", gps_pose.header.frame_id.c_str(), gps_pose.pose.position.x, gps_pose.pose.position.y,
             heading_tmp);
  }

  // UTM frame
  geometry_msgs::PoseStamped utm_pose;
  utm_pose.pose            = new_pose;
  utm_pose.header.frame_id = msg->header.frame_id;
  utm_pose.header.stamp    = ros::Time::now();

  transform_pose_srv.request.frame_id = "utm_origin";
  transform_pose_srv.request.pose     = utm_pose;

  /* ROS_INFO("[PoseEstimate]: Calling transformer service: %s", ("/" + uav_name + "/control_manager/transform_pose").c_str()); */

  srv_client_transform_pose.call(transform_pose_srv);

  /* ROS_INFO("[PoseEstimate]: Transformer service response: %s", transform_pose_srv.response.message.c_str()); */

  if (transform_pose_srv.response.success) {
    utm_pose = transform_pose_srv.response.pose;
  } else {
    ROS_ERROR("[PoseEstimate]: Could not transform to utm_origin.");
    return;
  }
  orientation_tmp = utm_pose.pose.orientation;

  try {
    heading_tmp = mrs_lib::AttitudeConverter(orientation_tmp).getHeading();
  }
  catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
    heading_tmp = 0;
  }

  ROS_INFO("[PoseEstimate]: frame_id: %s\nx: %f y: %f yaw: %f", utm_pose.header.frame_id.c_str(), utm_pose.pose.position.x, utm_pose.pose.position.y,
           heading_tmp);

  {
    std::scoped_lock lock(mutex_utm_pose_array_);

    utm_pose_array_.poses.push_back(utm_pose.pose);
    utm_pose_array_.header.stamp    = ros::Time::now();
    utm_pose_array_.header.frame_id = utm_pose.header.frame_id;
  }

  // LatLon frame
  geometry_msgs::PoseStamped latlon_pose;
  latlon_pose.pose            = new_pose;
  latlon_pose.header.frame_id = msg->header.frame_id;
  latlon_pose.header.stamp    = ros::Time::now();

  transform_pose_srv.request.frame_id = "latlon_origin";
  transform_pose_srv.request.pose     = latlon_pose;

  srv_client_transform_pose.call(transform_pose_srv);

  if (transform_pose_srv.response.success) {
    latlon_pose = transform_pose_srv.response.pose;
  } else {
    ROS_ERROR("[PoseEstimate]: Could not transform to latlon_origin.");
    return;
  }

  orientation_tmp = latlon_pose.pose.orientation;

  try {
    heading_tmp = mrs_lib::AttitudeConverter(orientation_tmp).getHeading();
  }
  catch (mrs_lib::AttitudeConverter::GetHeadingException &e) {
    heading_tmp = 0;
  }

  ROS_INFO("[PoseEstimate]: frame_id: %s\nlatitude: %f longitude: %f yaw: %f", latlon_pose.header.frame_id.c_str(), latlon_pose.pose.position.x,
           latlon_pose.pose.position.y, heading_tmp);
  {
    std::scoped_lock lock(mutex_latlon_pose_array_);

    latlon_pose_array_.poses.push_back(latlon_pose.pose);
    latlon_pose_array_.header.stamp    = ros::Time::now();
    latlon_pose_array_.header.frame_id = latlon_pose.header.frame_id;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerPub() //{ */

void PoseEstimate::timerPub(const ros::TimerEvent &event) {

  if (!is_initialized_)
    return;

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("timerPub", _pub_rate_, 0.01, event);

  if (!got_rviz_pose_estimate_)
    return;

  geometry_msgs::PoseArray pose_array_out;

  {
    std::scoped_lock lock(mutex_pose_array_);

    pose_array_out = pose_array_;
  }

  try {
    pub_pose_array_.publish(pose_array_out);
  }
  catch (...) {
    ROS_ERROR("[PoseEstimate]: Exception caught during publishing topic %s.", pub_pose_array_.getTopic().c_str());
  }

  visualization_msgs::MarkerArray marker_array_out;

  {
    std::scoped_lock lock(mutex_marker_array_);

    marker_array_out = marker_array_;
  }

  try {
    pub_marker_array_.publish(marker_array_out);
  }
  catch (...) {
    ROS_ERROR("[PoseEstimate]: Exception caught during publishing topic %s.", pub_marker_array_.getTopic().c_str());
  }
}

//}

/* callbackDump() //{ */

bool PoseEstimate::callbackDump([[maybe_unused]] std_srvs::Trigger::Request &req, [[maybe_unused]] std_srvs::Trigger::Response &res) {

  if (!is_initialized_) {

    ROS_WARN("[PoseEstimate]: Cannot dump coordinates, nodelet is not initialized.");
    res.message = "Cannot dump coordinates, nodelet is not initialized.";
    res.success = false;
    return true;
  }

  if (!got_rviz_pose_estimate_) {
    ROS_WARN("[PoseEstimate]: Cannot dump coordinates, no point available.");
    res.message = "Cannot dump coordinates, no point available.";
    res.success = false;
    return true;
  }

  ROS_INFO("[PoseEstimate]: UTM pose array dump:");
  std::stringstream ss;
  ss << "UTM: ";
  {
    std::scoped_lock lock(mutex_utm_pose_array_);

    for (size_t i = 0; i < utm_pose_array_.poses.size(); i++) {
      std::printf("[%.6f, %.6f]\n", utm_pose_array_.poses[i].position.x, utm_pose_array_.poses[i].position.y);
      ss << "[" << utm_pose_array_.poses[i].position.x << ", " << utm_pose_array_.poses[i].position.x << "] ";
    }
  }

  ROS_INFO("[PoseEstimate]: LatLon pose array dump:");
  ss << "LatLon: ";
  {
    std::scoped_lock lock(mutex_latlon_pose_array_);

    for (size_t i = 0; i < latlon_pose_array_.poses.size(); i++) {
      std::printf("[%.6f, %.6f]\n", latlon_pose_array_.poses[i].position.x, latlon_pose_array_.poses[i].position.y);
      ss << "[" << latlon_pose_array_.poses[i].position.x << ", " << latlon_pose_array_.poses[i].position.x << "] ";
    }
  }

  res.message = ss.str();
  res.success = true;
  return true;
}

//}

}  // namespace rviz_interface

}  // namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::rviz_interface::PoseEstimate, nodelet::Nodelet)
