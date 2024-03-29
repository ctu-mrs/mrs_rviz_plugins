#include "control/drone_entity.h"

using namespace visualization_msgs;

namespace mrs_rviz_plugins
{

DroneEntity::DroneEntity(const std::string &name_) {
  name                       = name_;
  nh                         = ros::NodeHandle(name);
  status_subscriber          = nh.subscribe("mrs_uav_status/uav_status", 1, &DroneEntity::statusCallback, this, ros::TransportHints().tcpNoDelay());
  custom_services_subsrciber = nh.subscribe("mrs_uav_status/set_trigger_service", 5, &DroneEntity::newSeviceCallback, this, ros::TransportHints().tcpNoDelay());
  position_cmd_subscriber    = nh.subscribe("control_manager/position_cmd", 5, &DroneEntity::positionCmdCallback, this, ros::TransportHints().tcpNoDelay());

  server = new interactive_markers::InteractiveMarkerServer("control", name.c_str(), true);

  // | ------------------------ Services ------------------------ |
  service_goto_reference  = mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv>(nh, "control_manager/reference");
  service_land            = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, "uav_manager/land");
  service_land_home       = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, "uav_manager/land_home");
  service_takeoff         = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, "uav_manager/takeoff");
  service_set_constraints = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh, "constraint_manager/set_constraints");
  service_set_gains       = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh, "gain_manager/set_gains");
  service_set_controller  = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh, "control_manager/switch_controller");
  service_set_tracker     = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh, "control_manager/switch_tracker");
  service_set_estimator   = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh, "estimation_manager/change_estimator");

  // | ------------------- Interactive marker ------------------- |
  // Regular marker
  Marker marker_msg;
  marker_msg.id              = 1;
  marker_msg.type            = Marker::CUBE;
  marker_msg.scale.x         = 0.45;
  marker_msg.scale.y         = 0.45;
  marker_msg.scale.z         = 0.45;
  marker_msg.color.r         = 0.5;
  marker_msg.color.g         = 0.5;
  marker_msg.color.b         = 0.5;
  marker_msg.color.a         = 0.0;  // Absolutely transparent
  marker_msg.pose.position.x = 0;
  marker_msg.pose.position.y = 0;
  marker_msg.pose.position.z = 0;

  // Control
  InteractiveMarkerControl control_msg;
  control_msg.name                           = "control1";
  control_msg.always_visible                 = true;
  control_msg.always_visible                 = true;
  control_msg.interaction_mode               = InteractiveMarkerControl::BUTTON;
  control_msg.orientation_mode               = InteractiveMarkerControl::INHERIT;
  control_msg.independent_marker_orientation = true;
  control_msg.markers.push_back(marker_msg);

  // Interactive marker
  InteractiveMarker int_marker_msg;
  int_marker_msg.name            = name + " marker";
  int_marker_msg.scale           = 1;
  int_marker_msg.header.frame_id = name + "/fcu";
  int_marker_msg.pose.position.y = 0;
  int_marker_msg.controls.push_back(control_msg);
  server->insert(int_marker_msg);

  updateMenu();
}

DroneEntity::~DroneEntity() {
  if (menu_handler != nullptr) {
    delete menu_handler;
  }
  if (server != nullptr) {
    delete server;
  }
}

bool DroneEntity::compareAndUpdate(std::vector<std::string> &current, const std::vector<std::string> &actual) {
  if (current.size() != actual.size()) {
    current = actual;
    return true;
  }

  for (int i = 0; i < current.size(); i++) {
    if (current[i] != actual[i]) {
      current = actual;
      return true;
    }
  }

  return false;
}

void DroneEntity::updateMenu() {
  // This seems to be inefficient, but must work. And since no
  // clearEntry() method is present, it's the only option
  if (menu_handler != nullptr) {
    delete menu_handler;
  }
  menu_handler = new interactive_markers::MenuHandler();

  entries.resize(EntryIndex::SIZE + custom_services.size());
  if (null_tracker) {
    entries[TAKEOFF] = menu_handler->insert("Takeoff", [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) { takeoff(feedback); });
  } else {
    entries[LAND] = menu_handler->insert("Land", [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) { land(feedback); });
    entries[LAND_HOME] =
        menu_handler->insert("Land Home", [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) { landHome(feedback); });
  }

  entries[SET_CONSTRAINT] = menu_handler->insert("Set Constraint");
  entries[SET_GAIN]       = menu_handler->insert("Set Gains");
  entries[SET_CONTROLLER] = menu_handler->insert("Set Controller");
  entries[SET_TRACKER]    = menu_handler->insert("Set Tracker");
  entries[SET_ESTIMATOR]  = menu_handler->insert("Set Estimator");

  for (const std::string &service_name : custom_service_names) {
    menu_handler->insert(service_name, [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) { processCustomService(feedback); });
  }

  for (const auto constraint : constraints) {
    menu_handler->insert(entries[SET_CONSTRAINT], constraint,
                         [this, constraint](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) { setConstraint(constraint, feedback); });
  }

  for (const auto gain : gains) {
    menu_handler->insert(entries[SET_GAIN], gain,
                         [this, gain](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) { setGain(gain, feedback); });
  }

  for (const auto controller : controllers) {
    menu_handler->insert(entries[SET_CONTROLLER], controller,
                         [this, controller](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) { setController(controller, feedback); });
  }

  for (const auto tracker : trackers) {
    menu_handler->insert(entries[SET_TRACKER], tracker,
                         [this, tracker](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) { setTracker(tracker, feedback); });
  }

  for (const auto odom_estimator : odom_estimators) {
    menu_handler->insert(entries[SET_ESTIMATOR], odom_estimator,
                         [this, odom_estimator](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) { setEstimator(odom_estimator, feedback); });
  }

  menu_handler->apply(*server, name + " marker");
  server->applyChanges();
  ROS_INFO("[Control tool]: Menu options updated");
}

void DroneEntity::statusCallback(const mrs_msgs::UavStatusConstPtr &msg) {
  bool updated = false;

  updated |= compareAndUpdate(constraints, msg->constraints);
  updated |= compareAndUpdate(gains, msg->gains);
  updated |= compareAndUpdate(controllers, msg->controllers);
  updated |= compareAndUpdate(trackers, msg->trackers);
  updated |= compareAndUpdate(odom_estimators, msg->odom_estimators);
  /* updated |= compareAndUpdate(odom_alt_sources, msg->odom_estimators_vert); */
  /* updated |= compareAndUpdate(odom_hdg_sources, msg->odom_estimators_hdg); */
  updated |= msg->null_tracker != null_tracker;
  null_tracker = msg->null_tracker;

  if (updated) {
    updateMenu();
  }
}

void DroneEntity::newSeviceCallback(const std_msgs::StringConstPtr &msg) {
  std::size_t index = msg->data.find(" ");

  if (index == std::string::npos) {
    ROS_INFO("[Control tool]: invalid add service request received: %s", msg->data.c_str());
    return;
  }

  std::string service_address = msg->data.substr(0, index);
  std::string service_name    = msg->data.substr(index + 1, std::string::npos);
  if (service_address.at(0) != '/') {
    service_address = "/" + name + "/" + service_address;
  }
  custom_services.push_back(mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh, service_address));
  custom_service_names.push_back(service_name);
  updateMenu();
  ROS_INFO("[Control tool] %s: new service \"%s\" has been added.", name.c_str(), service_name.c_str());
}

void DroneEntity::positionCmdCallback(const mrs_msgs::HwApiPositionCmdConstPtr &msg) {
  last_position = *msg;
}

// Menu callbacks
void DroneEntity::land(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  land();
}

void DroneEntity::landHome(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  landHome();
}

void DroneEntity::takeoff(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  takeoff();
}

void DroneEntity::setConstraint(const std::string &value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  setConstraint(value);
}

void DroneEntity::setGain(const std::string &value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  setGain(value);
}

void DroneEntity::setController(const std::string &value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  setController(value);
}

void DroneEntity::setTracker(const std::string &value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  setTracker(value);
}

void DroneEntity::setEstimator(const std::string &value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  setEstimator(value);
}

void DroneEntity::processCustomService(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  int service_index;
  // If Takeoff option is active
  if (null_tracker) {
    service_index = feedback->menu_entry_id - (EntryIndex::SIZE) + 1;
  }
  // If Land and Land Home options are active
  else {
    service_index = feedback->menu_entry_id - (EntryIndex::SIZE);
  }

  std_srvs::Trigger trigger;
  custom_services[service_index].call(trigger, service_num_calls, service_delay);
  ROS_INFO("[Control tool]: %s %s call processed %s. %s", name.c_str(), custom_service_names[service_index].c_str(),
           trigger.response.success ? "successfuly" : "with fail", trigger.response.message.c_str());
}


// API for ImServer
bool DroneEntity::land() {
  std_srvs::Trigger service;
  service_land.call(service, service_num_calls, service_delay);
  ROS_INFO("[Control tool]: %s land call processed %s. %s", name.c_str(), service.response.success ? "successfuly" : "with fail",
           service.response.message.c_str());
  return service.response.success;
}

bool DroneEntity::landHome() {
  std_srvs::Trigger service;
  service_land_home.call(service, service_num_calls, service_delay);
  ROS_INFO("[Control tool]: %s land_home call processed %s. %s", name.c_str(), service.response.success ? "successfuly" : "with fail",
           service.response.message.c_str());
  return service.response.success;
}

bool DroneEntity::takeoff() {
  std_srvs::Trigger service;
  service_takeoff.call(service, service_num_calls, service_delay);
  ROS_INFO("[Control tool]: %s land_home call processed %s. %s", name.c_str(), service.response.success ? "successfuly" : "with fail",
           service.response.message.c_str());
  return service.response.success;
}

bool DroneEntity::setConstraint(const std::string &value) {
  mrs_msgs::String service;
  service.request.value = value;
  service_set_constraints.call(service, service_num_calls, service_delay);
  ROS_INFO("[Control tool]: %s set_constraint call processed %s. %s", name.c_str(), service.response.success ? "successfuly" : "with fail",
           service.response.message.c_str());
  return service.response.success;
}

bool DroneEntity::setGain(const std::string &value) {
  mrs_msgs::String service;
  service.request.value = value;
  service_set_gains.call(service, service_num_calls, service_delay);
  ROS_INFO("[Control tool]: %s set_gains call processed %s. %s", name.c_str(), service.response.success ? "successfuly" : "with fail",
           service.response.message.c_str());
  return service.response.success;
}

bool DroneEntity::setController(const std::string &value) {
  mrs_msgs::String service;
  service.request.value = value;
  service_set_controller.call(service, service_num_calls, service_delay);
  ROS_INFO("[Control tool]: %s set_controller call processed %s. %s", name.c_str(), service.response.success ? "successfuly" : "with fail",
           service.response.message.c_str());
  return service.response.success;
}

bool DroneEntity::setTracker(const std::string &value) {
  mrs_msgs::String service;
  service.request.value = value;
  service_set_tracker.call(service, service_num_calls, service_delay);
  ROS_INFO("[Control tool]: %s set_tracker call processed %s. %s", name.c_str(), service.response.success ? "successfuly" : "with fail",
           service.response.message.c_str());
  return service.response.success;
}

bool DroneEntity::setEstimator(const std::string &value) {
  mrs_msgs::String service;
  service.request.value = value;
  service_set_estimator.call(service, service_num_calls, service_delay);
  ROS_INFO("[Control tool]: %s set_estimator call processed %s. %s", name.c_str(), service.response.success ? "successfuly" : "with fail",
           service.response.message.c_str());
  return service.response.success;
}

bool DroneEntity::flyForward(bool global_mode_on) {
  mrs_msgs::ReferenceStampedSrv reference;
  if (global_mode_on) {
    reference.request.header.frame_id      = last_position.header.frame_id;
    reference.request.reference.position.x = last_position.position.x + 2.0;
    reference.request.reference.position.y = last_position.position.y;
    reference.request.reference.position.z = last_position.position.z;
    reference.request.reference.heading    = last_position.heading;
  } else {
    reference.request.header.frame_id      = name + "/fcu_untilted";
    reference.request.header.stamp         = ros::Time::now();
    reference.request.reference.position.x = 2.0;
    reference.request.reference.position.y = 0.0;
    reference.request.reference.position.z = 0.0;
    reference.request.reference.heading    = 0.0;
  }
  service_goto_reference.call(reference);
  return reference.response.success;
}

bool DroneEntity::flyBackward(bool global_mode_on) {
  mrs_msgs::ReferenceStampedSrv reference;
  if (global_mode_on) {
    reference.request.header.frame_id      = last_position.header.frame_id;
    reference.request.reference.position.x = last_position.position.x - 2.0;
    reference.request.reference.position.y = last_position.position.y;
    reference.request.reference.position.z = last_position.position.z;
    reference.request.reference.heading    = last_position.heading;
  } else {
    reference.request.header.frame_id      = name + "/fcu_untilted";
    reference.request.header.stamp         = ros::Time::now();
    reference.request.reference.position.x = -2.0;
    reference.request.reference.position.y = 0.0;
    reference.request.reference.position.z = 0.0;
    reference.request.reference.heading    = 0.0;
  }
  service_goto_reference.call(reference);
  return reference.response.success;
}

bool DroneEntity::flyRight(bool global_mode_on) {
  mrs_msgs::ReferenceStampedSrv reference;
  if (global_mode_on) {
    reference.request.header.frame_id      = last_position.header.frame_id;
    reference.request.reference.position.x = last_position.position.x;
    reference.request.reference.position.y = last_position.position.y - 2.0;
    reference.request.reference.position.z = last_position.position.z;
    reference.request.reference.heading    = last_position.heading;
  } else {
    reference.request.header.frame_id      = name + "/fcu_untilted";
    reference.request.header.stamp         = ros::Time::now();
    reference.request.reference.position.x = 0.0;
    reference.request.reference.position.y = -2.0;
    reference.request.reference.position.z = 0.0;
    reference.request.reference.heading    = 0.0;
  }
  service_goto_reference.call(reference);
  return reference.response.success;
}

bool DroneEntity::flyLeft(bool global_mode_on) {
  mrs_msgs::ReferenceStampedSrv reference;
  if (global_mode_on) {
    reference.request.header.frame_id      = last_position.header.frame_id;
    reference.request.reference.position.x = last_position.position.x;
    reference.request.reference.position.y = last_position.position.y + 2.0;
    reference.request.reference.position.z = last_position.position.z;
    reference.request.reference.heading    = last_position.heading;
  } else {
    reference.request.header.frame_id      = name + "/fcu_untilted";
    reference.request.header.stamp         = ros::Time::now();
    reference.request.reference.position.x = 0.0;
    reference.request.reference.position.y = 2.0;
    reference.request.reference.position.z = 0.0;
    reference.request.reference.heading    = 0.0;
  }
  service_goto_reference.call(reference);
  return reference.response.success;
}

bool DroneEntity::flyUp() {
  mrs_msgs::ReferenceStampedSrv reference;
  reference.request.header.frame_id      = name + "/fcu_untilted";
  reference.request.header.stamp         = ros::Time::now();
  reference.request.reference.position.x = 0.0;
  reference.request.reference.position.y = 0.0;
  reference.request.reference.position.z = 1.0;
  reference.request.reference.heading    = 0.0;
  service_goto_reference.call(reference);
  return reference.response.success;
}

bool DroneEntity::flyDown() {
  mrs_msgs::ReferenceStampedSrv reference;
  reference.request.header.frame_id      = name + "/fcu_untilted";
  reference.request.header.stamp         = ros::Time::now();
  reference.request.reference.position.x = 0.0;
  reference.request.reference.position.y = 0.0;
  reference.request.reference.position.z = -1.0;
  reference.request.reference.heading    = 0.0;
  service_goto_reference.call(reference);
  return reference.response.success;
}

bool DroneEntity::rotateClockwise() {
  mrs_msgs::ReferenceStampedSrv reference;
  reference.request.header.frame_id      = name + "/fcu_untilted";
  reference.request.header.stamp         = ros::Time::now();
  reference.request.reference.position.x = 0.0;
  reference.request.reference.position.y = 0.0;
  reference.request.reference.position.z = 0.0;
  reference.request.reference.heading    = -0.5;
  service_goto_reference.call(reference);
  return reference.response.success;
}

bool DroneEntity::rotateAntiClockwise() {
  mrs_msgs::ReferenceStampedSrv reference;
  reference.request.header.frame_id      = name + "/fcu_untilted";
  reference.request.header.stamp         = ros::Time::now();
  reference.request.reference.position.x = 0.0;
  reference.request.reference.position.y = 0.0;
  reference.request.reference.position.z = 0.0;
  reference.request.reference.heading    = 0.5;
  service_goto_reference.call(reference);
  return reference.response.success;
}

std::vector<std::string> DroneEntity::getConstraints() {
  return constraints;
}

std::vector<std::string> DroneEntity::getGains() {
  return gains;
}

std::vector<std::string> DroneEntity::getControllers() {
  return controllers;
}

std::vector<std::string> DroneEntity::getTrackers() {
  return trackers;
}

std::vector<std::string> DroneEntity::getOdomEstimators() {
  return odom_estimators;
}

bool DroneEntity::getNullTracker() {
  return null_tracker;
}

void DroneEntity::setServiceNumCalls(const int value) {
  service_num_calls = value;
}

}  // namespace mrs_rviz_plugins
