/* Note: this version compiles, but does  not make any callbacks.
  Even ros::spinOnce does not help. The only callback that seems to work is reset.
  rqt_graph says that rviz is subscribed to required topics. The same is implied from the fact that
  reset callback is calles only when interactiveMarkerServer is running */



#include "control/client_wrapper.h"


#include <rviz/frame_manager.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/validate_floats.h>
#include <rviz/validate_quaternions.h>

namespace mrs_rviz_plugins{

bool validateFloats(const visualization_msgs::InteractiveMarker& msg)
{
  bool valid = true;
  valid = valid && rviz::validateFloats(msg.pose);
  valid = valid && rviz::validateFloats(msg.scale);
  for (unsigned c = 0; c < msg.controls.size(); c++)
  {
    valid = valid && rviz::validateFloats(msg.controls[c].orientation);
    for (unsigned m = 0; m < msg.controls[c].markers.size(); m++)
    {
      valid = valid && rviz::validateFloats(msg.controls[c].markers[m].pose);
      valid = valid && rviz::validateFloats(msg.controls[c].markers[m].scale);
      valid = valid && rviz::validateFloats(msg.controls[c].markers[m].color);
      valid = valid && rviz::validateFloats(msg.controls[c].markers[m].points);
    }
  }
  return valid;
}

bool validateQuaternions(const visualization_msgs::InteractiveMarker& marker)
{
  if (!rviz::validateQuaternions(marker.pose.orientation))
    return false;
  for (size_t c = 0; c < marker.controls.size(); ++c)
  {
    if (!rviz::validateQuaternions(marker.controls[c].orientation))
      return false;
    for (size_t m = 0; m < marker.controls[c].markers.size(); ++m)
    {
      if (!rviz::validateQuaternions(marker.controls[c].markers[m].pose.orientation))
        return false;
    }
  }
  return true;
}

ClientWrapper::ClientWrapper(rviz::DisplayContext* context, rviz::StatusList* status) {
    // TODO: smth might be required to be added
    context_ = context;
    status_ = status;
    ROS_INFO("getting frame ...");
    ROS_INFO("%p", context_);
    fixed_frame_ = context_->getFixedFrame().toStdString();
    ROS_INFO("getting root node ..");
    root_scene_node_ = context_->getSceneManager()->getRootSceneNode();
}

void ClientWrapper::onInitialize()
{
  auto tf = context_->getFrameManager()->getTF2BufferPtr();
  im_client_.reset(new interactive_markers::InteractiveMarkerClient(*tf, fixed_frame_));

  im_client_->setInitCb(boost::bind(&ClientWrapper::initCb, this, boost::placeholders::_1));
  im_client_->setUpdateCb(
      boost::bind(&ClientWrapper::updateCb, this, boost::placeholders::_1));
  im_client_->setResetCb(boost::bind(&ClientWrapper::resetCb, this, boost::placeholders::_1));
  im_client_->setStatusCb(boost::bind(&ClientWrapper::statusCb, this, boost::placeholders::_1,
                                      boost::placeholders::_2, boost::placeholders::_3));

  // todo: make an alternative to getName()
  // client_id_ = ros::this_node::getName() + "/" + getNameStd();
  client_id_ = ros::this_node::getName() + "/client";

  enable();
}

void ClientWrapper::enable()
{
  is_enabled = true;
  subscribe();
}

void ClientWrapper::disable()
{
  is_enabled = false;
  unsubscribe();
}

void ClientWrapper::setShowAxes(bool value) {
  show_axes_ = value;
}

void ClientWrapper::setShowVisualAids(bool value) {
  show_visual_aids_ = value;
}

void ClientWrapper::setShowDescriptions(bool value) {
  show_descriptions_ = value;
}

void ClientWrapper::onFixedFrameChanged(){
  fixed_frame_ = context_->getFixedFrame().toStdString();
  if (im_client_){
    im_client_->setTargetFrame(fixed_frame_);
  }
  reset();
}

void ClientWrapper::subscribe()
{
  if (is_enabled)
  {
    ROS_INFO("subscribing... %s ", topic_ns_.c_str());
    im_client_->subscribe(topic_ns_);

    std::string feedback_topic = topic_ns_ + "/feedback";
    feedback_pub_ =
        update_nh_.advertise<visualization_msgs::InteractiveMarkerFeedback>(feedback_topic, 100, false);
  }
}

void ClientWrapper::publishFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback)
{
  feedback.client_id = client_id_;
  feedback_pub_.publish(feedback);
}

void ClientWrapper::onStatusUpdate(rviz::StatusProperty::Level level, const std::string& name, const std::string& text)
{
  ROS_INFO("Status update: %s, %s", name.c_str(), text.c_str());
  setStatusStd(level, name, text);
}

void ClientWrapper::unsubscribe()
{
  if (im_client_)
  {
    ROS_INFO("shutting client down");
    im_client_->shutdown();
  }
  ROS_INFO("publisher off ...");
  feedback_pub_.shutdown();
  ROS_INFO("Resetting....");
  if(status_){
    status_->clear();
  }
}

void ClientWrapper::update(float wall_dt, float /*ros_dt*/)
{
  ros::spinOnce();
  im_client_->update();


  M_StringToStringToIMPtr::iterator server_it;
  for (server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++)
  {
    M_StringToIMPtr::iterator im_it;
    for (im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++)
    {
      im_it->second->update(wall_dt);
    }
  }
}

ClientWrapper::M_StringToIMPtr&
ClientWrapper::getImMap(const std::string& server_id)
{
  M_StringToStringToIMPtr::iterator im_map_it = interactive_markers_.find(server_id);

  if (im_map_it == interactive_markers_.end())
  {
    im_map_it = interactive_markers_.insert(std::make_pair(server_id, M_StringToIMPtr())).first;
  }

  return im_map_it->second;
}

void ClientWrapper::updateMarkers(
    const std::string& server_id,
    const std::vector<visualization_msgs::InteractiveMarker>& markers)
{
  M_StringToIMPtr& im_map = getImMap(server_id);

  for (size_t i = 0; i < markers.size(); i++)
  {
    const visualization_msgs::InteractiveMarker& marker = markers[i];

    if (!validateFloats(marker))
    {
      ROS_ERROR("[CLIENT WRAPPER: Marker contains invalid floats]");
      setStatusStd(rviz::StatusProperty::Error, marker.name, "Marker contains invalid floats!");
      setStatusStd(rviz::StatusProperty::Error, "General", "Marker " + marker.name + " contains invalid floats!" );
      continue;
    }

    if (!validateQuaternions(marker))
    {
      ROS_WARN_ONCE_NAMED("quaternions",
                          "Interactive marker '%s' contains unnormalized quaternions. "
                          "This warning will only be output once but may be true for others; "
                          "enable DEBUG messages for ros.rviz.quaternions to see more details.",
                          marker.name.c_str());
      ROS_DEBUG_NAMED("quaternions", "Interactive marker '%s' contains unnormalized quaternions.",
                      marker.name.c_str());
    }
    ROS_DEBUG("Processing interactive marker '%s'. %d", marker.name.c_str(), (int)marker.controls.size());

    std::map<std::string, IMPtr>::iterator int_marker_entry = im_map.find(marker.name);

    if (int_marker_entry == im_map.end())
    {
      int_marker_entry =
          im_map
              .insert(std::make_pair(marker.name, IMPtr(new rviz::InteractiveMarker(root_scene_node_, context_))))
              .first;
      connect(int_marker_entry->second.get(), &rviz::InteractiveMarker::userFeedback, this,
              &ClientWrapper::publishFeedback);
      connect(int_marker_entry->second.get(), &rviz::InteractiveMarker::statusUpdate, this,
              &ClientWrapper::onStatusUpdate);
    }

    if (int_marker_entry->second->processMessage(marker))
    {
      int_marker_entry->second->setShowAxes(show_axes_);
      int_marker_entry->second->setShowVisualAids(show_visual_aids_);
      int_marker_entry->second->setShowDescription(show_descriptions_);
    }
    else
    {
      unsubscribe();
      return;
    }
  }
}

void ClientWrapper::eraseMarkers(const std::string& server_id,
                                            const std::vector<std::string>& erases)
{
  M_StringToIMPtr& im_map = getImMap(server_id);

  for (size_t i = 0; i < erases.size(); i++)
  {
    im_map.erase(erases[i]);
    // deleteStatusStd(erases[i]);
  }
}

void ClientWrapper::setStatusStd(rviz::StatusProperty::Level level, const std::string &name, const std::string &text){
  status_->setStatus(level, QString(name.c_str()), QString(text.c_str()));
}

void ClientWrapper::updatePoses(
    const std::string& server_id,
    const std::vector<visualization_msgs::InteractiveMarkerPose>& marker_poses)
{
  M_StringToIMPtr& im_map = getImMap(server_id);

  for (size_t i = 0; i < marker_poses.size(); i++)
  {
    const visualization_msgs::InteractiveMarkerPose& marker_pose = marker_poses[i];

    if (!rviz::validateFloats(marker_pose.pose))
    {
      ROS_ERROR("Pose message contains invalid floats!");
      setStatusStd(rviz::StatusProperty::Error, marker_pose.name, "Pose message contains invalid floats!");
      return;
    }

    if (!rviz::validateQuaternions(marker_pose.pose))
    {
      ROS_ERROR("Pose message contains invalid quaternions (length not equal to 1)!");
      setStatusStd(rviz::StatusProperty::Error, marker_pose.name,
                   "Pose message contains invalid quaternions (length not equal to 1)!");
      return;
    }

    std::map<std::string, IMPtr>::iterator int_marker_entry = im_map.find(marker_pose.name);

    if (int_marker_entry != im_map.end())
    {
      int_marker_entry->second->processMessage(marker_pose);
    }
    else
    {
      ROS_ERROR("Pose received for non-existing marker");
      setStatusStd(rviz::StatusProperty::Error, marker_pose.name,
                   "Pose received for non-existing marker '" + marker_pose.name);
      unsubscribe();
      return;
    }
  }
}

void ClientWrapper::setTopic(std::string topic){
  ROS_INFO("unsubscribing...");
  unsubscribe();

  size_t idx = topic.find("/update");
  if (idx != std::string::npos)
  {
    ROS_INFO("found");
    topic_ns_ = topic.substr(0, idx);
    subscribe();
  }
  else
  {
    ROS_ERROR("Invalid topic name: %s", topic.c_str());
    setStatusStd(rviz::StatusProperty::Error, "Topic", "Invalid topic name: " + topic);
  }
}

void ClientWrapper::initCb(const visualization_msgs::InteractiveMarkerInitConstPtr& msg)
{ 
  ROS_INFO("init called");
  resetCb(msg->server_id);
  updateMarkers(msg->server_id, msg->markers);
}

void ClientWrapper::updateCb(const visualization_msgs::InteractiveMarkerUpdateConstPtr& msg)
{
  ROS_INFO("update called");
  updateMarkers(msg->server_id, msg->markers);
  updatePoses(msg->server_id, msg->poses);
  eraseMarkers(msg->server_id, msg->erases);
}

void ClientWrapper::resetCb(const std::string& server_id)
{
  ROS_INFO("reset called");
  interactive_markers_.erase(server_id);
  status_->deleteStatus(QString(server_id.c_str()));
  // deleteStatusStd(server_id);
}

void ClientWrapper::statusCb(interactive_markers::InteractiveMarkerClient::StatusT status,
                                        const std::string& server_id,
                                        const std::string& msg)
{
  // ROS_INFO("here: %s: %s", server_id.c_str(), msg.c_str());
  setStatusStd(static_cast<rviz::StatusProperty::Level>(status), server_id, msg);
}

void ClientWrapper::reset()
{
  // Display::reset();
  unsubscribe();
  subscribe();
}


} // namespace mrs_rviz_plugins