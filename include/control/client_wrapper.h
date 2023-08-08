#ifndef CLIENT_WRAPPER_H
#define CLIENT_WRAPPER_H

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/InteractiveMarkerInit.h>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <interactive_markers/interactive_marker_client.h>
#endif

#include <rviz/display_context.h>
#include <rviz/properties/status_list.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/bool_property.h>

#include <rviz/default_plugin/interactive_markers/interactive_marker.h>

#include <map>
#include <set>

namespace mrs_rviz_plugins{

class ClientWrapper : public QObject{
  Q_OBJECT
public:
  ClientWrapper(rviz::DisplayContext* context, rviz::StatusList* status);

  void onInitialize();

  void update(float wall_dt, float ros_dt);

  void reset();

  // Note: topic must end with "/update"
  void setTopic(std::string topic);

  void enable();
  
  void disable();

  void setShowAxes(bool value);

  void setShowVisualAids(bool value);

  void setShowDescriptions(bool value);

  // Note: meant to be Q_SLOTS. It is here temorarily for debug
  void onFixedFrameChanged();

protected Q_SLOTS:
  void publishFeedback(visualization_msgs::InteractiveMarkerFeedback& feedback);
  void onStatusUpdate(rviz::StatusProperty::Level level, const std::string& name, const std::string& text);

private:
  // Subscribe to all message topics
  void subscribe();

  // Unsubscribe from all message topics
  void unsubscribe();

  void initCb(const visualization_msgs::InteractiveMarkerInitConstPtr& msg);
  void updateCb(const visualization_msgs::InteractiveMarkerUpdateConstPtr& msg);

  void resetCb(const std::string& server_id);

  void statusCb(interactive_markers::InteractiveMarkerClient::StatusT /*status*/,
                const std::string& server_id,
                const std::string& msg);

  void updateMarkers(const std::string& server_id,
                     const std::vector<visualization_msgs::InteractiveMarker>& markers);

  void updatePoses(const std::string& server_id,
                   const std::vector<visualization_msgs::InteractiveMarkerPose>& marker_poses);

  void eraseMarkers(const std::string& server_id, const std::vector<std::string>& names);

  void setStatusStd(rviz::StatusProperty::Level level, const std::string &name, const std::string &text);

  // Update the display's versions of the markers.
  void processMarkerChanges(const std::vector<visualization_msgs::InteractiveMarker>* markers = nullptr,
                            const std::vector<visualization_msgs::InteractiveMarkerPose>* poses = nullptr,
                            const std::vector<std::string>* erases = nullptr);
  // Aliases
  typedef boost::shared_ptr<rviz::InteractiveMarker> IMPtr;
  typedef std::map<std::string, IMPtr> M_StringToIMPtr;
  typedef std::map<std::string, M_StringToIMPtr> M_StringToStringToIMPtr;

  M_StringToIMPtr& getImMap(const std::string& server_id);

  // |-------------------------- ATRIBUTES --------------------------|
  M_StringToStringToIMPtr interactive_markers_;   // Map server_id : marker_name : interactive marker

  boost::shared_ptr<interactive_markers::InteractiveMarkerClient> im_client_;

  std::string client_id_;

  ros::Publisher feedback_pub_;

  std::string topic_ns_;

  rviz::DisplayContext* context_;
  rviz::StatusList* status_;

  ros::NodeHandle update_nh_;

  Ogre::SceneNode* root_scene_node_;

  // |------------------------ CURRENT STATE ------------------------|
  bool is_enabled = false;

  bool show_axes_ = false;
  bool show_visual_aids_ = false;
  bool show_descriptions_ = false;

  std::string fixed_frame_;
}; // class ClientWrapper

}// namespace mrs_rviz_plugins

#endif