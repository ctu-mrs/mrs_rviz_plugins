#include "control/control.h"

#include <visualization_msgs/InteractiveMarker.h>

#include <rviz/visualization_manager.h>



namespace mrs_rviz_plugins{

ControlTool::ControlTool() : rviz::SelectionTool(){
  shortcut_key_ = 'c';
  server = new ImServer();
}

ControlTool::~ControlTool(){
}

void ControlTool::onInitialize(){
  rviz::SelectionTool::onInitialize();

  rviz::InteractiveMarkerDisplay* dis = new rviz::InteractiveMarkerDisplay();
  dynamic_cast<rviz::VisualizationManager*>(context_)->addDisplay(dis, true);

  dis->setName(QString("Control Display"));
  dis->setTopic(QString("control/update"), QString("visualization_msgs/InteractiveMarkerUpdate"));

  // TODO: /mrs_drone_spawner/diagnostics seems to have that info and it is more efficient
  // Note: it may also be put in timer for example, check a new drone every 3 secs
  // Preparing for searching the drone's name
  XmlRpc::XmlRpcValue      req = "/node";
  XmlRpc::XmlRpcValue      res;
  XmlRpc::XmlRpcValue      pay;
  std::vector<std::string> drone_names;
  ros::master::execute("getSystemState", req, res, pay, true);

  // Search for the drone's name
  std::string state[res[2][2].size()];
  for (int x = 0; x < res[2][2].size(); x++) {
    std::string name = res[2][2][x][0].toXml().c_str();
    if (name.find("trajectory_generation/path") == std::string::npos) {
      continue;
    }
    ROS_INFO("[Control tool]: %s found", name.c_str());

    std::size_t index = name.find("/", 0, 1);
    if (index != std::string::npos) {
      name = name.erase(0, index + 1);
    }

    index = name.find("/", 1, 1);
    if (index != std::string::npos) {
      name = name.erase(index);
    }

    drone_names.push_back(name);
    ROS_INFO("[Control tool]: %s was added to drone names", name.c_str());
    state[x] = name;
  }

  // Initialize all the markers
  for(auto name : drone_names){
    server->addDrone(name);
  }
}

void ControlTool::activate(){
  rviz::SelectionTool::activate();

}

void ControlTool::deactivate(){
  rviz::SelectionTool::deactivate();

}

// TODO: what to write here?
// The method is taken from interaction_tool.cpp  
void ControlTool::updateFocus(const rviz::ViewportMouseEvent& event) {
  rviz::M_Picked results;
  // Pick exactly 1 pixel
  context_->getSelectionManager()->pick(event.viewport, event.x, event.y, event.x + 1, event.y + 1,
                                        results, true);

  last_selection_frame_count_ = context_->getFrameCount();

  rviz::InteractiveObjectPtr new_focused_object;

  // look for a valid handle in the result.
  rviz::M_Picked::iterator result_it = results.begin();
  if (result_it != results.end())
  {
    rviz::Picked pick = result_it->second;
    rviz::SelectionHandler* handler = context_->getSelectionManager()->getHandler(pick.handle);
    if (pick.pixel_count > 0 && handler)
    {
      rviz::InteractiveObjectPtr object = handler->getInteractiveObject().lock();
      if (object && object->isInteractive())
      {
        new_focused_object = object;
      }
    }
  }

  // If the mouse has gone from one object to another, defocus the old
  // and focus the new.
  rviz::InteractiveObjectPtr new_obj = new_focused_object;
  rviz::InteractiveObjectPtr old_obj = focused_object_.lock();
  if (new_obj != old_obj)
  {
    // Only copy the event contents here, once we know we need to use
    // a modified version of it.
    rviz::ViewportMouseEvent event_copy = event;
    if (old_obj)
    {
      event_copy.type = QEvent::FocusOut;
      old_obj->handleMouseEvent(event_copy);
    }

    if (new_obj)
    {
      event_copy.type = QEvent::FocusIn;
      new_obj->handleMouseEvent(event_copy);
    }
  }

  focused_object_ = new_focused_object;
}


int ControlTool::processMouseEvent(rviz::ViewportMouseEvent& event){
  int flags = rviz::SelectionTool::processMouseEvent(event);

  // TODO: what to write here?
  // The method is taken from interaction_tool.cpp
  if (event.panel->contextMenuVisible())
  {
    return flags;
  }

  // make sure we let the vis. manager render at least one frame between selection updates
  bool need_selection_update = context_->getFrameCount() > last_selection_frame_count_;

  // We are dragging if a button was down and is still down
  Qt::MouseButtons buttons = event.buttons_down & (Qt::LeftButton | Qt::RightButton | Qt::MidButton);
  if (event.type == QEvent::MouseButtonPress){
    buttons &= ~event.acting_button;
  }
  bool dragging = buttons != 0;

  // unless we're dragging, check if there's a new object under the mouse
  if (need_selection_update && !dragging && event.type != QEvent::MouseButtonRelease)
  {
    updateFocus(event);
    flags = Render;
  }

  // If alt is pressed, interaction is disabled, so finish
  if(event.alt()){
    return flags;
  }

  {
    rviz::InteractiveObjectPtr focused_object = focused_object_.lock();
    if (focused_object)
    {
      focused_object->handleMouseEvent(event);
      setCursor(focused_object->getCursor());
    }
  }

  if (event.type == QEvent::MouseButtonRelease)
  {
    updateFocus(event);
  }

  return flags;
}

int ControlTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel){
  int res = rviz::SelectionTool::processKeyEvent(event, panel);

  // Menu
  if(event->key() == KEY_M){
    processMKey();
  }

  return res;
}

void ControlTool::processMKey(){
  ROS_INFO("M key received");
  std::vector<std::string> marker_names{};

  rviz::M_Picked picked = context_->getSelectionManager()->getSelection();
  
  // Find all selected markers
  for (auto picked_it :  picked){
    rviz::Picked pick = picked_it.second;
    rviz::SelectionHandler* handler = context_->getSelectionManager()->getHandler(pick.handle);
    if (!(pick.pixel_count > 0 && handler)){
      continue;
    }

    rviz::InteractiveObjectPtr object = handler->getInteractiveObject().lock();
    if (!(object && object->isInteractive())){
      continue;
    }
    
    auto int_mar_con_ptr = boost::dynamic_pointer_cast<rviz::InteractiveMarkerControl>(object);
    if(!int_mar_con_ptr){
      continue;
    }

    rviz::InteractiveMarker* int_mar = int_mar_con_ptr->getParent();
    if(!int_mar){
      continue;
    }

    marker_names.push_back(int_mar->getName());
  }
  // Make menu
  rviz::RenderPanel* render_panel = dynamic_cast<rviz::VisualizationManager*>(context_)->getRenderPanel();
  render_panel->showContextMenu(server->getMenu(marker_names));
  


}

void ControlTool::some_action(){
  ROS_INFO("Action called");
}

}// namespace mrs_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::ControlTool, rviz::Tool)