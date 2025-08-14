/**
 * This file is part of flame_ros.
 * Copyright (C) 2017 W. Nicholas Greene (wng@csail.mit.edu)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * @file textured_mesh_display.cc
 * @author W. Nicholas Greene
 * @date 2017-02-21 15:52:08 (Tue)
 */

#include "mrs_rviz_plugins/textured_mesh_display/textured_mesh_display.hpp"

#include <vector>

#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <pluginlib/class_loader.hpp>

#include <image_transport/subscriber_plugin.hpp>

#include "mrs_rviz_plugins/textured_mesh_display/textured_mesh_visual.hpp"
#include "mrs_rviz_plugins/textured_mesh_display/surface_normals_visual.hpp"

#include <message_filters/message_traits.hpp>
#include <rosidl_runtime_cpp/message_type_support_decl.hpp>

#include <rclcpp/exceptions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace mrs_rviz_plugins {

TexturedMeshDisplay::TexturedMeshDisplay() :
    rviz_common::Display(),
    mesh_filter_(nullptr),
    num_meshes_received_(0),
    tex_it_(nullptr),
    tex_filter_(nullptr),
    mesh_queue_(),
    tex_queue_(),
    visual_(nullptr),
    normals_(nullptr),
    mesh_topic_prop_("PolygonMesh Topic", "", "", "", this, SLOT(updateTopic())),
    tex_topic_prop_("Texture topic", "", "", "", this, SLOT(updateTopic())),
    tex_transport_prop_(new rviz_common::properties::EnumProperty("Texture Transport Hint", "raw",
                                               "Preferred method of sending images.",
                                               this, SLOT(updateTopic()))),
    polygon_mode_prop_("Polygon Mode", "", "Polygon rendering mode.",
                       this, SLOT(updatePolygonMode())),
    shader_program_prop_("Shader Program", "", "Shader program for mesh.",
                   this, SLOT(updateShaderProgram())),
    phong_shading_prop_("Phong Shading", true, "Use Phong Shading.", this,
                        SLOT(updatePhongShading())),
    scene_color_scale_prop_("Scene Color Scale", 1.0f, "Color scale for shaders.",
                            this, SLOT(updateSceneColorScale())),
    show_normals_prop_("Show Normals", false, "Show normal vectors.", this,
                       SLOT(updateShowNormals())),
    normal_size_prop_("Normal Size", 0.05f, "Size of normal vector lines.",
                      this, SLOT(updateNormalSize())),
    queue_size_prop_("Queue Size", 25,
                     "Advanced: set the size of the incoming message queue.  Increasing this "
                     "is useful if your incoming TF data is delayed significantly from your"
                     " image data, but it can greatly increase memory usage if the messages are big.",
                     this, SLOT(updateQueueSize())),
    transport_plugin_types_(),
    mtx_() {
  /* Add polygon mode enums. */
  polygon_mode_prop_.addOptionStd("POINTS",
                                  static_cast<int>(Ogre::PM_POINTS));
  polygon_mode_prop_.addOptionStd("WIREFRAME",
                                  static_cast<int>(Ogre::PM_WIREFRAME));
  polygon_mode_prop_.addOptionStd("SOLID",
                                  static_cast<int>(Ogre::PM_SOLID));

  // Add shader enums.
  shader_program_prop_.addOptionStd("TEXTURE",
                              static_cast<int>(TexturedMeshVisual::ShaderProgram::TEXTURE));
  shader_program_prop_.addOptionStd("INVERSE_DEPTH",
                              static_cast<int>(TexturedMeshVisual::ShaderProgram::INVERSE_DEPTH));
  shader_program_prop_.addOptionStd("JET",
                                    static_cast<int>(TexturedMeshVisual::ShaderProgram::JET));
  shader_program_prop_.addOptionStd("SURFACE_NORMAL",
                              static_cast<int>(TexturedMeshVisual::ShaderProgram::SURFACE_NORMAL));

  // Scene color scale property.
  scene_color_scale_prop_.setMin(0.0f);

  // Texture transport property.
  // connect(tex_transport_prop_.get(), SIGNAL(requestOptions(EnumProperty*)),
  //         this, SLOT(fillTransportOptionList(EnumProperty*)));
  // connect(tex_transport_prop_.get(), SIGNAL(aboutToShowOptions()),
  //       this, SLOT(fillTransportOptionList()));
  // connect(tex_transport_prop_.get(), SIGNAL(changed()),
  //       this, SLOT(fillTransportOptionList()));

  tex_transport_prop_->addOptionStd("raw");
  tex_transport_prop_->addOptionStd("compressed");
  tex_transport_prop_->addOptionStd("theora");
  tex_transport_prop_->setStdString("raw");

  // Queue size property
  queue_size_prop_.setMin(1);
  queue_size_ = queue_size_prop_.getInt();

  return;
}

void TexturedMeshDisplay::onInitialize() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  // Get the node from the RViz context
  ros_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  tex_it_.reset(new image_transport::ImageTransport(ros_node));

  // Initialize the topic properties with the ROS node
  auto rviz_ros_node = context_->getRosNodeAbstraction();
  mesh_topic_prop_.initialize(rviz_ros_node);
  tex_topic_prop_.initialize(rviz_ros_node);

  // Set message types with static strings (more reliable)
  mesh_topic_prop_.setMessageType("pcl_msgs/msg/PolygonMesh");
  mesh_topic_prop_.setDescription("pcl_msgs/msg/PolygonMesh topic to subscribe to.");

  tex_topic_prop_.setMessageType("sensor_msgs/msg/Image");
  tex_topic_prop_.setDescription("sensor_msgs/msg/Image topic to subscribe to.");

  // Scan for available transport plugins
  scanForTransportSubscriberPlugins();

  return;
}

TexturedMeshDisplay::~TexturedMeshDisplay() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  unsubscribe();
  return;
}

void TexturedMeshDisplay::reset() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  Display::reset();
  /*
    This causes the plugin to crash when switching the topics.
    That is caused by recreating TexturedMeshVisual when old OGRE resources still exist.
    So the easiest solution is to just not reset it and it won't be recreated with
    old resources being reused.
  */
  // visual_.reset();
  // normals_.reset();

  return;
}

void TexturedMeshDisplay::updateTopic() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
  return;
}

void TexturedMeshDisplay::updateQueueSize() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  queue_size_ = queue_size_prop_.getInt();
  return;
}

void TexturedMeshDisplay::updatePolygonMode() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (visual_ != nullptr) {
    Ogre::PolygonMode pm =
        static_cast<Ogre::PolygonMode>(polygon_mode_prop_.getOptionInt());
    visual_->setPolygonMode(pm);
  }
  return;
}

void TexturedMeshDisplay::updateShaderProgram() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (visual_ != nullptr) {
    TexturedMeshVisual::ShaderProgram sp =
        static_cast<TexturedMeshVisual::ShaderProgram>(shader_program_prop_.getOptionInt());
    visual_->setShaderProgram(sp);
  }
  return;
}

void TexturedMeshDisplay::updatePhongShading() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (visual_ != nullptr) {
    visual_->setPhongShading(phong_shading_prop_.getBool());
  }
  return;
}

void TexturedMeshDisplay::updateSceneColorScale() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (visual_ != nullptr) {
    visual_->setSceneColorScale(scene_color_scale_prop_.getFloat());
  }
  return;
}

void TexturedMeshDisplay::updateShowNormals() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (normals_ != nullptr) {
    normals_->setVisible(show_normals_prop_.getBool());
  }
  return;
}

void TexturedMeshDisplay::updateNormalSize() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  if (normals_ != nullptr) {
    normals_->setNormalSize(normal_size_prop_.getFloat());
  }
  return;
}

/*
  I left this here in case dynamic topics exploration will be needed in the future...
*/
// void TexturedMeshDisplay::fillTransportOptionList() {
//   std::lock_guard<std::recursive_mutex> lock(mtx_);

//   // Get the property that triggered this
//   EnumProperty* property = tex_transport_prop_.get();
  
//   property->clearOptions();

//   std::vector<std::string> choices;
//   choices.push_back("raw");
  
//   // Add common image transport types
//   if (transport_plugin_types_.find("compressed") != transport_plugin_types_.end()) {
//     choices.push_back("compressed");
//   }
//   if (transport_plugin_types_.find("theora") != transport_plugin_types_.end()) {
//     choices.push_back("theora");
//   }

//   for (size_t ii = 0; ii < choices.size(); ii++) {
//     property->addOptionStd(choices[ii]);
//   }

//   return;
// }

void TexturedMeshDisplay::scanForTransportSubscriberPlugins() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  pluginlib::ClassLoader<image_transport::SubscriberPlugin>
      sub_loader("image_transport", "image_transport::SubscriberPlugin");

  BOOST_FOREACH(const std::string& lookup_name, sub_loader.getDeclaredClasses()) {
    // lookup_name is formatted as "pkg/transport_sub", for instance
    // "image_transport/compressed_sub" for the "compressed"
    // transport.  This code removes the "_sub" from the tail and
    // everything up to and including the "/" from the head, leaving
    // "compressed" (for example) in transport_name.
    std::string transport_name = lookup_name;
    
    // Remove "_sub" suffix if it exists
    const std::string suffix = "_sub";
    if (transport_name.size() >= suffix.size() && 
        transport_name.substr(transport_name.size() - suffix.size()) == suffix) {
      transport_name = transport_name.substr(0, transport_name.size() - suffix.size());
    }
    
    // Remove everything up to and including the "/"
    size_t slash_pos = transport_name.find('/');
    if (slash_pos != std::string::npos) {
      transport_name = transport_name.substr(slash_pos + 1);
    }

    // If the plugin loads without throwing an exception, add its
    // transport name to the list of valid plugins, otherwise ignore
    // it.
    try {
      std::shared_ptr<image_transport::SubscriberPlugin> sub = sub_loader.createSharedInstance(lookup_name);
      transport_plugin_types_.insert(transport_name);
    } catch (const pluginlib::LibraryLoadException& e) {}
    catch (const pluginlib::CreateClassException& e) {}
  }

  return;
}

void TexturedMeshDisplay::subscribe() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  if (!isEnabled()) {
    return;
  }

  try {
    mesh_filter_.reset(new message_filters::Subscriber<pcl_msgs::msg::PolygonMesh>());
    tex_filter_.reset(new image_transport::SubscriberFilter());

    std::string mesh_topic = mesh_topic_prop_.getTopicStd();
    std::string tex_topic = tex_topic_prop_.getTopicStd();

    std::string tex_transport = tex_transport_prop_->getStdString();

    if (!mesh_topic.empty()) {
      // Subscribe to the mesh topic.
      mesh_filter_->subscribe(ros_node, mesh_topic, rmw_qos_profile_default);
      mesh_filter_->registerCallback(
          std::bind(&TexturedMeshDisplay::processPolygonMeshMessage, this, std::placeholders::_1));

      if (!tex_topic.empty() && !tex_transport.empty()) {
        // Subscribe to texture topic.
        tex_filter_->subscribe(ros_node.get(), tex_topic, tex_transport, rmw_qos_profile_default);
        tex_filter_->registerCallback(
            std::bind(&TexturedMeshDisplay::processTextureMessage, this, std::placeholders::_1));
      }
    }

    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");

  } catch (const rclcpp::exceptions::RCLError& e) {
    RCLCPP_DEBUG(ros_node->get_logger(), "Error subscribing: %s", e.what());
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
              QString("Error subscribing: ") + e.what());
  } catch (const rclcpp::exceptions::InvalidTopicNameError& e) {
    RCLCPP_DEBUG(ros_node->get_logger(), "Error subscribing: %s", e.what());
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
              QString("Error subscribing: ") + e.what());
  } catch (const std::runtime_error& e) {
    RCLCPP_DEBUG(ros_node->get_logger(), "Error subscribing: %s", e.what());
    setStatus(rviz_common::properties::StatusProperty::Error, "Message",
              QString("Error subscribing: ") + e.what());
  } catch (const std::exception& e) {
    RCLCPP_DEBUG(ros_node->get_logger(), "Error subscribing: %s", e.what());
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
              QString("Error subscribing: ") + e.what());
  } catch (...) {
    RCLCPP_DEBUG(ros_node->get_logger(), "Caught unknown exception!");
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
              QString("Unknown error occurred while subscribing"));
  }

  return;
}

void TexturedMeshDisplay::unsubscribe() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  try {
    mesh_filter_.reset();
    tex_filter_.reset();
  } catch (const std::exception& e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Message",
              QString("Error unsubscribing: ") + e.what());
  }

  return;
}

void TexturedMeshDisplay::onEnable() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  subscribe();
  return;
}

void TexturedMeshDisplay::onDisable() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);
  unsubscribe();
  reset();
  return;
}

void TexturedMeshDisplay::fixedFrameChanged() {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  // reset();

  return;
}

void TexturedMeshDisplay::
processTextureMessage(const sensor_msgs::msg::Image::ConstSharedPtr& tex_msg) {
  double timestamp = tex_msg->header.stamp.sec + tex_msg->header.stamp.nanosec * 1e-9;
  RCLCPP_DEBUG(ros_node->get_logger(), "Got a texture message at %f!\n", timestamp);
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  if (!tex_msg) {
    return;
  }

  // Push message onto queue.
  tex_queue_.push(tex_msg);

  // Enforce queue size.
  while (tex_queue_.size() > queue_size_) {
    tex_queue_.pop();
  }

  RCLCPP_DEBUG(ros_node->get_logger(), "Processed a texture message!\n");

  return;
}

void TexturedMeshDisplay::
processPolygonMeshMessage(const pcl_msgs::msg::PolygonMesh::ConstSharedPtr& msg) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  RCLCPP_DEBUG(ros_node->get_logger(), "Got a mesh message at %f!\n", timestamp);

  if (!msg) {
    return;
  }

  // Push message onto queue.
  mesh_queue_.push(msg);

  // Enforce queue size.
  while (mesh_queue_.size() > queue_size_) {
    mesh_queue_.pop();
  }

  // Synchronize and process the texture and mesh messages.
  pcl_msgs::msg::PolygonMesh::ConstSharedPtr mesh_msg;
  sensor_msgs::msg::Image::ConstSharedPtr tex_msg;
  //double tol = 5e-3; // 5 ms tolerance.
  double tol = 10;
  while ((tex_queue_.size() > 0) && (mesh_queue_.size() > 0)) {
    double tex_time = tex_queue_.front()->header.stamp.sec + 10e-9 * tex_queue_.front()->header.stamp.nanosec;
    double mesh_time = mesh_queue_.front()->header.stamp.sec + 10e-9 * mesh_queue_.front()->header.stamp.nanosec;
    RCLCPP_DEBUG(ros_node->get_logger(), "tex_time: %f", tex_time);
    RCLCPP_DEBUG(ros_node->get_logger(), "mesh_time: %f", mesh_time);

    RCLCPP_DEBUG(ros_node->get_logger(), "tex_time - mesh_time: %f", std::fabs(tex_time - mesh_time));
    if (std::fabs(tex_time - mesh_time) <= tol) {
      RCLCPP_DEBUG(ros_node->get_logger(), "in tolerance, processing");
      mesh_msg = mesh_queue_.front();
      tex_msg = tex_queue_.front();
      mesh_queue_.pop();
      tex_queue_.pop();
      break;
    } else {
      RCLCPP_DEBUG(ros_node->get_logger(), "out of tolernace");
      if (tex_time < mesh_time) {
        RCLCPP_DEBUG(ros_node->get_logger(), "tex_queue_.pop()");
        tex_queue_.pop();
      } else {
        RCLCPP_DEBUG(ros_node->get_logger(), "mesh_queue_.pop()");
        mesh_queue_.pop();
      }
    }
  }

  if ((mesh_msg != nullptr) && (tex_msg != nullptr)) {
    RCLCPP_DEBUG(ros_node->get_logger(), "Found a match!");
    processTexturedMeshMessages(mesh_msg, tex_msg);
  }

  RCLCPP_DEBUG(ros_node->get_logger(), "Processed a mesh message!\n");

  return;
}

void TexturedMeshDisplay::
processTexturedMeshMessages(const pcl_msgs::msg::PolygonMesh::ConstSharedPtr& mesh_msg,
                            const sensor_msgs::msg::Image::ConstSharedPtr& tex_msg) {
  RCLCPP_DEBUG(ros_node->get_logger(), "Got mesh and texture messages!\n");
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  if (!mesh_msg || !tex_msg) {
    return;
  }

  updateFixedFrameTransform(mesh_msg->header);

  num_meshes_received_++;
  setStatus(rviz_common::properties::StatusProperty::Ok, "Topic",
            QString::number(num_meshes_received_) + " messages received");

  if (visual_ == nullptr) {
    Ogre::PolygonMode pm =
        static_cast<Ogre::PolygonMode>(polygon_mode_prop_.getOptionInt());
    TexturedMeshVisual::ShaderProgram sp =
        static_cast<TexturedMeshVisual::ShaderProgram>(shader_program_prop_.getOptionInt());

    visual_ = std::make_shared<TexturedMeshVisual>(scene_manager_,
                                                   getSceneNode(), ros_node, pm, sp);
    visual_->setSceneColorScale(scene_color_scale_prop_.getFloat());
  }

  visual_->setFromMessage(mesh_msg, tex_msg);

  if (normals_ == nullptr) {
    normals_ = std::make_shared<SurfaceNormalsVisual>(scene_manager_,
                                                      getSceneNode(),
                                                      Ogre::ColourValue::Red,
                                                      normal_size_prop_.getFloat());
  }

  normals_->setFromMessage(mesh_msg);

  RCLCPP_DEBUG(ros_node->get_logger(), "Processed a texture and mesh messages!\n");

  return;
}

void TexturedMeshDisplay::
updateFixedFrameTransform(const std_msgs::msg::Header& header) {
  std::lock_guard<std::recursive_mutex> lock(mtx_);

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this message.
  // The fixed frame is the frame being displayed in RVIZ (e.g. FLU world).
  // The frame in the msg header should be the camera RDF world frame.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(header.frame_id,
                                                 header.stamp,
                                                 position, orientation)) {
    RCLCPP_DEBUG(ros_node->get_logger(), "Error transforming from frame '%s' to frame '%s'",
             header.frame_id.c_str(), context_->getFixedFrame().toStdString().c_str());
    return;
  }

  getSceneNode()->setPosition(position);
  getSceneNode()->setOrientation(orientation);

  return;
}

}  // namespace mrs_rviz_plugins

// Tell pluginlib about this class. It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_rviz_plugins::TexturedMeshDisplay, rviz_common::Display)
