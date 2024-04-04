 #include "fast_arrow/fast_arrow.h"

#include <rviz/ogre_helpers/shape.h>
 
 #include <OgreSceneManager.h>
 #include <OgreSceneNode.h>
 #include <OgreVector3.h>
 #include <OgreQuaternion.h>
 
 #include <sstream>
 
 namespace rviz
 {
 FastArrow::FastArrow(Ogre::SceneManager* scene_manager,
              Ogre::SceneNode* parent_node,
              float shaft_length,
              float head_length,
              float head_diameter)
   : Object(scene_manager)
 {
   if (!parent_node)
   {
     parent_node = scene_manager_->getRootSceneNode();
   }
 
   scene_node_ = parent_node->createChildSceneNode();
 
   shaft_   = new rviz::SmartLine(scene_manager_, scene_node_);
   head_l_  = new rviz::SmartLine(scene_manager_, scene_node_);
   head_r_  = new rviz::SmartLine(scene_manager_, scene_node_);

   set(shaft_length, head_length, head_diameter);
 
   setOrientation(Ogre::Quaternion::IDENTITY);
 }
 
 FastArrow::~FastArrow()
 {
   delete shaft_;
   delete head_l_;
   delete head_r_;
 
   scene_manager_->destroySceneNode(scene_node_->getName());
 }
 
 void FastArrow::set(float shaft_length, float head_length, float head_diameter)
 {
   shaft_->setPoints(Ogre::Vector3(0,0,0), Ogre::Vector3(0, shaft_length, 0));
   head_l_->setPoints(Ogre::Vector3(0,shaft_length,0), Ogre::Vector3(0, shaft_length-head_length,  head_diameter/2));
   head_r_->setPoints(Ogre::Vector3(0,shaft_length,0), Ogre::Vector3(0, shaft_length-head_length, -head_diameter/2));
   /* shaft_->setPosition(Ogre::Vector3(0.0f, shaft_length / 2.0f, 0.0f)); */
 
   /* head_->setScale(Ogre::Vector3(head_diameter, head_length, head_diameter)); */
   /* head_->setPosition(Ogre::Vector3(0.0f, shaft_length, 0.0f)); */
 }
 
 void FastArrow::setColor(const Ogre::ColourValue& c)
 {
   shaft_->setColorUnshaded(c);
   /* shaft_->manual_object_material_->getTechique(0)->setAmbient(1,1,1); */
   head_l_->setColorUnshaded(c);
   /* head_l_->manual_object_material_->getTechique(0)->setAmbient(1,1,1); */
   head_r_->setColorUnshaded(c);
   /* head_r_->manual_object_material_->getTechique(0)->setAmbient(1,1,1); */
 }
 
 void FastArrow::setColor(float r, float g, float b, float a)
 {
   setColor(Ogre::ColourValue(r, g, b, a));
 }
 
 void FastArrow::setPosition(const Ogre::Vector3& position)
 {
   scene_node_->setPosition(position);
 }
 
 void FastArrow::setOrientation(const Ogre::Quaternion& orientation)
 {
   // "forward" (negative z) should always be our identity orientation
   // ... wouldn't need to mangle the orientation if we just fix the cylinders!
   scene_node_->setOrientation(orientation * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_X));
 }
 
 void FastArrow::setDirection(const Ogre::Vector3& direction)
 {
   if (!direction.isZeroLength())
   {
     setOrientation(Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(direction));
   }
 }
 
 void FastArrow::setScale(const Ogre::Vector3& scale)
 {
   // Have to mangle the scale because of the default orientation of the cylinders :(
   scene_node_->setScale(Ogre::Vector3(scale.z, scale.x, scale.y));
 }
 
 const Ogre::Vector3& FastArrow::getPosition()
 {
   return scene_node_->getPosition();
 }
 
 const Ogre::Quaternion& FastArrow::getOrientation()
 {
   return scene_node_->getOrientation();
 }

 rviz::SmartLine* FastArrow::getShaft(){
   return shaft_;
 }
 rviz::SmartLine* FastArrow::getHeadL(){
   return head_l_;
 }
 rviz::SmartLine* FastArrow::getHeadR(){
   return head_l_;
 }
 
 void FastArrow::setUserData(const Ogre::Any& data)
 {
   shaft_->setUserData(data);
   head_l_->setUserData(data);
   head_r_->setUserData(data);
 }
 
 } // namespace rviz
