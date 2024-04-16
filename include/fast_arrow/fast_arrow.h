#include <rviz/ogre_helpers/object.h>
#include "smart_line/smart_line.h"
 
 #ifndef FAST_ARROW_H
 #define FAST_ARROW_H
 
 namespace Ogre
 {
 class SceneManager;
 class SceneNode;
 class Vector3;
 class Quaternion;
 class ColourValue;
 class Any;
 } // namespace Ogre
 
 namespace rviz
 {
 class Shape;
 
 class FastArrow : public Object
 {
 public:
   FastArrow(Ogre::SceneManager* scene_manager,
         Ogre::SceneNode* parent_node = nullptr,
         float shaft_length = 1.0f,
         float head_length = 0.3f,
         float head_diameter = 0.2f);
   ~FastArrow() override;
 
   void set(float shaft_length, float head_length, float head_diameter);
 
   void setColor(float r, float g, float b, float a) override;
   void setColor(const Ogre::ColourValue& color);
 
   void setOrientation(const Ogre::Quaternion& orientation) override;
 
   void setPosition(const Ogre::Vector3& position) override;
 
   void setDirection(const Ogre::Vector3& direction);
 
   void setScale(const Ogre::Vector3& scale) override;
   const Ogre::Vector3& getPosition() override;
   const Ogre::Quaternion& getOrientation() override;
 
   Ogre::SceneNode* getSceneNode()
   {
     return scene_node_;
   }

   rviz::SmartLine* getShaft();
   rviz::SmartLine* getHeadL();
   rviz::SmartLine* getHeadR();
 
   void setUserData(const Ogre::Any& data) override;
 
 private:
   Ogre::SceneNode* scene_node_;
 
   rviz::SmartLine* shaft_;
   rviz::SmartLine* head_l_;
   rviz::SmartLine* head_r_;
 };
 
 } // namespace rviz
 
 #endif /* FAST_ARROW_H */
