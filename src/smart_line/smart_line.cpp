 #include "smart_line/smart_line.h"
 
 #include <sstream>
 
 #include <OgreSceneNode.h>
 #include <OgreSceneManager.h>
 #include <OgreManualObject.h>
 #include <OgreMaterialManager.h>
 #include <OgreTechnique.h>
 
 namespace rviz
 {
     void SmartLine::setColorUnshaded(const Ogre::ColourValue& c){
       /* manual_object_material_->setLightingEnabled(false); */
       manual_object_material_->setAmbient(c);
       manual_object_material_->setDiffuse(c);
       manual_object_material_->setSelfIllumination(c);
       manual_object_material_->setShadingMode(Ogre::SO_FLAT);

       if (c.a < 0.9998)
       {
         manual_object_material_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
         manual_object_material_->getTechnique(0)->setDepthWriteEnabled(false);
       }
       else
       {
         manual_object_material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
         manual_object_material_->getTechnique(0)->setDepthWriteEnabled(true);
       }
     }
 
 } // namespace rviz
