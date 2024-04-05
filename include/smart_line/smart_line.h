 #ifndef SMART_LINE_H_
 #define SMART_LINE_H_
 
#include <rviz/ogre_helpers/line.h>

 namespace rviz
 {
 class SmartLine : public Line
 {
 public:
   using Line::Line; //inherited constructor
   void setColorUnshaded(const Ogre::ColourValue& c);

 };
 
 } // namespace rviz
 
 #endif /* SMART_LINE_H_ */
