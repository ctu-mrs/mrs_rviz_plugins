#ifndef COVARIANCE_PROPERTY_H
#define COVARIANCE_PROPERTY_H

#include <QColor>

#include <OgreColourValue.h>

#include <rviz/properties/bool_property.h>

#include <covariance/visual.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace rviz
{
class ColorProperty;
class FloatProperty;
class EnumProperty;
}  // end namespace rviz

namespace mrs_rviz_plugins
{

namespace covariance
{

class Property : public rviz::BoolProperty {
  Q_OBJECT
public:
  typedef boost::shared_ptr<Visual> VisualPtr;

  enum Frame
  {
    Local,
    Fixed,
  };

  enum ColorStyle
  {
    Unique,
    RGB,
  };

  Property(const QString& name = "Covariance", bool default_value = false, const QString& description = QString(), rviz::Property* parent = 0,
           const char* changed_slot = 0, QObject* receiver = 0);

  virtual ~Property();

  bool getPositionBool();
  bool getOrientationBool();

  // Methods to manage the deque of Covariance Visuals
  VisualPtr createAndPushBackVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  void      popFrontVisual();
  void      clearVisual();
  size_t    sizeVisual();

public Q_SLOTS:
  void updateVisibility();

private Q_SLOTS:
  void updateColorAndAlphaAndScaleAndOffset();
  void updateOrientationFrame();
  void updateColorStyleChoice();

private:
  void updateColorAndAlphaAndScaleAndOffset(const VisualPtr& visual);
  void updateOrientationFrame(const VisualPtr& visual);
  void updateVisibility(const VisualPtr& visual);

  typedef std::deque<VisualPtr> D_Covariance;
  D_Covariance                  covariances_;

  rviz::BoolProperty*  position_property_;
  rviz::ColorProperty* position_color_property_;
  rviz::FloatProperty* position_alpha_property_;
  rviz::FloatProperty* position_scale_property_;
  rviz::BoolProperty*  orientation_property_;
  rviz::EnumProperty*  orientation_frame_property_;
  rviz::EnumProperty*  orientation_colorstyle_property_;
  rviz::ColorProperty* orientation_color_property_;
  rviz::FloatProperty* orientation_alpha_property_;
  rviz::FloatProperty* orientation_offset_property_;
  rviz::FloatProperty* orientation_scale_property_;
};

}  // namespace covariance

}  // namespace mrs_rviz_plugins

#endif  // COVARIANCE_PROPERTY_H
