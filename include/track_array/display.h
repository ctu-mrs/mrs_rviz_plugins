#pragma once

#include <string> 

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>

#include <mrs_msgs/Track.h>
#include <mrs_msgs/TrackArrayStamped.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include <rviz/message_filter_display.h>
#include <rviz/selection/forwards.h>
#include <rviz/default_plugin/covariance_property.h>
#include <rviz/default_plugin/covariance_visual.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreVector3.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/validate_floats.h>
#include <rviz/validate_quaternions.h>
#include <rviz/validate_floats.h>
#include <rviz/validate_quaternions.h>
#include <rviz/selection/selection_handler.h>
#include <rviz/msg_conversions.h>


namespace rviz
{
class Arrow;
class Axes;
class ColorProperty;
class EnumProperty;
class FloatProperty;
class BoolProperty;
class Shape;
}  // namespace rviz

namespace mrs_rviz_plugins
{
namespace track_array
{
    class TextID;

    struct display_object
    {
        //position and orientation:
        boost::shared_ptr<rviz::Axes> axes_pose_;
        boost::shared_ptr<rviz::CovarianceVisual> covariance_pose_;

        //velocity:
        boost::shared_ptr<rviz::Arrow> arrow_vel_;
        float arrow_vel_len_; //real length of velocity arrow without scaling
        boost::shared_ptr<rviz::CovarianceVisual> covariance_vel_;

        //marker:
        boost::shared_ptr<TextID> text_id_;
    };

    class DisplaySelectionHandler;

    class Display : public rviz::MessageFilterDisplay<mrs_msgs::TrackArrayStamped> {
        Q_OBJECT
    public:

        Display();
        virtual ~Display();

        virtual void onInitialize();
        virtual void reset();

    protected:
        /** @brief Overridden from MessageFilterDisplay to get Arrow/Axes visibility correct. */
        virtual void onEnable();

    private Q_SLOTS:
        void updateCovariancePoseVisibility();
        void updateCovarianceVelocityVisibility();

        void updateVelocityArrowVisibility();
        void updateVelocityColorAndAlpha();
        void updateVelocityArrowGeometry();

        void updateAxesVisibility();
        void updateAxesGeometry();

        void updateTextIDVisibility();
        void updateTextIDColorAndAlpha();
        void updateTextIDSize();
        void updateTextIDShift();

    private:
        void clear();

        virtual void processMessage(const mrs_msgs::TrackArrayStamped::ConstPtr& message);

        std::vector<display_object> disp_data_; //all tracked objects

        std::unique_ptr<DisplaySelectionHandler> coll_handler_;

        bool pose_valid_;
        bool velocity_valid_;

        std::unique_ptr<rviz::BoolProperty> velocity_arrow_bool_property_;
        std::unique_ptr<rviz::ColorProperty> velocity_color_property_;
        std::unique_ptr<rviz::FloatProperty> velocity_arrow_alpha_property_;
        std::unique_ptr<rviz::FloatProperty> velocity_arrow_radius_property_;
        std::unique_ptr<rviz::FloatProperty> velocity_arrow_length_scale_property_;
        std::unique_ptr<rviz::FloatProperty> velocity_arrow_head_radius_property_;
        std::unique_ptr<rviz::FloatProperty> velocity_arrow_head_length_property_;

        std::unique_ptr<rviz::CovarianceProperty> velocity_covariance_property_;
        std::unique_ptr<rviz::FloatProperty> velocity_covariance_alpha_;

        std::unique_ptr<rviz::BoolProperty> axes_bool_property_;
        std::unique_ptr<rviz::FloatProperty> axes_length_property_;
        std::unique_ptr<rviz::FloatProperty> axes_radius_property_;

        std::unique_ptr<rviz::CovarianceProperty> pose_covariance_property_;

        std::unique_ptr<rviz::BoolProperty> id_text_bool_property_;
        std::unique_ptr<rviz::ColorProperty> id_text_color_property_;
        std::unique_ptr<rviz::FloatProperty> id_text_alpha_property_;
        std::unique_ptr<rviz::FloatProperty> id_text_size_property_;
        std::unique_ptr<rviz::FloatProperty> id_text_shift_x_property_;
        std::unique_ptr<rviz::FloatProperty> id_text_shift_y_property_;
        std::unique_ptr<rviz::FloatProperty> id_text_shift_z_property_;

        friend class DisplaySelectionHandler;   
    };

    /* Helper class to show text markers with id for each tracked object */
    class TextID {
        public:
            TextID(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
            virtual ~TextID();

            void setPosition(Ogre::Vector3& position, float x, float y, float z);
            void setSize(const float size);
            void setColor(const Ogre::ColourValue color);
            void setText(const std::string& text, Ogre::ColourValue color, float size);

            Ogre::SceneNode* getSceneNode();
            Ogre::Vector3 getPosition();

        protected:
            Ogre::SceneManager* scene_manager_;
            Ogre::SceneNode* scene_node_;

            Ogre::Vector3 position_;

            rviz::MovableText* text_; 
    };

}  // namespace track_array

}  // namespace mrs_rviz_plugins