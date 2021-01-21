<launch>

  <arg name="uav_name" default="$(optenv UAV_NAME uav)"/>
  <arg name="uav_type" default="$(optenv UAV_TYPE f550)"/>
  <arg name="base_link" default="$(arg uav_name)/fcu"/>

  <group ns="$(arg uav_name)">
    <param name="robot_model" command="$(find mrs_rviz_plugins)/scripts/generate_robot_model_xml.py $(find mrs_rviz_plugins)/data/$(arg uav_type).xml $(arg base_link) $(find mrs_rviz_plugins)" />
    <node name="tf_published_uav_marker_link" pkg="tf2_ros" type="static_transform_publisher" args="0 0 0 0 0 0 $(arg base_link) $(arg base_link)/uav_marker" />
    <node name="tf_published_props_link" pkg="tf2_ros" type="static_transform_publisher" args="0 0 0 0 0 0 $(arg base_link) $(arg base_link)/props" />
    <node name="tf_published_arms_link" pkg="tf2_ros" type="static_transform_publisher" args="0 0 0 0 0 0 $(arg base_link) $(arg base_link)/arms" />
    <node name="tf_published_arms_red_link" pkg="tf2_ros" type="static_transform_publisher" args="0 0 0 0 0 0 $(arg base_link) $(arg base_link)/arms_red" />
  </group>

</launch>

