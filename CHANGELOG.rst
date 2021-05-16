^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrs_rviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2021-05-16)
------------------
* updated CI
* Contributors: Tomas Baca

1.0.0 (2021-03-18)
------------------
* Major release

0.0.2 (2021-03-16)
------------------
* Noetic-compatible
* added ignore flag for the  storage class deprecation warnings due to OGRE
* fixes in PoseWithCovarianceArrayStamped display
* + Sphere
* added missing TF for colorful arms of UAV robot model in Rviz
* updated to python3
* Fixed the orientation of covariance view to match with the interpretation of the covariance in e.g. geometry_msgs::PoseWithCovariance used by tf2 (`#2 <https://github.com/ctu-mrs/mrs_rviz_plugins/issues/2>`_)
* + BatchVisualizer
* + f450 and t650 vehicle types
* + FOV Marker
* added vertical threshold to bumper visualization
* Contributors: Matouš Vrba, Petr Stibinger, Petr Štibinger, Tomas Baca, Tomáš Báča, Viktor Walter, ViktorWalter, Vojtech Spurny, klaxalk, mergify[bot], stibipet

* add other models, vehicle selection follows env UAV_TYPE
* fixed camera FOV script to not crash with time moving backwards
* automated the camera FOV marker code a bit
* first version of FOV marker ready
* added vertical threshold to bumper visualization
* Contributors: Matouš Vrba, Tomas Baca

0.0.1 (2019-05-20)
------------------
