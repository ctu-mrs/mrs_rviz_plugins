#!/usr/bin/env python  
import rospy
import numpy as np
import random

from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo

def add_line(ptlist, pt1, pt2):
    pt = Point()
    pt.x = pt1[0]
    pt.y = pt1[1]
    pt.z = pt1[2]
    ptlist.append(pt)

    pt = Point()
    pt.x = pt2[0]
    pt.y = pt2[1]
    pt.z = pt2[2]
    ptlist.append(pt)

def generate_fov_marker(frame_id, fov_l, fov_h, fov_v, r, g, b, a, thickness):
    msg = Marker()

    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.ns = "camera_fov"
    msg.id = 0
    msg.type = Marker.LINE_LIST
    msg.action = Marker.ADD
    msg.pose.position.x = 0.0
    msg.pose.position.y = 0.0
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    msg.scale.x = thickness
    msg.scale.y = 0.0
    msg.scale.z = 0.0
    msg.color.r = r
    msg.color.g = g
    msg.color.b = b
    msg.color.a = a
    msg.frame_locked = True

    fov_h2 = fov_h/2.0
    fov_v2 = fov_v/2.0
    x = fov_l*np.tan(fov_h2)
    y = fov_l*np.tan(fov_v2)
    z = fov_l

    add_line(msg.points, (0, 0, 0), (x, y, z))
    add_line(msg.points, (0, 0, 0), (-x, y, z))
    add_line(msg.points, (0, 0, 0), (-x, -y, z))
    add_line(msg.points, (0, 0, 0), (x, -y, z))

    add_line(msg.points, (x, y, z), (-x, y, z))
    add_line(msg.points, (-x, y, z), (-x, -y, z))
    add_line(msg.points, (-x, -y, z), (x, -y, z))
    add_line(msg.points, (x, -y, z), (x, y, z))

    return msg

def calc_fov(camera_info):
    fx = camera_info.P[0]
    fy = camera_info.P[5]
    w = camera_info.width
    h = camera_info.height

    fov_h = 2*np.arctan2(w, 2*fx)
    fov_v = 2*np.arctan2(h, 2*fy)
    return (fov_h, fov_v)

cinfo = None
def cinfo_callback(msg):
    global cinfo
    cinfo = msg
    rospy.loginfo_once("Received camera info")

if __name__ == '__main__':
    rospy.init_node('camera_fov_marker_{:9d}'.format(random.randint(0, 1e9)))

    length = rospy.get_param("~length")
    thickness = rospy.get_param("~line_thickness")

    r = rospy.get_param("~color/r")
    g = rospy.get_param("~color/g")
    b = rospy.get_param("~color/b")
    a = rospy.get_param("~color/a")

    rospy.Subscriber("~camera_info", CameraInfo, cinfo_callback)
    pub = rospy.Publisher("~camera_fov_marker", Marker, queue_size=10)

    msg = None
    rate = 1
    sleeper = rospy.Rate(rate)
    while not rospy.is_shutdown():
        if msg is None:
            if cinfo is None:
                sleeper.sleep()
                rospy.loginfo_once("Waiting for camera info at topic '{}'".format(rospy.resolve_name("~camera_info")))
                continue
            else:
                frame_id = cinfo.header.frame_id
                fov_h, fov_v = calc_fov(cinfo)
                rospy.loginfo_once("Calculated FOV: {:.2f}x{:.2f} degrees".format(fov_v/np.pi*180, fov_h/np.pi*180))
                msg = generate_fov_marker(frame_id, length, fov_h, fov_v, r, g, b, a, thickness)

        pub.publish(msg)
        sleeper.sleep()

