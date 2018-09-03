#!/usr/bin/env python
import rospy

from visualization_msgs.msg import Marker

def make_label(text, position, id = 0 ,duration = 5.0, color=[1.0,1.0,1.0]):
    marker = Marker()
    marker.header.frame_id = 'camera_link'
    marker.id = id
    marker.type = marker.TEXT_VIEW_FACING
    marker.text = text
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.lifetime = rospy.Duration(duration)
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    return marker
