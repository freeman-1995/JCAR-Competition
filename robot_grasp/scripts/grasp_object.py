#!/usr/bin/env python

import numpy as np
from robot_grasp.srv import coordinate_transform
from geometry_msgs.msg import Pose
from robot_grasp.msg import DetectedObjectsArray
from robot_grasp.msg import DetectedObject
from robot_grasp.msg import Detected_obj_param_Array
from robot_grasp.msg import Detected_obj_param
from robot_grasp.msg import tramsform_obj_param
from pcl_helper import *
from filtering_helper import *
from gazebo_msgs.srv import GetModelProperties
from gazebo_msgs.srv import GetModelState
from robot_grasp.srv import G_pick_place

#names = {\
#    'cube': 1,
#    'cone': 2,
#    'cylinder': 3,
#    'fourPrism': 4,
#    'fourPrismoid': 5,
#    'fourPyramid': 6,
#    'threePyramid': 7,
#    'sphere':8,
#    'sixPrism': 9,
#    'threePrism': 10,
#}

names = {\
    'cone': 2,
    'cylinder': 3,
    'fourPrism': 4,
    'sphere':8,
    'sixPrism': 9,
}

def transformer(Detected_objects_coordinate):
    transformer_prox = rospy.ServiceProxy('/coordinate_transform',coordinate_transform)
    return transformer_prox(Detected_objects_coordinate).tranformed_coordinate

def Call_pick_place_routine(Detected_objects_param):
    get_pick_place_routine_prox = rospy.ServiceProxy('/get_pick_place_routine',G_pick_place)
    return get_pick_place_routine_prox(Detected_objects_param).success

def pcl_callback(pcl_msg):
  cloud = ros_to_pcl(pcl_msg)
  detected_objects_labels = []
  detected_objects = []
  detected_objects_param = []
  detected_object_param = Detected_obj_param()
  get_model_properties_prox = rospy.ServiceProxy('/gazebo/get_model_properties',GetModelProperties)
  get_model_state_prox = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
  for key in names.keys():
      name_existed = get_model_properties_prox(key).success
      if name_existed != True:
          continue
      model_state = get_model_state_prox(key,'world')
      obj_p_x = model_state.pose.position.x
      if -0.35<obj_p_x and obj_p_x<0:
          continue
      obj_p_y = model_state.pose.position.y
      obj_p_z = model_state.pose.position.z
      obj_o_x = model_state.pose.orientation.x
      obj_o_y = model_state.pose.orientation.y
      obj_o_z = model_state.pose.orientation.z
      obj_o_w = model_state.pose.orientation.w
      obj_list = [obj_p_x,obj_p_y,obj_p_z,obj_o_x,obj_o_y,obj_o_z,obj_o_w]
      transformed_objects_centroid = transformer(obj_list)
      detected_object_param.num_obj_label = names[key]
      detected_object_param.pcaCentroid = transformed_objects_centroid
      print(detected_object_param)
      print("------------------------------------")
      print("pick "+key+" at: ")
      print(transformed_objects_centroid)
      pick_place_success = Call_pick_place_routine(detected_object_param)

if __name__ == '__main__':

  # Initialize ros node
  rospy.init_node('obj_grasp', anonymous = True)

  # Create Subscribers
  subscriber = rospy.Subscriber("/camera/depth_registered/points", pc2.PointCloud2, pcl_callback, queue_size = 1)

  # Initialize color_list
  get_color_list.color_list = []

  while not rospy.is_shutdown():
    rospy.spin()

