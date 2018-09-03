#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from robot_grasp.srv import GetNormals
from robot_grasp.srv import GetVfh
from robot_grasp.srv import Centroid_Compute
from robot_grasp.srv import VFH_pick_pace
from features import compute_color_histograms
from features import compute_normal_histograms
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from marker_tools import *
from robot_grasp.msg import DetectedObjectsArray
from robot_grasp.msg import DetectedObject
from robot_grasp.msg import Detected_obj_param_Array
from robot_grasp.msg import Detected_obj_param
from pcl_helper import *
from filtering_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

def get_vfh(cloud):
    get_vfh_prox = rospy.ServiceProxy('/feature_extractor/get_vfh', GetVfh)
    return get_vfh_prox(cloud).VFH_data

def get_centroid_(cloud):
    get_centroid_prox = rospy.ServiceProxy('/centroid_computer/get_centroid', Centroid_Compute)
    return get_centroid_prox(cloud).Centroid_param

def get_pick_place_routine(Detected_objects_param):
    get_pick_place_routine_prox = rospy.ServiceProxy('/get_pick_place_routine',PickPlace)
    return get_pick_place_routine_prox(Detected_objects_param).success

def split_cloud(cloud):
  downsampled_cloud = do_voxel_grid_filter(point_cloud = cloud, LEAF_SIZE = 0.002)
  table_cloud, objects_cloud = do_ransac_plane_segmentation(downsampled_cloud, max_distance = 0.012)
  return objects_cloud, table_cloud

def get_clusters(cloud, tolerance, min_size, max_size):

  tree = cloud.make_kdtree()
  extraction_object = cloud.make_EuclideanClusterExtraction()
  extraction_object.set_ClusterTolerance(tolerance)
  extraction_object.set_MinClusterSize(min_size)
  extraction_object.set_MaxClusterSize(max_size)
  extraction_object.set_SearchMethod(tree)
  clusters = extraction_object.Extract()
  return clusters
  
def get_colored_clusters(clusters, cloud):
  number_of_clusters = len(clusters)
  colors = get_color_list(number_of_clusters)

  colored_points = []
  for cluster_id, cluster in enumerate(clusters):
    for c, i in enumerate(cluster):
      x, y, z = cloud[i][0], cloud[i][1], cloud[i][2]
      color = rgb_to_float(colors[cluster_id])
      colored_points.append([x, y, z, color])
  return colored_points

def get_obj_property(obj):
    get_model_state_prox_1 = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
    model_state = get_model_state_prox(obj,'world')

    obj_p_x = model_state.pose.position.x
    obj_p_y = model_state.pose.position.y
    obj_p_z = model_state.pose.position.z
    obj_o_x = model_state.pose.orientation.x
    obj_o_y = model_state.pose.orientation.y
    obj_o_z = model_state.pose.orientation.z
    obj_o_w = model_state.pose.orientation.w
    obj_list = [obj,obj_p_x,obj_p_y,obj_p_z]
    return obj_list

def pcl_callback(pcl_msg):
  cloud = ros_to_pcl(pcl_msg)
  objects_cloud, table_cloud = split_cloud(cloud) 

  pick_pose = Pose()
  place_pose = Pose()
  pick_pose.orientation.x = 0.893997
  pick_pose.orientation.y = 0
  pick_pose.orientation.z = 0
  pick_pose.orientation.w = -0.448074

  place_pose.position.x = 0.2
  place_pose.position.y = 0.2
  place_pose.position.z = 1.2
  place_pose.orientation.x = 0
  place_pose.orientation.y = 0
  place_pose.orientation.z = 0
  place_pose.orientation.w = 0

  colorless_cloud = XYZRGB_to_XYZ(objects_cloud)
  clusters = get_clusters(colorless_cloud, tolerance = 0.009, min_size = 10, max_size = 5000)
  colored_points = get_colorobj_real_detected_in_camed_clusters(clusters, colorless_cloud)
  clusters_cloud = pcl.PointCloud_PointXYZRGB()
  clusters_cloud.from_list(colored_points)
  clusters_msg = pcl_to_ros(clusters_cloud)
  #clusters_publisher.publish(clusters_msg)
  detected_objects_labels = []
  detected_objects = []
  detected_objects_param = []
  print(str(len(clusters))+'cobj_real_detected_in_camlusters found');
  for i, indices in enumerate(clusters):
    cluster = objects_cloud.extract(indices)
    cluster_msg = pcl_to_ros(cluster)
    detected_objects_centroid = get_centroid_(cluster_msg)  
    #print(detected_objects_centroid)
    vfhs = get_vfh(cluster_msg)
    features = np.array(vfhs)
    prediction = classifier.predict(scaler.transform(features.reshape(1, -1)))
    label = encoder.inverse_transform(prediction)[0]
    #detected_objects_labels.append(label)
    #label_position = list(colorless_cloud[indices[0]])
    #label_position[2] += 0.4
    #object_markers_publisher.publish(make_label(label, label_position, i))

    #detectedObject = DetectedObject()
    #detectedObject.label = label
    #detectedObject.cloud = pcl_to_ros(clusters_cloud)
    #detected_objects.append(detectedObject)

    detected_obj_param = Detected_obj_param()
    detected_obj_param.obj_label = label
    detected_obj_param.pcaCentroid = detected_objects_centroid
    detected_objects_param.append(detected_obj_param)
  #rospy.sleep(5.0)
  #rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
  #rospy.loginfo('position {}'.format(detected_objects_param))
  #detected_objects_publisher.publish(detected_objects)
  #detected_objects_publisher.publish(detected_objects_param)

  #START Moveit Routine
  pick_place_success = get_pick_place_routine(detected_objects_param)
  print('The end of grasp!')

if __name__ == '__main__':

  # Initialize ros node
  rospy.init_node('object_markers_pub', anonymous = True)
  
  # Create Subscribers
  subscriber = rospy.Subscriber("/camera/depth_registered/points", pc2.PointCloud2, pcl_callback, queue_size = 1)

  # Create Publishers
  object_markers_publisher = rospy.Publisher("/object_markers", Marker, queue_size = 1)
  detected_objects_publisher = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size = 1)
  detected_objects_param_publisher = rospy.Publisher("/detected_objects_param", Detected_obj_param_Array, queue_size = 1)
  clusters_publisher = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)
  # Load Model From disk
  model = pickle.load(open('model.sav', 'rb'))
  classifier = model['classifier']
  encoder = LabelEncoder()
  encoder.classes_ = model['classes']
  scaler = model['scaler']

  # Initialize color_list
  get_color_list.color_list = []

  while not rospy.is_shutdown():
    rospy.spin()

