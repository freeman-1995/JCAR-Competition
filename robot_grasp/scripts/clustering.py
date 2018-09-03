#!/usr/bin/env python

from pcl_helper import *
from filtering_helper import *

def split_cloud(cloud):
  downsampled_cloud = do_voxel_grid_filter(point_cloud = cloud, LEAF_SIZE = 0.002)
  #filtered_cloud = do_passthrough_filter(point_cloud = downsampled_cloud, 
                                         #name_axis = 'z', min_axis = 0.6, max_axis = 1.1)
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

def pcl_callback(pcl_msg):
  cloud = ros_to_pcl(pcl_msg) 
  objects_cloud, table_cloud = split_cloud(cloud) 
  colorless_cloud = XYZRGB_to_XYZ(objects_cloud)
  clusters = get_clusters(colorless_cloud, tolerance = 0.009, min_size = 10, max_size = 5000)
  colored_points = get_colored_clusters(clusters, colorless_cloud)
  clusters_cloud = pcl.PointCloud_PointXYZRGB()
  clusters_cloud.from_list(colored_points)
  objects_msg = pcl_to_ros(objects_cloud)
  table_msg = pcl_to_ros(table_cloud)
  clusters_msg = pcl_to_ros(clusters_cloud)
  objects_publisher.publish(objects_msg)
  table_publisher.publish(table_msg)
  clusters_publisher.publish(clusters_msg)
 

if __name__ == '__main__':
  rospy.init_node('clustering', anonymous = True)
  subscriber = rospy.Subscriber("/camera/depth_registered/points", pc2.PointCloud2, pcl_callback, queue_size = 1)
  objects_publisher = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
  table_publisher = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
  clusters_publisher = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)
  get_color_list.color_list = []
  while not rospy.is_shutdown():
    rospy.spin()
