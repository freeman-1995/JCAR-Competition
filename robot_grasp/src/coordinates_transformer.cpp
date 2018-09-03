#include "ros/ros.h"
#include "robot_grasp/coordinate_transform.h"
#include "robot_grasp/tramsform_obj_param.h"
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_listener.h>

std::vector<float> centroid_param(3,0);
bool transform(robot_grasp::coordinate_transform::Request &req,
         robot_grasp::coordinate_transform::Response &res)
{
  geometry_msgs::PointStamped centroid_cam_pt;
  geometry_msgs::TransformStamped transformStamped;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  centroid_cam_pt.point.x = req.transform_coordinate[0];
  centroid_cam_pt.point.y = req.transform_coordinate[1];
  centroid_cam_pt.point.z = req.transform_coordinate[2];
  centroid_cam_pt.header.frame_id = "/world";
  try
  {
      // Transformation Matrix from the camera_rgb_optical_frame frame to the robot_base_link frame
      transformStamped = tfBuffer.lookupTransform("j2s7s300_link_base", "world", ros::Time(0),ros::Duration(3.0));
      //创建robot_base_link下的质心与法向量坐标点
      geometry_msgs::PointStamped  centroid_robot_base_pt;
      centroid_robot_base_pt.header.frame_id = "/j2s7s300_link_base";
      //将质心与法向量从camera_rgb_optical_frame转化到robot_base_link
      tf2::doTransform(centroid_cam_pt, centroid_robot_base_pt, transformStamped);
      //定义centroid_robot_base用来存储机器人坐标系下的质心坐标点
      centroid_param[0] = centroid_robot_base_pt.point.x;
      centroid_param[1] = centroid_robot_base_pt.point.y;
      centroid_param[2] = centroid_robot_base_pt.point.z;
      centroid_param[3] = req.transform_coordinate[7];
      res.tranformed_coordinate = centroid_param;
  }
  catch (tf2::TransformException &ex)
  {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinates_transformer");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("/coordinate_transform", &transform);
  ros::spin();

  return 0;
}
