#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <robot_grasp/Centroid_Compute.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_listener.h>

std::vector<float> centroid_param(3,0);

class centroid_computer
{
public:
  explicit centroid_computer(ros::NodeHandle nh)
    : nh_(nh),tfListener(tfBuffer)
  {
    get_centroid_srv_ = nh_.advertiseService("get_centroid", &centroid_computer::getCentroidReq, this);
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceServer get_centroid_srv_;
  geometry_msgs::PointStamped centroid_cam_pt;
  geometry_msgs::TransformStamped transformStamped;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  bool getCentroidReq(robot_grasp::Centroid_Compute::Request &req, robot_grasp::Centroid_Compute::Response &rsp)
  {
    pcl::PointCloud<pcl::PointXYZ> *p_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    Eigen::Vector4f pcaCentroid;
    pcl::fromROSMsg(req.cluster, *p_cloud);
    pcl::compute3DCentroid(*p_cloud, pcaCentroid);

    centroid_cam_pt.point.x = pcaCentroid(0);
    centroid_cam_pt.point.y = pcaCentroid(1);
    centroid_cam_pt.point.z = pcaCentroid(2);
    centroid_cam_pt.header.frame_id = "/camera_rgb_optical_frame";
    try
    {
        // Transformation Matrix from the camera_rgb_optical_frame frame to the robot_base_link frame
        transformStamped = tfBuffer.lookupTransform("j2s7s300_link_base", "camera_rgb_optical_frame", ros::Time(0),ros::Duration(3.0));
        //创建robot_base_link下的质心与法向量坐标点
        geometry_msgs::PointStamped  centroid_robot_base_pt;
        centroid_robot_base_pt.header.frame_id = "/j2s7s300_link_base";
        //将质心与法向量从camera_rgb_optical_frame转化到robot_base_link
        tf2::doTransform(centroid_cam_pt, centroid_robot_base_pt, transformStamped);
        //定义centroid_robot_base用来存储机器人坐标系下的质心坐标点
        centroid_param[0] = centroid_robot_base_pt.point.x;
        centroid_param[1] = centroid_robot_base_pt.point.y;
        centroid_param[2] = centroid_robot_base_pt.point.z;
        (rsp.Centroid_param) = centroid_param;
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "centroid_computer");
  ros::NodeHandle nh("~");

  centroid_computer cc(nh);
  while (ros::ok())
    ros::spin();

  return 0;
}
