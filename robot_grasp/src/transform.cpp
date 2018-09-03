#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/console.h>
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
#include "robot_grasp/calibration_transform.h"

class calib_trans
{
public:
  explicit calib_trans(ros::NodeHandle nh)
    : nh_(nh)
  {
    trans_service = nh_.advertiseService("transform", &calib_trans::transform, this);
    Intrinsic_matrix<<1.068627748400805, 0, 0.947266362,
                      0, 1.0692998463589911, 0.5775551951048131,
                      0, 0, 1.0;
    Extrinsic_matrix<<0.03308510442216442, -0.9990111993421, 0.02969844289965112, -0.4792858102453171,
  -0.999440603164699, -0.03321500484350666, -0.003890988412249358, -0.6272767204865666,
  0.004873575509995856, -0.02955308484901648, -0.9995512790596954, 0.7123777245156747,
  0, 0, 0, 1;
    cx=947.27;
    cy=577.55;
    fx=1068.63;
    fy=1069.30;
    depth=660;
    std::cout<<Extrinsic_matrix<<std::endl;
    std::cout<<"-------------------"<<std::endl;
    std::cout<<Intrinsic_matrix<<std::endl;
  }
public:
  ros::NodeHandle nh_;
  ros::ServiceServer trans_service;
  Eigen::Matrix3d Intrinsic_matrix;
  Eigen::Matrix4d Extrinsic_matrix;
  Eigen::Vector4d projection_point,grasp_point;

  float depth;
  float cx;
  float cy;
  float fx;
  float fy;
  float xm, ym; //image co
  float x1, x2, x3;  //projected point

  bool transform(robot_grasp::calibration_transform::Request &req,
           robot_grasp::calibration_transform::Response &res)
  {

    x3 = depth+20;
    std::vector<float> res_obj_param;

    xm = req.obj_real_detected_in_cam.pose[0];
    ym = req.obj_real_detected_in_cam.pose[1];

    float Orientation = req.obj_real_detected_in_cam.pose[2];
    x1 = (xm-cx)*depth/fx;
    x2 = (ym-cy)*depth/fy;

    projection_point<<x1, x2, depth,1;
    grasp_point = Extrinsic_matrix.inverse()*projection_point;
    res_obj_param.push_back(grasp_point[0]);
    res_obj_param.push_back(grasp_point[1]);
    res_obj_param.push_back(grasp_point[2]);
    res_obj_param.push_back(Orientation);
    res.obj_real_detected_in_base = res_obj_param;
    return true;

  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calib_trans_");
  ros::NodeHandle nh("~");

  calib_trans nfe(nh);

  while (ros::ok())
    ros::spin();

  return 0;
}

