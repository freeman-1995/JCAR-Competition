#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

typedef Eigen::Matrix<Scalar,3,3> Matrix3_;
using namespace cv;
using namespace std;

tf::TransformBroadcaster br;
tf::Transform transform;
bool found;

Size image_size;  /* 图像的尺寸 */
Size board_size = Size(7,5);    /* 标定板上每行、列的角点数 */
vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
vector<vector<Point2f> > image_points_seq; /* 保存检测到的所有角点 */
vector<Mat> tvecsMat;  /* 每幅图像的旋转向量 */
vector<Mat> rvecsMat; /* 每幅图像的平移向量 */
/*棋盘三维信息*/
Size square_size = Size(25.5,25.5);  /* 实际测量得到的标定板上每个棋盘格的大小 */
vector<vector<Point3f> > object_points; /* 保存标定板上角点的三维坐标 */
Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0)); /* 摄像机内参数矩阵 */
Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
Mat view, viewGray;
Mat Rotation_Matrix;
Eigen::Quaterniond Q;
Matrix3_ rotate_mat;
;
Eigen::AngleAxis rotate_axis;
void img_callback(sensor_msgs::Image msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  view = cv_ptr->image;
  cvtColor(view, viewGray, COLOR_BGR2GRAY);
  found = findChessboardCorners(view, board_size, image_points_buf,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
  if (found)
  {
    cornerSubPix(viewGray, image_points_buf, Size(11, 11),Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    drawChessboardCorners(view, board_size, Mat(image_points_buf), found);
  }
  image_points_seq.push_back(image_points_buf);
  imshow("Image View", view);
  waitKey(1);
  vector<Point3f> tempPointSet;
  for (int i=0;i<board_size.height;i++)
  {
    for (int j=0;j<board_size.width;j++)
    {
      Point3f realPoint;
      /* 假设标定板放在世界坐标系中z=0的平面上 */
      realPoint.x = i*square_size.width;
      realPoint.y = j*square_size.height;
      realPoint.z = 0;
      tempPointSet.push_back(realPoint);
    }
  }
  object_points.push_back(tempPointSet);
  calibrateCamera(object_points,image_points_seq,image_size,cameraMatrix,distCoeffs,rvecsMat,tvecsMat,0);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_hand_eye");
  ros::NodeHandle nh;
  while (nh.ok())
  {
    rotate_mat = rotate_axis.toRotationMatrix;
    rotate_mat
    transform.setOrigin(tf::Vector3(tvecsMat[0].at<float>(0,0),tvecsMat[0].at<float>(1,0),tvecsMat[0].at<float>(2,0)));
    transform.setRotation(tf::Quaternion(Q2[0],Q2[1],Q2[2],Q2[3]));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "kinect2_rgb_optical_frame", "world_frame"));
  }
  return 0;
}





