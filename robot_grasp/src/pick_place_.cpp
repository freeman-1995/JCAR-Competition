#include <pick_place.h>
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_ros/transform_listener.h>
const double FINGER_MAX = 6400;

using namespace kinova;

tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2)
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
}

PickPlace::PickPlace(ros::NodeHandle &nh):
    nh_(nh),tfListener(tfBuffer)
{
//    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
//    {
//        ros::console::notifyLoggerLevelsChanged();
//    }

    ros::NodeHandle pn("~");

    nh_.param<std::string>("/robot_type",robot_type_,"j2s7s300");
    nh_.param<bool>("/robot_connected",robot_connected_,false);

    if (robot_connected_)
    {
        //sub_joint_ = nh_.subscribe<sensor_msgs::JointState>("/j2s7s300_driver/out/joint_state", 1, &PickPlace::get_current_state, this);
        sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/" + robot_type_ +"_driver/out/tool_pose", 1, &PickPlace::get_current_pose, this);
    }

    // Before we can load the planner, we need two objects, a RobotModel and a PlanningScene.
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();

    // construct a `PlanningScene` that maintains the state of the world (including the robot).
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

//    //  every time need retrive current robot state, do the following.
//    robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
//    const robot_state::JointModelGroup *joint_model_group = robot_state.getJointModelGroup("arm");

    group_ = new moveit::planning_interface::MoveGroup("arm");
    gripper_group_ = new moveit::planning_interface::MoveGroup("gripper");

    group_->setEndEffectorLink(robot_type_ + "_end_effector");

    finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
            ("/" + robot_type_ + "_driver/fingers_action/finger_positions", false);
//    while(robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))){
//      ROS_INFO("Waiting for the finger action server to come up");
//    }

    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    int arm_joint_num = robot_type_[3]-'0';
    joint_names_.resize(arm_joint_num);
    joint_values_.resize(joint_names_.size());
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = robot_type_ + "_joint_" + boost::lexical_cast<std::string>(i+1);
    }
    kinova_gripper_joint_model_group = group_->getCurrentState()->getJointModelGroup("gripper");
    // set pre-defined joint and pose values.
    define_cartesian_pose();
    define_joint_values();

    //手爪参数初始化
//    cube_param[0] = "cube";
//    cube_param[1] = 0.92;
//    cube_param[2] = 0.93;
//    cube_param[3] = 0.93;
//    cone_param
//    cylinder_param
//    fourPrism_param
//    fourPrismoid_param
//    fourPyramid_param
//    halfSphere_param
//    ShortCylinder_param
//    sixPrism_param
//    threePrism_param
//    threePyramid_param

    //obj_finger_param.push_back(cube_param);

    // pick process
    result_ = false;

    group_->setNamedTarget("START");
    result_ = group_->plan(kinova_plan);
    group_->execute(kinova_plan);
    //evaluate_plan(*group_);

    OperateGripper(false);

    group_->setPoseTarget(intermedia_pt_0);
    result_ = group_->plan(kinova_plan);
    group_->execute(kinova_plan);
    //evaluate_plan(*group_);

    group_->setPoseTarget(intermedia_pt_1);
    result_ = group_->plan(kinova_plan);
    group_->execute(kinova_plan);
    //evaluate_plan(*group_);

    group_->setPoseTarget(intermedia_pt_2);
    result_ = group_->plan(kinova_plan);
    group_->execute(kinova_plan);
    //evaluate_plan(*group_);

    group_->setPoseTarget(intermedia_pt_3);
    result_ = group_->plan(kinova_plan);
    group_->execute(kinova_plan);
    //evaluate_plan(*group_);

    group_->setPoseTarget(intermedia_pt_4);
    result_ = group_->plan(kinova_plan);
    group_->execute(kinova_plan);

    //my_pick();
}


PickPlace::~PickPlace()
{
    // shut down pub and subs
    //sub_joint_.shutdown();
    //sub_pose_.shutdown();
    pub_co_.shutdown();
    pub_aco_.shutdown();
    pub_planning_scene_diff_.shutdown();

    // release memory
    delete group_;
    delete gripper_group_;
    delete finger_client_;
}


void PickPlace::get_current_state(const sensor_msgs::JointStateConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_state_);
    current_state_ = *msg;
}

void PickPlace::get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_pose_);
    current_pose_ = *msg;
}

/**
 * @brief PickPlace::gripper_action
 * @param gripper_rad close for 6400 and open for 0.0
 * @return true is gripper motion reaches the goal
 */
/*
bool PickPlace::gripper_action(double finger_turn)
{
    if(robot_connected_ == false)
    {
        if (finger_turn>0.5*FINGER_MAX)
        {
          gripper_group_->setNamedTarget("Close");
        }
        else
        {
          gripper_group_->setNamedTarget("Open");
        }
        gripper_group_->move();
        return true;
    }

    if (finger_turn < 0)
    {
        finger_turn = 0.0;
    }
    else
    {
        finger_turn = std::min(finger_turn, FINGER_MAX);
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    finger_client_->sendGoal(goal);

    if (finger_client_->waitForResult(ros::Duration(5.0)))
    {
        finger_client_->getResult();
        return true;
    }
    else
    {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}


void PickPlace::clear_workscene()
{
    // remove table
    co_.id = "table";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    // remove target
    co_.id = "target_cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    //remove attached target
    aco_.object.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_aco_.publish(aco_);

    planning_scene_msg_.world.collision_objects.clear();
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);

    clear_obstacle();
}

void PickPlace::build_workscene()
{
    co_.header.frame_id = "root";
    co_.header.stamp = ros::Time::now();

    // remove table
    co_.id = "table";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    // add table
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.4;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.4;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;
    co_.primitive_poses[0].position.x = 0;
    co_.primitive_poses[0].position.y = 0.0;
    co_.primitive_poses[0].position.z = -0.03/2.0;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();}

void PickPlace::clear_obstacle()
{
    co_.id = "pole";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    co_.id = "bot_obstacle";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    co_.id = "top_obstacle";
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": remove pole ");
    //      std::cin >> pause_;
}

void PickPlace::add_obstacle()
{
    clear_obstacle();

    co_.id = "pole";
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.4;

    co_.primitive_poses[0].position.x = 0.5;
    co_.primitive_poses[0].position.y = -0.1;
    co_.primitive_poses[0].position.z = 0.4/2.0;

    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
}

void PickPlace::add_complex_obstacle()
{
    clear_obstacle();

    // add obstacle between robot and object
    co_.id = "bot_obstacle";
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    double box_h = 0.2;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.4;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = box_h;

    co_.primitive_poses[0].position.x = 0;
    co_.primitive_poses[0].position.y = 0.3;
    co_.primitive_poses[0].position.z = box_h/2.0;
    co_.primitive_poses[0].orientation.w = 1.0;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    co_.id = "top_obstacle";
    co_.primitive_poses[0].position.z = box_h/2.0 + box_h + 0.4;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);

    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();
    //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ": ADD pole ");
    //      std::cin >> pause_;
}

void PickPlace::add_attached_obstacle()
{
    //once the object is know to be grasped
    //we remove obstacle from work scene
    co_.id = "target_cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    //and then we declare it as an attached obstacle
    aco_.object.operation = moveit_msgs::CollisionObject::ADD;
    aco_.link_name = robot_type_ + "_end_effector";
    aco_.touch_links.push_back(robot_type_ + "_end_effector");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_1");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_2");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_3");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_tip_1");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_tip_2");
    aco_.touch_links.push_back(robot_type_ + "_link_finger_tip_3");
    pub_aco_.publish(aco_);
}

void PickPlace::add_target()
{
    //remove target_cylinder
    co_.id = "target_cylinder";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    //add target_cylinder
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.5;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.04;
    co_.primitive_poses[0].position.x = 0.0;
    co_.primitive_poses[0].position.y = 0.6;
    co_.primitive_poses[0].position.z = 0.3;
    can_pose_.pose.position.x = co_.primitive_poses[0].position.x;
    can_pose_.pose.position.y = co_.primitive_poses[0].position.y;
    can_pose_.pose.position.z = co_.primitive_poses[0].position.z;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    aco_.object = co_;
    ros::WallDuration(0.1).sleep();
}
*/

void PickPlace::define_cartesian_pose()
{
    intermedia_pt_0.header.frame_id = "j2s7s300_link_base";
    intermedia_pt_0.header.stamp = ros::Time::now();
    intermedia_pt_0.pose.position.x = 0.0;
    intermedia_pt_0.pose.position.y = 0.0;
    intermedia_pt_0.pose.position.z = 0.89836;
    intermedia_pt_0.pose.orientation.x = 0.71602;
    intermedia_pt_0.pose.orientation.y = 0.0;
    intermedia_pt_0.pose.orientation.z = 0.69808;
    intermedia_pt_0.pose.orientation.w = 0.0;

    intermedia_pt_1.header.frame_id = "j2s7s300_link_base";
    intermedia_pt_1.header.stamp = ros::Time::now();
    intermedia_pt_1.pose.position.x = 0.085989;
    intermedia_pt_1.pose.position.y = 0.0;
    intermedia_pt_1.pose.position.z = 0.6272;
    intermedia_pt_1.pose.orientation.x = 0.71851;
    intermedia_pt_1.pose.orientation.y = 0.0;
    intermedia_pt_1.pose.orientation.z = 0.69551;
    intermedia_pt_1.pose.orientation.w = 0.0;

    intermedia_pt_2.header.frame_id = "j2s7s300_link_base";
    intermedia_pt_2.header.stamp = ros::Time::now();
    intermedia_pt_2.pose.position.x = 0.35653;
    intermedia_pt_2.pose.position.y = 0.0;
    intermedia_pt_2.pose.position.z = 0.33787;
    intermedia_pt_2.pose.orientation.x = 1.0;
    intermedia_pt_2.pose.orientation.y = 0.0;
    intermedia_pt_2.pose.orientation.z = 0.0;
    intermedia_pt_2.pose.orientation.w = 0.0;

    intermedia_pt_3.header.frame_id = "j2s7s300_link_base";
    intermedia_pt_3.header.stamp = ros::Time::now();
    intermedia_pt_3.pose.position.x = 0.35704;
    intermedia_pt_3.pose.position.y = 0.34392;
    intermedia_pt_3.pose.position.z = 0.33698;
    intermedia_pt_3.pose.orientation.x = 1.0;
    intermedia_pt_3.pose.orientation.y = 0.0;
    intermedia_pt_3.pose.orientation.z = 0.0;
    intermedia_pt_3.pose.orientation.w = 0.0;

    intermedia_pt_4.header.frame_id = "j2s7s300_link_base";
    intermedia_pt_4.header.stamp = ros::Time::now();
    intermedia_pt_4.pose.position.x = 0.46955;
    intermedia_pt_4.pose.position.y = 0.3427;
    intermedia_pt_4.pose.position.z = 0.12178;
    intermedia_pt_4.pose.orientation.x = 1.0;
    intermedia_pt_4.pose.orientation.y = 0.0;
    intermedia_pt_4.pose.orientation.z = 0.0;
    intermedia_pt_4.pose.orientation.w = 0.0;

    obj_place_pose.header.frame_id = "j2s7s300_link_base";
    obj_place_pose.header.stamp = ros::Time::now();
    obj_place_pose.pose.position.x = 0.537;
    obj_place_pose.pose.position.y = -0.25853;
    obj_place_pose.pose.position.z = 0.042587;
    obj_place_pose.pose.orientation.x = 1.0;
    obj_place_pose.pose.orientation.y = 0.0;
    obj_place_pose.pose.orientation.z = 0.0;
    obj_place_pose.pose.orientation.w = 0.0;

    pre_obj_place_pose = obj_place_pose;
    pre_obj_place_pose.pose.position.z = obj_place_pose.pose.position.z +0.1;
}

void PickPlace::define_joint_values()
{
    start_joint_.resize(joint_names_.size());
    //    getInvK(start_pose_, start_joint_);
    start_joint_[0] = 0 *M_PI/180.0;
    start_joint_[1] = 135 *M_PI/180.0;
    start_joint_[2] = 0 *M_PI/180.0;
    start_joint_[3] = 135 *M_PI/180.0;
    start_joint_[4] = 0 *M_PI/180.0;
    start_joint_[5] = 135 *M_PI/180.0;


    grasp_joint_.resize(joint_names_.size());
    //    getInvK(grasp_pose, grasp_joint_);
    grasp_joint_[0] = 144.0 *M_PI/180.0;
    grasp_joint_[1] = 256.5 *M_PI/180.0;
    grasp_joint_[2] = 91.3 *M_PI/180.0;
    grasp_joint_[3] = 163.8 *M_PI/180.0;
    grasp_joint_[4] = 88.5 *M_PI/180.0;
    grasp_joint_[5] = 151.3 *M_PI/180.0;

    pregrasp_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, pregrasp_joint_);
    pregrasp_joint_[0] = 145.4 *M_PI/180.0;
    pregrasp_joint_[1] = 253.7 *M_PI/180.0;
    pregrasp_joint_[2] = 67.0 *M_PI/180.0;
    pregrasp_joint_[3] = 151.0 *M_PI/180.0;
    pregrasp_joint_[4] = 118.5 *M_PI/180.0;
    pregrasp_joint_[5] = 141.7 *M_PI/180.0;

//    postgrasp_joint_ = pregrasp_joint_;
    postgrasp_joint_.resize(joint_names_.size());
    //    getInvK(pregrasp_pose, postgrasp_joint_);
    postgrasp_joint_[0] = 144 *M_PI/180.0;
    postgrasp_joint_[1] = 249 *M_PI/180.0;
    postgrasp_joint_[2] = 88 *M_PI/180.0;
    postgrasp_joint_[3] = 165 *M_PI/180.0;
    postgrasp_joint_[4] = 83 *M_PI/180.0;
    postgrasp_joint_[5] = 151 *M_PI/180.0;
}

/*
geometry_msgs::PoseStamped PickPlace::generate_gripper_align_pose(geometry_msgs::PoseStamped targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z)
{
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.frame_id = "root";

    // computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = -dist * cos(azimuth) * sin(polar);
    double delta_y = -dist * sin(azimuth) * sin(polar);
    double delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q = EulerZYZ_to_Quaternion(azimuth, polar, rot_gripper_z);

    pose_msg.pose.position.x = targetpose_msg.pose.position.x + delta_x;
    pose_msg.pose.position.y = targetpose_msg.pose.position.y + delta_y;
    pose_msg.pose.position.z = targetpose_msg.pose.position.z + delta_z;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "pose_msg: x " << pose_msg.pose.position.x  << ", y " << pose_msg.pose.position.y  << ", z " << pose_msg.pose.position.z  << ", qx " << pose_msg.pose.orientation.x  << ", qy " << pose_msg.pose.orientation.y  << ", qz " << pose_msg.pose.orientation.z  << ", qw " << pose_msg.pose.orientation.w );

    return pose_msg;

}


void PickPlace::setup_constrain(geometry_msgs::Pose target_pose, bool orientation, bool position)
{
    if ( (!orientation) && (!position) )
    {
        ROS_WARN("Neither orientation nor position constrain applied.");
        return;
    }

    moveit_msgs::Constraints grasp_constrains;

    // setup constrains
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = robot_type_ + "_end_effector";
    ocm.header.frame_id = "root";
    ocm.orientation = target_pose.orientation;
    ocm.absolute_x_axis_tolerance = 2*M_PI;
    ocm.absolute_y_axis_tolerance = 2*M_PI;
    ocm.absolute_z_axis_tolerance = M_PI/10;
    ocm.weight = 0.5;
    if (orientation)
    {
        grasp_constrains.orientation_constraints.push_back(ocm);
    }


    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);

    // group_->getCurrentPose() does not work.
//    current_pose_ = group_->getCurrentPose();
    geometry_msgs::Pose current_pose;
    { // scope for mutex update
        boost::mutex::scoped_lock lock_pose(mutex_pose_);
        current_pose = current_pose_.pose;
//        ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "current_pose_: x " << current_pose_.pose.position.x  << ", y " << current_pose_.pose.position.y  << ", z " << current_pose_.pose.position.z  << ", qx " << current_pose_.pose.orientation.x  << ", qy " << current_pose_.pose.orientation.y  << ", qz " << current_pose_.pose.orientation.z  << ", qw " << current_pose_.pose.orientation.w );
    }

    double constrain_box_scale = 2.0;
    primitive.dimensions[0] = constrain_box_scale * std::abs(target_pose.position.x - current_pose.position.x);
    primitive.dimensions[1] = constrain_box_scale * std::abs(target_pose.position.y - current_pose.position.y);
    primitive.dimensions[2] = constrain_box_scale * std::abs(target_pose.position.z - current_pose.position.z);

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    // place between start point and goal point.
    box_pose.position.x = (target_pose.position.x + current_pose.position.x)/2.0;
    box_pose.position.y = (target_pose.position.y + current_pose.position.y)/2.0;
    box_pose.position.z = (target_pose.position.z + current_pose.position.z)/2.0;

    moveit_msgs::PositionConstraint pcm;
    pcm.link_name = robot_type_+"_end_effector";
    pcm.header.frame_id = "root";
    pcm.constraint_region.primitives.push_back(primitive);
    pcm.constraint_region.primitive_poses.push_back(box_pose);
    pcm.weight = 0.5;
    if(position)
    {
        grasp_constrains.position_constraints.push_back(pcm);
    }

    group_->setPathConstraints(grasp_constrains);


//    // The bellowing code is just for visulizing the box and check.
//    // Disable this part after checking.
//    co_.id = "check_constrain";
//    co_.operation = moveit_msgs::CollisionObject::REMOVE;
//    pub_co_.publish(co_);

//    co_.operation = moveit_msgs::CollisionObject::ADD;
//    co_.primitives.push_back(primitive);
//    co_.primitive_poses.push_back(box_pose);
//    pub_co_.publish(co_);
//    planning_scene_msg_.world.collision_objects.push_back(co_);
//    planning_scene_msg_.is_diff = true;
//    pub_planning_scene_diff_.publish(planning_scene_msg_);
//    ros::WallDuration(0.1).sleep();
}

void PickPlace::check_constrain()
{
    moveit_msgs::Constraints grasp_constrains = group_->getPathConstraints();
    bool has_constrain = false;
    ROS_INFO("check constrain result: ");
    if (!grasp_constrains.orientation_constraints.empty())
    {
        has_constrain = true;
        ROS_INFO("Has orientation constrain. ");
    }
    if(!grasp_constrains.position_constraints.empty())
    {
        has_constrain = true;
        ROS_INFO("Has position constrain. ");
    }
    if(has_constrain == false)
    {
        ROS_INFO("No constrain. ");
    }
}
tf::Quaternion PickPlace::RPYToQuaternion(float R, float P, float Y)
{
  tf::Matrix3x3 mat;
  mat.setEulerYPR(Y,P,R);
  tf::Quaternion quat;
  mat.getRotation(quat);
  return quat;
}
void PickPlace::check_collision()
{
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");

    collision_request.group_name = "arm";
    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 3: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");

    // check contact
    planning_scene_->checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 4: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for(it = collision_result.contacts.begin();
        it != collision_result.contacts.end();
        ++it)
    {
        ROS_INFO("Contact between: %s and %s",
                 it->first.first.c_str(),
                 it->first.second.c_str());
    }

    // allowed collision matrix
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene_->getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for(it2 = collision_result.contacts.begin();
        it2 != collision_result.contacts.end();
        ++it2)
    {
        acm.setEntry(it2->first.first, it2->first.second, true);
    }
    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 5: Current state is "
                    << (collision_result.collision ? "in" : "not in")
                    << " collision");
}


geometry_msgs::PoseStamped PickPlace::cale_obj_pose_in_robot_base(float w_x,float w_y,float w_z)
{
  geometry_msgs::PoseStamped Obj_Pose;
  tf::Quaternion obj_quat;
  //obj_quat = RPYToQuaternion(R,P,Y);
  try
  {
    geometry_msgs::PointStamped TEMP_WORLD_pt;
    TEMP_WORLD_pt.point.x = w_x;
    TEMP_WORLD_pt.point.y = w_y;
    TEMP_WORLD_pt.point.z = w_z;
    TEMP_WORLD_pt.header.frame_id = "/camera_rgb_frame";
    transformStamped = tfBuffer.lookupTransform("j2s7s300_link_base", "camera_rgb_frame", ros::Time(0),ros::Duration(3.0));
    geometry_msgs::PointStamped  TEMP_ROBOT_pt;
    TEMP_ROBOT_pt.header.frame_id = "/j2s7s300_link_base";
    tf2::doTransform(TEMP_WORLD_pt, TEMP_ROBOT_pt, transformStamped);
    Obj_Pose.pose.position.x = TEMP_ROBOT_pt.point.x;
    Obj_Pose.pose.position.y = TEMP_ROBOT_pt.point.y;
    Obj_Pose.pose.position.z = TEMP_ROBOT_pt.point.z;
    Obj_Pose.pose.orientation.x = 0.70468;
    Obj_Pose.pose.orientation.y = 0.70949;
    Obj_Pose.pose.orientation.z = -0.0028649;
    Obj_Pose.pose.orientation.w = -0.0060894;
  }
  catch (tf2::TransformException &ex)
  {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
  }
  return Obj_Pose;
}
*/

void PickPlace::evaluate_plan(moveit::planning_interface::MoveGroup &group)
{
    bool replan = true;
    int count = 0;

    moveit::planning_interface::MoveGroup::Plan my_plan;

    while (replan == true && ros::ok())
    {
        // reset flag for replan
        count = 0;
        result_ = false;

        // try to find a success plan.
        double plan_time;
        while (result_ == false && count < 5)
        {
            count++;
            plan_time = 20+count*10;
            ROS_INFO("Setting plan time to %f sec", plan_time);
            group.setPlanningTime(plan_time);
            result_ = group.plan(my_plan);
            std::cout << "at attemp: " << count << std::endl;
            ros::WallDuration(0.1).sleep();
        }

        // found a plan
        if (result_ == true)
        {
            std::cout << "plan success at attemp: " << count << std::endl;

            replan = false;
            std::cout << "please input e to execute the plan, r to replan, others to skip: ";
            std::cin >> pause_;
            ros::WallDuration(0.5).sleep();
            if (pause_ == "r" || pause_ == "R" )
            {
                replan = true;
            }
            else
            {
                replan = false;
            }
        }
        else // not found
        {
            std::cout << "Exit since plan failed until reach maximum attemp: " << count << std::endl;
            replan = false;
            break;
        }
    }

    if(result_ == true)
    {
        if (pause_ == "e" || pause_ == "E")
        {
            group.execute(my_plan);
        }
    }
    ros::WallDuration(1.0).sleep();
}

bool PickPlace::OperateGripper(bool close_gripper)
{
  moveit::core::RobotStatePtr gripper_current_state = gripper_group_->getCurrentState();
  std::vector<double> gripper_joint_positions;
  gripper_current_state->copyJointGroupPositions(kinova_gripper_joint_model_group,gripper_joint_positions);
  ROS_INFO("No. of joints in eef_group: %zd", gripper_joint_positions.size());
  if (close_gripper)
  {
    switch (num_of_label)
    {
      case 1:
      {
      //"cube" ok
        gripper_joint_positions[0] = 0.94;
        gripper_joint_positions[1] = 0.93;//89
        gripper_joint_positions[2] = 0.93;//90
        ROS_INFO("closed");
        break;
      }
      case 2:
      {
      //"cone" ok
        gripper_joint_positions[0] = 1.19;
        gripper_joint_positions[1] = 1.19;//89
        gripper_joint_positions[2] = 1.19;//90
        ROS_INFO("closed");
        break;
      }
      case 3:
      {
      //"cylinder" ok
        gripper_joint_positions[0] = 1.3;
        gripper_joint_positions[1] = 1.3;//89
        gripper_joint_positions[2] = 1.3;//90
        ROS_INFO("closed");
        break;
      }
      case 4:
      {
      //"fourPrism" ok
        gripper_joint_positions[0] = 1.35;
        gripper_joint_positions[1] = 1.35;//89
        gripper_joint_positions[2] = 1.35;//90
        ROS_INFO("closed");
        break;
      }
      case 5:
      {
      //"fourPrismoid" ok
        gripper_joint_positions[0] = 1.27;
        gripper_joint_positions[1] = 1.26;//89
        gripper_joint_positions[2] = 1.26;//90
        ROS_INFO("closed");
        break;
      }
      case 6:
      {
      //"fourPyramid" ok
        gripper_joint_positions[0] = 1.3;
        gripper_joint_positions[1] = 1.3;//89
        gripper_joint_positions[2] = 1.3;//90
        ROS_INFO("closed");
        break;
      }
      case 7:
      {
      //"threePyramid"
        gripper_joint_positions[0] = 1.26;
        gripper_joint_positions[1] = 1.25;//89
        gripper_joint_positions[2] = 1.25;//90
        ROS_INFO("closed");
        break;
      }
      case 8:
      {
      //"sphere" ok
        gripper_joint_positions[0] = 1.15;
        gripper_joint_positions[1] = 1.15;//89
        gripper_joint_positions[2] = 1.15;//90
        ROS_INFO("closed");
        break;
      }
      case 9:
      {
      //"sixPrism" ok
        gripper_joint_positions[0] = 1.35;
        gripper_joint_positions[1] = 1.35;//89
        gripper_joint_positions[2] = 1.35;//90
        ROS_INFO("closed");
        break;
      }
      case 10:
      {
      //"threePrism"
        gripper_joint_positions[0] = 1.33;
        gripper_joint_positions[1] = 1.32;//89
        gripper_joint_positions[2] = 1.33;//90
        ROS_INFO("closed");
        break;
      }
    }
  }
  else
  {
    gripper_joint_positions[0] = 0.73;
    gripper_joint_positions[1] = 0.73;
    gripper_joint_positions[2] = 0.73;
    ROS_INFO("opened");
  }
  gripper_group_->setJointValueTarget(gripper_joint_positions);
  ros::Duration(1.0).sleep();
  bool success = gripper_group_->move();
  return success;
}

bool PickPlace::kinova_routine(robot_grasp::G_pick_place::Request &req,robot_grasp::G_pick_place::Response &res)
{
  if(req.transformed_param.pcaCentroid.size()==0)
  {
    std::cout<<"No obj to be grasped !"<<std::endl;
    return true;
  }
  //bool move_success;
  //group_->setPlanningTime(10);

  num_of_label = req.transformed_param.num_obj_label;

  detected_param_obj_list = req.transformed_param.pcaCentroid;

  //evaluate_plan(*group_);
  /*for(int i=0;i<detected_param_obj_list.size();i++)
  {
    //obj_grasp_label = detected_param_obj_list[i].obj_label;

    obj_grasp_pose.header.frame_id = "j2s7s300_link_base";
    obj_grasp_pose.header.stamp = ros::Time::now();
    obj_grasp_pose.pose.position.x = detected_param_obj_list[i].pcaCentroid[0];
    obj_grasp_pose.pose.position.y = detected_param_obj_list[i].pcaCentroid[1];
    obj_grasp_pose.pose.position.z = detected_param_obj_list[i].pcaCentroid[2]+0.003;

    ROS_INFO_STREAM("Press any key to send robot to grasp ...");
    //std::cin >> pause_;
    obj_grasp_pose.pose.orientation.x = 1.0;
    obj_grasp_pose.pose.orientation.y = 0.0;
    obj_grasp_pose.pose.orientation.z = 0.0;
    obj_grasp_pose.pose.orientation.w = 0.0;

    obj_pre_grasp_pose = obj_grasp_pose;
    obj_pre_grasp_pose.pose.position.z +=0.1;

    //std::cout<<" Grasp Obj "+obj_grasp_label+" at :"<<std::endl;
    std::cout<<obj_grasp_pose.pose.position.x<<" ,"<<obj_grasp_pose.pose.position.y<<" ,"<<obj_grasp_pose.pose.position.z<<std::endl;
    std::cout<<"------------------------------grasp test----------------------------"<<std::endl;
    ROS_INFO_STREAM("Approaching Pre grasp position ...");
    group_->setPoseTarget(obj_pre_grasp_pose);
    result_ = group_->plan(kinova_plan);
    group_->execute(kinova_plan);
    //evaluate_plan(*group_);

    ROS_INFO_STREAM("Approaching grasp position ...");
    group_->setPoseTarget(obj_grasp_pose);
    result_ = group_->plan(kinova_plan);
    group_->execute(kinova_plan);
    //evaluate_plan(*group_);
    ros::WallDuration(1.0).sleep();

    OperateGripper(true);

    ROS_INFO_STREAM("Approaching Pre grasp position ...");
    group_->setPoseTarget(obj_pre_grasp_pose);
    result_ = group_->plan(kinova_plan);
    group_->execute(kinova_plan);
    //evaluate_plan(*group_);

    ROS_INFO_STREAM("Placing Obj");
    group_->setPoseTarget(obj_place_pose);
    result_ = group_->plan(kinova_plan);
    group_->execute(kinova_plan);
    //evaluate_plan(*group_);

    std::cout<<"--------------------------------------------------------------------"<<std::endl;
  }*/

  obj_grasp_pose.header.frame_id = "j2s7s300_link_base";
  obj_grasp_pose.header.stamp = ros::Time::now();
  obj_grasp_pose.pose.position.x = detected_param_obj_list[0];
  obj_grasp_pose.pose.position.y = detected_param_obj_list[1];
  obj_grasp_pose.pose.position.z = detected_param_obj_list[2]+0.022;
  obj_grasp_pose.pose.orientation.x = 1.0;
  obj_grasp_pose.pose.orientation.y = 0.0;
  obj_grasp_pose.pose.orientation.z = 0.0;
  obj_grasp_pose.pose.orientation.w = 0.0;

  obj_pre_grasp_pose = obj_grasp_pose;
  obj_pre_grasp_pose.pose.position.z +=0.23;

  group_->setPoseTarget(obj_pre_grasp_pose);
  result_ = group_->plan(kinova_plan);
  group_->execute(kinova_plan);
  //evaluate_plan(*group_);

  group_->setPoseTarget(obj_grasp_pose);
  result_ = group_->plan(kinova_plan);
  group_->execute(kinova_plan);
  //evaluate_plan(*group_);
  ros::WallDuration(1.0).sleep();

  OperateGripper(true);

  group_->setPoseTarget(obj_pre_grasp_pose);
  result_ = group_->plan(kinova_plan);
  group_->execute(kinova_plan);
  //evaluate_plan(*group_);

  group_->setPoseTarget(pre_obj_place_pose);
  result_ = group_->plan(kinova_plan);
  group_->execute(kinova_plan);
  //evaluate_plan(*group_);

  result_ = OperateGripper(false);
  result_ = OperateGripper(false);
  res.success = result_;

//  group_->setPoseTarget(pre_obj_place_pose);
//  result_ = group_->plan(kinova_plan);
//  group_->execute(kinova_plan);
  //evaluate_plan(*group_);


  return true;
}
/*
bool PickPlace::my_pick()
{
    clear_workscene();
    ros::WallDuration(1.0).sleep();
    build_workscene();
    ros::WallDuration(1.0).sleep();

    ROS_INFO_STREAM("Press any key to send robot to START position ...");
    std::cin >> pause_;
     group_->clearPathConstraints();
    group_->setNamedTarget("START");
    evaluate_plan(*group_);

    ros::WallDuration(1.0).sleep();
    gripper_group_->setNamedTarget("Open");
    gripper_group_->move();

    ///////////////////////////////////////////////////////////
    //// joint space without obstacle
    ///////////////////////////////////////////////////////////

//    ROS_INFO_STREAM("Joint space motion planing without obstacle");
//    ROS_INFO_STREAM("Demonstrates moving robot from one joint position to another");
//    ROS_INFO_STREAM("Planning to go to start pose ...");
//    group_->setJointValueTarget(start_joint_);
//    evaluate_plan(*group_);

//    ROS_INFO_STREAM("Planning to go to pre-grasp joint pose ...");
//    group_->setJointValueTarget(pregrasp_joint_);
//    evaluate_plan(*group_);

//    ROS_INFO_STREAM("Approaching grasp position ...");
//    group_->setJointValueTarget(grasp_joint_);
//    evaluate_plan(*group_);

//    ROS_INFO_STREAM("Grasping ...");
//    gripper_action(0.75*FINGER_MAX); // partially close

//    ROS_INFO_STREAM("Planning to go to retract position ...");
//    group_->setJointValueTarget(postgrasp_joint_);
//    evaluate_plan(*group_);

//    ROS_INFO_STREAM("Planning to go to start position ...");
//    group_->setJointValueTarget(start_joint_);
//    evaluate_plan(*group_);

//    ROS_INFO_STREAM("Releasing ...");
//    gripper_action(0.0); // full open


    ///////////////////////////////////////////////////////////
    //// Cartesian space without obstacle
    ///////////////////////////////////////////////////////////
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("Motion planning in cartesian space without obstacles ...");
    clear_workscene();
    build_workscene();
    add_target();
    ros::WallDuration(0.1).sleep();

    ROS_INFO_STREAM("Press any key to move to start pose ...");
    std::cin >> pause_;
    group_->setPoseTarget(start_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Planning to go to pre-grasp position ...");
    group_->setPoseTarget(intermedia_pt_1);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Approaching grasp position ...");
    group_->setPoseTarget(intermedia_pt_2);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Approaching grasp position ...");
    group_->setPoseTarget(intermedia_pt_3);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Grasping ...");
    add_attached_obstacle();
    gripper_action(0.75*FINGER_MAX); // partially close

    ROS_INFO_STREAM("Releasing gripper ...");

    gripper_action(0.0); // full open

    clear_workscene();
    ROS_INFO_STREAM("Press any key to quit ...");
    std::cin >> pause_;
    return true;
}
*/

void PickPlace::getInvK(geometry_msgs::Pose &eef_pose, std::vector<double> &joint_value)
{
    // TODO: transform cartesian command to joint space, and alway motion plan in joint space.
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_place_demo");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    kinova::PickPlace pick_place(node);
    ros::ServiceServer service = node.advertiseService("/get_pick_place_routine", &PickPlace::kinova_routine, &pick_place);
    ros::spin();
    return 0;
}
