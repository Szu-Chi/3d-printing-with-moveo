#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_msgs/Float64MultiArray.h"
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <include/Joint_division/joint_division.hpp>

void motor_steps_convert(std::vector<double> &data, KDL::Chain &chain, std::vector<double> &joint_division){
  for(int i = 0; i < chain.getNrOfJoints(); i++){
    ROS_INFO_STREAM("Joint" << (i+1) << " = " << int(data.at(i)/360*joint_division[i]));
    data.at(i) = data.at(i)*(M_PI)/180;
  }
}

void print_current_pose(geometry_msgs::PoseStamped current_pose){
  ROS_INFO_NAMED("moveo", "x position: %f", current_pose.pose.position.x);
  ROS_INFO_NAMED("moveo", "y position: %f", current_pose.pose.position.y);
  ROS_INFO_NAMED("moveo", "z position: %f", current_pose.pose.position.z);
  ROS_INFO_NAMED("moveo", "x orientation: %f", current_pose.pose.orientation.x);
  ROS_INFO_NAMED("moveo", "y orientation: %f", current_pose.pose.orientation.y);
  ROS_INFO_NAMED("moveo", "z orientation: %f", current_pose.pose.orientation.z);
  ROS_INFO_NAMED("moveo", "w orientation: %f", current_pose.pose.orientation.w);
}

void set_target_joint(const std_msgs::Float64MultiArray& chain_end_joint){
  ros::NodeHandle node_handle("~");

  //----------------------------
  //Setup
  //----------------------------
  static const std::string PLANNING_GROUP = "arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ros::Publisher pose_joint_pub = node_handle.advertise<sensor_msgs::JointState>("pose_joint", 10);
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  std::string chain_start, chain_end, urdf_param;
  double timeout, eps;
  node_handle.param("chain_start", chain_start, std::string(""));
  node_handle.param("chain_end", chain_end, std::string(""));
  if (chain_start == "" || chain_end == ""){
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }
  node_handle.param("timeout", timeout, 0.1);
  node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
  node_handle.param("eps", eps, 1e-5);
  ROS_INFO_STREAM("eps :" << eps);
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps, TRAC_IK::Speed);
  KDL::Chain chain;
  bool valid = tracik_solver.getKDLChain(chain);
  if (!valid){
    ROS_ERROR("There was no valid KDL chain found");
    exit(-1);
  }
  ROS_INFO("Using %d joints", chain.getNrOfJoints());

  std::string joint_division_name;
  node_handle.param("joint_division", joint_division_name, std::string("/joint_division"));
  std::vector<double> joint_division = read_joint_division(joint_division_name, chain.getNrOfJoints());

  std::vector<double> target_joints(chain.getNrOfJoints());
  for(int i=0;i<chain.getNrOfJoints();i++){
    target_joints.at(i) = chain_end_joint.data[i];
  }
  
  motor_steps_convert(target_joints,chain, joint_division);
  if (!target_joints.empty()){
    move_group.setJointValueTarget(target_joints);
    move_group.setPlanningTime(0.5);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = move_group.plan(my_plan);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.move();
    print_current_pose(move_group.getCurrentPose());
    sensor_msgs::JointState moveo_joint_state;
    std::vector<std::string> name{"moveo_joint1", "moveo_joint2", "moveo_joint3", "moveo_joint4", "moveo_joint5"};
    std::vector<double> position = move_group.getCurrentJointValues();
    moveo_joint_state.name = name;
    moveo_joint_state.position = position;
    pose_joint_pub.publish(moveo_joint_state);
    KDL::JntArray Joint(chain.getNrOfJoints());
    for(int i = 0;i < chain.getNrOfJoints();i++){
      Joint(i) = target_joints.at(i);
    }
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::Frame end_effector_pose;
    fk_solver.JntToCart(Joint, end_effector_pose);
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = end_effector_pose.p.x();
    target_pose1.position.y = end_effector_pose.p.y();
    target_pose1.position.z = end_effector_pose.p.z();
    end_effector_pose.M.GetQuaternion	(target_pose1.orientation.x, target_pose1.orientation.y, target_pose1.orientation.z, target_pose1.orientation.w);
    visual_tools.publishAxisLabeled(target_pose1, "target_pose1");
    visual_tools.trigger();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_1");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  //----------------------------
  //Setup
  //----------------------------
  static const std::string PLANNING_GROUP = "arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  // We can print the name of the reference frame for this robot.
  // also printing the current position and orientation of the robot.
  ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);
  ros::Subscriber eff_pose_sub = node_handle.subscribe("chain_end_joint", 100000, set_target_joint);
  print_current_pose(move_group.getCurrentPose());

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  //Plan a motion for this group to a desired pose for end-effector
  // hardcode desired position here before running node in a separate terminal
  while(ros::ok()){
    ros::waitForShutdown();
  }

  ros::shutdown();  
  return 0;
}
