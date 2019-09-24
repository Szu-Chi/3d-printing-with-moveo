#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>

#include <sstream>
#include <fstream>

#include <time.h>
#include <math.h>

ros::WallTime start_, end_;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_1");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //----------------------------
  //Setup
  //----------------------------
  static const std::string PLANNING_GROUP = "arm";
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  //Using :planning_scene_interface:'PlanningSceneInterface' class to deal directly with the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //     http://docs.ros.org/kinetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  //const robot_state::JointModelGroup *joint_model_group =
  //  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

  // We can print the name of the reference frame for this robot.
  // also printing the current position and orientation of the robot.
  ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);
  ros::Publisher pose_joint_pub = node_handle.advertise<sensor_msgs::JointState>("pose_joint", 10);
  ROS_INFO_NAMED("moveo", "x position: %f", current_pose.pose.position.x);
  ROS_INFO_NAMED("moveo", "y position: %f", current_pose.pose.position.y);
  ROS_INFO_NAMED("moveo", "z position: %f", current_pose.pose.position.z);
  ROS_INFO_NAMED("moveo", "x orientation: %f", current_pose.pose.orientation.x);
  ROS_INFO_NAMED("moveo", "y orientation: %f", current_pose.pose.orientation.y);
  ROS_INFO_NAMED("moveo", "z orientation: %f", current_pose.pose.orientation.z);
  ROS_INFO_NAMED("moveo", "w orientation: %f", current_pose.pose.orientation.w);

  std::string chain_start, chain_end, urdf_param;
  double timeout;
  node_handle.param("chain_start", chain_start, std::string(""));
  node_handle.param("chain_end", chain_end, std::string(""));
  if (chain_start == "" || chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }
  node_handle.param("timeout", timeout, 0.005);
  node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
  //TRAC_IK::SolveType solve_type;
  //node_handle.param("solve_type", solve_type, TRAC_IK::Speed);

  double eps = 1e-5;
  node_handle.param("eps", eps, 1e-5);
  ROS_INFO_STREAM("eps :" << eps);
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps, TRAC_IK::Distance);
  TRAC_IK::TRAC_IK tracik_solver2(chain_start, chain_end, urdf_param, timeout*20, eps, TRAC_IK::Distance);
  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  ROS_INFO_NAMED("moveo", "TRAC-IK setup");
  if (!valid)
  {
    ROS_ERROR("There was no valid KDL chain found");
    return -1;
  }

  valid = tracik_solver.getKDLLimits(ll, ul);

  if (!valid)
  {
    ROS_ERROR("There were no valid KDL joint limits found");
    return -1;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());
  ROS_INFO("Using %d joints", chain.getNrOfJoints());
  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in Rviz
  visual_tools.loadRemoteControl();

  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0; // above head of PR2
  visual_tools.publishText(text_pose, "Moveo Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
  visual_tools.trigger();


  //-----------------------------
  //Getting Basic Information
  //-----------------------------

  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("moveo", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("moveo", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  //-----------------------------
  //Planning to a Pose Goal
  //-----------------------------

  //Plan a motion for this group to a desired pose for end-effector
  // hardcode desired position here before running node in a separate terminal

  KDL::JntArray result;
  KDL::Vector end_effector_target_vol;
  //node_handle.param("end_effector_target_vol_x", end_effector_target_vol.data[0], 0.0);
  //node_handle.param("end_effector_target_vol_y", end_effector_target_vol.data[1], 0.3);
  //node_handle.param("end_effector_target_vol_z", end_effector_target_vol.data[2], 0.4);

  KDL::Rotation end_effector_target_rot;
  double end_effector_target_R, end_effector_target_P,end_effector_target_Y;
  node_handle.param("end_effector_target_rot_R", end_effector_target_R, 0.0);
  node_handle.param("end_effector_target_rot_P", end_effector_target_P, 0.0);
  node_handle.param("end_effector_target_rot_Y", end_effector_target_Y, 0.0);
  end_effector_target_rot  = KDL::Rotation::RPY(end_effector_target_R, end_effector_target_P, end_effector_target_Y);
  //node_handle.param("end_effector_target_rot", end_effector_rot,(0, 0.3, 0.4));
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  int rc;
  std::vector<KDL::JntArray> JointList;
  geometry_msgs::Pose target_pose1;
  std::vector<double> target_joints;
  robot_state::RobotState start_state(*move_group.getCurrentState());
  const robot_state::JointModelGroup *joint_model_group =
                start_state.getJointModelGroup(move_group.getName());
  KDL::Vector target_bounds_rot(0, 0, 2* M_PI), target_bounds_vel(0,0,0);
  const KDL::Twist target_bounds(target_bounds_vel, target_bounds_rot);
  std::ifstream inputFile("15mm test.gcode");
  std::string line;
  bool check = 0;
  int second_execution = 0;
  while(ros::ok()){
    while(getline(inputFile, line)){
      std::istringstream get_line(line);
      std::string file_line;
      file_line = get_line.str();
      if(file_line.find('G') < 1){
        if(file_line.find('0') < 2){
          check = 1;
        }
      }
      size_t colon_pos_non = file_line.find(';');
      if(colon_pos_non < 1){
        size_t colon_pos_G = file_line.find('G');
        if(colon_pos_G < 9){
          size_t colon_pos_c = file_line.find('c');
          if(colon_pos_c < 10){
            end_ = ros::WallTime::now();
            double execution_time = (end_ - start_).toNSec() * 1e-9;
            ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
            ROS_INFO_STREAM("second_execution: " << second_execution);
            ros::shutdown();
            return 0;
          }
        }
      }
      else if(check == 1){
        if (file_line.find('G') < 1){
          if(file_line.find('0') < 2 || file_line.find('1') < 2){
            size_t colon_pos_X = file_line.find('X');
            if(colon_pos_X < 100){
              end_effector_target_vol.data[0] = stod(file_line.substr(colon_pos_X+1))*1e-3;
            }
            size_t colon_pos_Y = file_line.find('Y');
            if(colon_pos_Y < 100){
              end_effector_target_vol.data[1] = (stod(file_line.substr(colon_pos_Y+1))*1e-3)+0.25;
            }
            size_t colon_pos_Z = file_line.find('Z');
            if(colon_pos_Z < 100){
              end_effector_target_vol.data[2] = stod(file_line.substr(colon_pos_Z+1))*1e-3;
            }
            KDL::Frame end_effector_pose(end_effector_target_rot, end_effector_target_vol);
            rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result, target_bounds);
            if(rc < 0){
              rc = tracik_solver2.CartToJnt(nominal, end_effector_pose, result, target_bounds);
              second_execution++;
            }
            // Create desired number of valid, random joint configurations
            ROS_INFO_STREAM("rc :" << rc);
            //visual_tools.deleteAllMarkers();
            target_pose1.position.x    = end_effector_pose.p.x();
            target_pose1.position.y    = end_effector_pose.p.y();
            target_pose1.position.z    = end_effector_pose.p.z();
            end_effector_pose.M.GetQuaternion	(target_pose1.orientation.x, target_pose1.orientation.y, target_pose1.orientation.z, target_pose1.orientation.w);

            if (rc >= 0){
              target_joints = move_group.getCurrentJointValues();
              for(int i = 0; i < chain.getNrOfJoints(); i++){
                target_joints.at(i) = result.data(i);
              }
              move_group.setJointValueTarget(target_joints);
              moveit::planning_interface::MoveGroupInterface::Plan my_plan;
              move_group.plan(my_plan);
              move_group.setMaxVelocityScalingFactor(1);
              move_group.setMaxAccelerationScalingFactor(1);
              //visual_tools.publishAxisLabeled(target_pose1, "target_pose1");
              visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
              visual_tools.trigger();
              move_group.move();
              //start_state.setJointGroupPositions(joint_model_group, target_joints);
              //start_state.setFromIK(joint_model_group, target_pose1);
              //move_group.setStartState(start_state);
              //move_group.move();
              //ROS_INFO_STREAM("joint :" << target_joints);
              //current_pose = move_group.getCurrentPose();
              // We can print the name of the reference frame for this robot.
              // also printing the current position and orientation of the robot.
              //ROS_INFO_NAMED("moveo", "x position: %f", current_pose.pose.position.x);
              //ROS_INFO_NAMED("moveo", "y position: %f", current_pose.pose.position.y);
              //ROS_INFO_NAMED("moveo", "z position: %f", current_pose.pose.position.z);
              //ROS_INFO_NAMED("moveo", "x orientation: %f", current_pose.pose.orientation.x);
              //ROS_INFO_NAMED("moveo", "y orientation: %f", current_pose.pose.orientation.y);
              //ROS_INFO_NAMED("moveo", "z orientation: %f", current_pose.pose.orientation.z);
              //ROS_INFO_NAMED("moveo", "w orientation: %f", current_pose.pose.orientation.w);
              //sensor_msgs::JointState moveo_joint_state;
              //std::vector<std::string> name{"moveo_joint1", "moveo_joint2", "moveo_joint3", "moveo_joint4", "moveo_joint5"};
              //std::vector<double> position = move_group.getCurrentJointValues();
              //moveo_joint_state.name = name;
              //moveo_joint_state.position = position;
              //pose_joint_pub.publish(moveo_joint_state);
            }
            else{
              ROS_INFO_STREAM("second_execution: " << second_execution);
              ros::shutdown();
              return 0;
            }
          }
        }
      }
    }
  }
}
