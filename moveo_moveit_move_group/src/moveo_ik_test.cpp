#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>

double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

int main(int argc, char **argv)
{	
  ros::init(argc, argv, "ik_test");
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

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  //move_group.setEndEffectorLink("moveo_link5");
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  // We can print the name of the reference frame for this robot.
  // also printing the current position and orientation of the robot.
  ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);
  ROS_INFO_NAMED("moveo", "x position: %f", current_pose.pose.position.x);
  ROS_INFO_NAMED("moveo", "y position: %f", current_pose.pose.position.y);
  ROS_INFO_NAMED("moveo", "z position: %f", current_pose.pose.position.z);
  ROS_INFO_NAMED("moveo", "x orientation: %f", current_pose.pose.orientation.x);
  ROS_INFO_NAMED("moveo", "y orientation: %f", current_pose.pose.orientation.y);
  ROS_INFO_NAMED("moveo", "z orientation: %f", current_pose.pose.orientation.z);
  ROS_INFO_NAMED("moveo", "w orientation: %f", current_pose.pose.orientation.w);
 
  int num_samples;
  std::string chain_start, chain_end, urdf_param;
  double timeout;
  node_handle.param("num_samples", num_samples, 1000);
  node_handle.param("chain_start", chain_start, std::string(""));
  node_handle.param("chain_end", chain_end, std::string(""));
  if (chain_start == "" || chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }
  node_handle.param("timeout", timeout, 0.005);
  node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
  double eps = 1e-5;

  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
  
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
  visual_tools.publishText(text_pose, "MoveGroupInterface Moveo Demo", rvt::WHITE, rvt::XLARGE);

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
 
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 
  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  int rc;
 

  // Create desired number of valid, random joint configurations
  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());

  for (uint i = 0; i < num_samples; i++)
  {
    for (uint j = 0; j < ll.data.size(); j++)
    {
      q(j) = fRand(ll(j), ul(j));
    }
    JointList.push_back(q);
  }

  uint success = 0;

  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  ROS_INFO_STREAM("*** Testing TRAC-IK with " << num_samples << " random samples");

  for (uint i = 0; i < num_samples; i++)
  {
    fk_solver.JntToCart(JointList[i], end_effector_pose);
    rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
    if (rc >= 0){
      success++;
      visual_tools.deleteAllMarkers();
      geometry_msgs::Pose target_pose1;
      target_pose1.position.x    = end_effector_pose.p.x();
      target_pose1.position.y    = end_effector_pose.p.y();
      target_pose1.position.z    = end_effector_pose.p.z();
      end_effector_pose.M.GetQuaternion	(target_pose1.orientation.x, target_pose1.orientation.y, target_pose1.orientation.z, target_pose1.orientation.w);
      visual_tools.publishAxisLabeled(target_pose1, "target_pose");
      visual_tools.trigger();
 
    
      ROS_INFO_STREAM("end_effector_pose :" << end_effector_pose.p.x() << " " << end_effector_pose.p.y() << " " << end_effector_pose.p.z());
      ROS_INFO_STREAM("end_effector_rotation :" << target_pose1.orientation.x << " " << target_pose1.orientation.y << " " << target_pose1.orientation.z << " " << target_pose1.orientation.w );
      std::vector<double> target_joints;
      target_joints = move_group.getCurrentJointValues();
      for(int i = 0; i < chain.getNrOfJoints(); i++){
        target_joints.at(i) = result.data(i);
      }
      move_group.setJointValueTarget(target_joints);
      move_group.move();
    }
    
    if (int((double)i / num_samples * 100) % 10 == 0)
      ROS_INFO_STREAM_THROTTLE(1, int((i) / num_samples * 100) << "\% done");
  }
  ROS_INFO_STREAM("TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples << "\%)");
  
  ros::shutdown();  
  return 0;
}
