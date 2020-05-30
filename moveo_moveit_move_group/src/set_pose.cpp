#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <include/Joint_division/joint_division.hpp>

tf::Quaternion euler_to_quaternion(double yaw,double pitch,double roll);

void motor_steps_convert(Eigen::VectorXd &data, KDL::Chain &chain, std::vector<double> &joint_division){
  for(int i = 0; i < chain.getNrOfJoints(); i++){
    data(i) = data(i)/(M_PI)*180*joint_division[i]/360;
    ROS_INFO_STREAM("Joint" << (i+1) << " = " << int(data(i)));
  }
}

std::vector<double> solveJoint(KDL::Frame end_effector_pose){
  ros::NodeHandle node_handle("~");
  //init TRAC_IK
  std::string chain_start, chain_end, urdf_param;
  double timeout;
  node_handle.param("chain_start", chain_start, std::string(""));
  node_handle.param("chain_end", chain_end, std::string(""));
  if (chain_start == "" || chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }
  node_handle.param("timeout", timeout, 0.1);
  node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));

  double eps = 1e-5;
  node_handle.param("eps", eps, 1e-5);
  ROS_INFO_STREAM("eps :" << eps);
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps, TRAC_IK::Speed);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  ROS_INFO_NAMED("moveo", "TRAC-IK setup");
  if (!valid)
  {
    ROS_ERROR("There was no valid KDL chain found");
    exit(-1);
  }

  valid = tracik_solver.getKDLLimits(ll, ul);

  if (!valid)
  {
    ROS_ERROR("There were no valid KDL joint limits found");
    exit(-1);
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());
  ROS_INFO("Using %d joints", chain.getNrOfJoints());

  std::string joint_division_name;
  node_handle.param("joint_division", joint_division_name, std::string("/joint_division"));
  std::vector<double> joint_division = read_joint_division(joint_division_name, chain.getNrOfJoints());

  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());
  for (uint j = 0; j < nominal.data.size(); j++){
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }
  //solving IK
  KDL::Vector target_bounds_rot(0, 0, M_PI*5/180), target_bounds_vel(0,0,0);
  const KDL::Twist target_bounds(target_bounds_vel, target_bounds_rot);
  KDL::JntArray result;
  int rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result, target_bounds);
  ROS_INFO_STREAM("rc :" << rc);
  ROS_INFO_STREAM("Timeout :" << timeout);
  if(rc > 0){
    std::vector<double> target_joints(chain.getNrOfJoints());
    for(int i = 0; i < chain.getNrOfJoints(); i++){
        target_joints.at(i) = result.data(i);
    }
    motor_steps_convert(result.data, chain, joint_division);
    return target_joints;
  }else{
    return {};
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

tf::Quaternion euler_to_quaternion(double yaw,double pitch,double roll){
  tf::Quaternion q;
  q.setRPY(yaw,pitch,roll);
  return q;
}

void set_target_pose(const geometry_msgs::PoseStamped& chain_end){
  ros::NodeHandle node_handle("~");

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

  ros::Publisher pose_joint_pub = node_handle.advertise<sensor_msgs::JointState>("pose_joint", 10);
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();


  KDL::Vector end_effector_target_vol;
  end_effector_target_vol.data[0] =chain_end.pose.position.x;
  end_effector_target_vol.data[1] =chain_end.pose.position.y;
  end_effector_target_vol.data[2] =chain_end.pose.position.z;


  KDL::Rotation end_effector_target_rot;
  double end_effector_target_R, end_effector_target_P,end_effector_target_Y;
  end_effector_target_rot  = KDL::Rotation::Quaternion(chain_end.pose.orientation.x, chain_end.pose.orientation.y, chain_end.pose.orientation.z, chain_end.pose.orientation.w);
  /*
  float angle_neg = asin(end_effector_target_vol.data[0] / sqrt(pow(end_effector_target_vol.data[0],2)+pow(end_effector_target_vol.data[1],2)));
  if(end_effector_target_vol.data[1] > 0){
    angle_neg = angle_neg * (-1);
  }
  else{
    angle_neg = angle_neg + M_PI;
  }
  if(sqrt(pow(end_effector_target_vol.data[0]*100,2)+pow(end_effector_target_vol.data[1]*100,2)) <= 35){
    angle_neg = angle_neg + M_PI;
  }
  
  ROS_INFO_STREAM("angle_neg = " << angle_neg);
  tf::Quaternion b = euler_to_quaternion(0.0,0.0,angle_neg);
  ROS_INFO_STREAM("quaternionX = " << b[0]);
  ROS_INFO_STREAM("quaternionY = " << b[1]);
  ROS_INFO_STREAM("quaternionZ = " << b[2]);
  ROS_INFO_STREAM("quaternionW = " << b[3]);
  end_effector_target_rot  = KDL::Rotation::Quaternion(b[0], b[1], b[2], b[3]);

  node_handle.param("end_effector_target_rot", end_effector_rot,(0, 0.3, 0.4));
  */
  KDL::Frame end_effector_pose(end_effector_target_rot, end_effector_target_vol);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  visual_tools.deleteAllMarkers();
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x    = end_effector_pose.p.x();
  target_pose1.position.y    = end_effector_pose.p.y();
  target_pose1.position.z    = end_effector_pose.p.z();
  //end_effector_pose.M.GetQuaternion	(b.at(0), b.at(1), b.at(2), b.at(3));
  end_effector_pose.M.GetQuaternion	(target_pose1.orientation.x, target_pose1.orientation.y, target_pose1.orientation.z, target_pose1.orientation.w);
  visual_tools.publishAxisLabeled(target_pose1, "target_pose1");
  visual_tools.trigger();
  current_pose = move_group.getCurrentPose();

  // Create desired number of valid, random joint configurations
  std::vector<double> target_joints = solveJoint(end_effector_pose);
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

  //Using :planning_scene_interface:'PlanningSceneInterface' class to deal directly with the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  // We can print the name of the reference frame for this robot.
  // also printing the current position and orientation of the robot.
  ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);
  ros::Subscriber eff_pose_sub = node_handle.subscribe("chain_end_pose", 100000, set_target_pose);
  print_current_pose(move_group.getCurrentPose());



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
  while(ros::ok()){
    ros::waitForShutdown();
  }

  ros::shutdown();  
  return 0;
}
