#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <math.h>
ros::WallTime start_, end_;
int main(int argc, char **argv)
{
  start_ = ros::WallTime::now();
  ros::init(argc, argv, "simulation");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  std::vector<double> target_joints;
  target_joints = move_group.getCurrentJointValues();
  moveit::core::RobotStatePtr start_state(move_group.getCurrentState());
  const robot_state::JointModelGroup *joint_model_group = start_state->getJointModelGroup(move_group.getName());

                                          // steps * micro_steps * belt * error
  static const double joint_division[5] = {200          * 64  * 10       * 1,     //J 128000
                                           200          * 128 * 5.5      * 1,     //A 140800
                                           19810.111813 * 4   * 4.357143 * 1,     //B 345261.960060921
                                           5370.24793   * 32  * 1        * 1,     //C 171847.93376
                                           1036.36364   * 16  * 4.5      * 1      //D 74618.18208
                                           };

  std::string gcode_in;
  node_handle.param("gcode_in", gcode_in, std::string("/gcode_in"));
  std::ifstream input_file(gcode_in);
  if(!input_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_in);
  moveit_msgs::RobotTrajectory trajectory;
  moveit_msgs::RobotTrajectory trajectory_path;
  std::string line;
  int first = 0;
  //double Z = 0;
  //double pre_Z = 0;
  bool draw = false;
  int save = 0;
  while(input_file){
    save++;
    std::getline(input_file, line);
    if(!line.compare(0,2,"G0") || !line.compare(0,2,"G1")){
      if(line.find('J') != std::string::npos) target_joints.at(0) = double(stod(line.substr(line.find('J')+1))*(2*M_PI)/joint_division[0]);
      if(line.find('A') != std::string::npos) target_joints.at(1) = double(stod(line.substr(line.find('A')+1))*(2*M_PI)/joint_division[1]);
      if(line.find('B') != std::string::npos) target_joints.at(2) = double(stod(line.substr(line.find('B')+1))*(2*M_PI)/joint_division[2]);
      if(line.find('C') != std::string::npos) target_joints.at(3) = double(stod(line.substr(line.find('C')+1))*(2*M_PI)/joint_division[3]);
      if(line.find('D') != std::string::npos) target_joints.at(4) = double(stod(line.substr(line.find('D')+1))*(2*M_PI)/joint_division[4]);
      /*
      if(line.find('Z') != std::string::npos) Z = stod(line.substr(line.find('Z')+1));
      }
      if(Z != pre_Z){
        draw = true;
        pre_Z = Z;
      }
      */
      move_group.setJointValueTarget(target_joints);
      if(first++ > 1){
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group.setPlanningTime(0.05);
        move_group.plan(my_plan);
        if(line.find('E') != std::string::npos){
          if(stod(line.substr(line.find('E')+1)) > 0){
            visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            visual_tools.trigger();
          }
        }
        trajectory.joint_trajectory.joint_names = my_plan.trajectory_.joint_trajectory.joint_names;
        joint_model_group = start_state->getJointModelGroup(move_group.getName());
        start_state->setJointGroupPositions(joint_model_group, target_joints);
        move_group.setStartState(*start_state);
        for(int j = 0; j < my_plan.trajectory_.joint_trajectory.points.size(); j++){
          trajectory.joint_trajectory.points.push_back(my_plan.trajectory_.joint_trajectory.points[j]);
        }
      }
      else{
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group.setPlanningTime(0.05);
        move_group.plan(my_plan);
        move_group.move();
      }
    }
    if(save == 600 || !input_file){
      draw = true;
      save = 0;
    }
    if(draw == true){
      draw = false;
      moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
      robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), "arm");
      rt.setRobotTrajectoryMsg(*move_group.getCurrentState(),trajectory);
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      iptp.computeTimeStamps(rt);
      rt.getRobotTrajectoryMsg(trajectory);
      //std::cout << trajectory;
      joinedPlan.trajectory_ = trajectory;
      if(!move_group.execute(joinedPlan)){
        ROS_ERROR("Failed to execute plan");
        return false;
      }
      trajectory.joint_trajectory.points.clear();
    }
  }
  end_ = ros::WallTime::now();
  double execution_time = (end_ - start_).toNSec() * 1e-9;
  ROS_INFO_STREAM("Exectution time (s): " << execution_time);
  ros::shutdown();
  return 0;
}
