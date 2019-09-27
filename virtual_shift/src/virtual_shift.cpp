#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <math.h>

ros::WallTime start_, end_;
int main(int argc, char **argv)
{
  start_ = ros::WallTime::now();
  ros::init(argc, argv, "move_group_1");
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
  robot_state::RobotState start_state(*move_group.getCurrentState());
  const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(move_group.getName());
  bool check = 0;
  
  std::string gcode_in;
  node_handle.param("gcode_in", gcode_in, std::string("/gcode_in"));
  std::ifstream input_file(gcode_in);
  if(!input_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_in);
  
  std::string line;
  while(input_file){
    std::getline(input_file, line);
    if(!line.compare(0,2,"G0")){
      check = 1;
    }
    
    if(!line.compare(0,1,";")){
    }
    else if(check == 1){
      if(!line.compare(0,2,"G0") || !line.compare(0,2,"G1")){
        size_t colon_pos_J = line.find('J');
          if(colon_pos_J < 100){
            target_joints.at(0) = double(stod(line.substr(colon_pos_J+1))*(2*M_PI)/65600);
          }
          size_t colon_pos_A = line.find('A');
          if(colon_pos_A < 100){
            target_joints.at(1) = double(stod(line.substr(colon_pos_A+1))*(2*M_PI)/18000);
          }
          size_t colon_pos_B = line.find('B');
          if(colon_pos_B < 100){
            target_joints.at(2) = double(stod(line.substr(colon_pos_B+1))*(2*M_PI)/144000);
          }
          size_t colon_pos_C = line.find('C');
          if(colon_pos_C < 100){
            target_joints.at(3) = double(stod(line.substr(colon_pos_C+1))*(2*M_PI)/6560);
          }
          size_t colon_pos_D = line.find('D');
          if(colon_pos_D < 100){
            target_joints.at(4) = double(stod(line.substr(colon_pos_D+1))*(2*M_PI)/28800);
          }
          move_group.setJointValueTarget(target_joints);
          moveit::planning_interface::MoveGroupInterface::Plan my_plan;
          move_group.plan(my_plan);
          move_group.setMaxVelocityScalingFactor(1);
          move_group.setMaxAccelerationScalingFactor(1);
          size_t colon_pos_E = line.find('E');
          if(colon_pos_E < 100){
            if(stod(line.substr(colon_pos_E+1)) > 0){
              visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            }
          }
          visual_tools.trigger();
          move_group.move();
      }
    }
  }
  end_ = ros::WallTime::now();
  double execution_time = (end_ - start_).toNSec() * 1e-9;
  ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
  ros::shutdown();
  return 0;
}
