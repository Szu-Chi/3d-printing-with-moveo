#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>

#include <sstream>
#include <fstream>

#include <time.h>
#include <math.h>

#include <omp.h>
void motor_setep_convert(Eigen::VectorXd &data);
bool check_trac_ik_valid(TRAC_IK::TRAC_IK &tracik_solver,KDL::Chain &chain, KDL::JntArray &ll, KDL::JntArray &ul);

bool check_trac_ik_valid(TRAC_IK::TRAC_IK &tracik_solver,KDL::Chain &chain, KDL::JntArray &ll, KDL::JntArray &ul){
  bool valid = tracik_solver.getKDLChain(chain);
  //ROS_INFO_NAMED("moveo", "TRAC-IK setup");
  if (!valid)
  {
    ROS_ERROR("There was no valid KDL chain found");
    return false;
  }
  valid = tracik_solver.getKDLLimits(ll, ul);
  if (!valid)
  {
    ROS_ERROR("There were no valid KDL joint limits found");
    return false;
  }
  return true;
}

void motor_setep_convert(Eigen::VectorXd &data){
  static const double joint_division[5] = {200*32*10.533*0.95, 200*16*5.71428*0.95, 1028.57143*16*4.523809*0.95, 200*32, 200*32*4.666};
  for(int i = 0; i < 5; i++){
    data(i) = data(i)/(M_PI)*180 * joint_division[i]/360;
  }
}

ros::WallTime start_, end_;
int main(int argc, char **argv)
{
  start_ = ros::WallTime::now();
  ros::init(argc, argv, "gcode_translation");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //----------------------------
  //Setup
  //----------------------------
  std::string chain_start, chain_end, urdf_param;
  double timeout,eps;
  node_handle.param("chain_start", chain_start, std::string(""));
  node_handle.param("chain_end", chain_end, std::string(""));
  if (chain_start == "" || chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }
  node_handle.param("timeout", timeout, 0.005);
  node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
  node_handle.param("eps", eps, 1e-5);
  //ROS_INFO_STREAM("eps :" << eps);
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps, TRAC_IK::Distance);
  TRAC_IK::TRAC_IK tracik_solver2(chain_start, chain_end, urdf_param, timeout*10, eps, TRAC_IK::Distance);
  TRAC_IK::TRAC_IK tracik_solver3(chain_start, chain_end, urdf_param, timeout*20, eps, TRAC_IK::Distance);
  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits
  if(!check_trac_ik_valid(tracik_solver, chain, ll, ul)) return -1;
  if(!check_trac_ik_valid(tracik_solver2, chain, ll, ul)) return -1;
  if(!check_trac_ik_valid(tracik_solver3, chain, ll, ul)) return -1;

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());
  //ROS_INFO("Using %d joints", chain.getNrOfJoints());
  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());
  //-----------------------------
  //Getting Basic Information
  //-----------------------------
  KDL::JntArray result;
  KDL::Vector target_bounds_rot(0, 0, 2* M_PI), target_bounds_vel(0,0,0);
  const KDL::Twist target_bounds(target_bounds_vel, target_bounds_rot);

  KDL::Vector end_effector_target_vol;
  KDL::Rotation end_effector_target_rot;
  
  std::string gcode_in, gcode_out;
  node_handle.param("gcode_in", gcode_in, std::string("/gcode_in"));
  node_handle.param("gcode_out", gcode_out, std::string("/gcode_out"));

  std::ifstream input_file(gcode_in);
  if(!input_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_in);

  std::ofstream output_file(gcode_out);
  if(!output_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_out);
  std::string line;
  bool check = 0;
  int all_line = 0;
  int save_times = 0;
  int second_execution = 0;
  int third_execution = 0;
  int rc = 0;
  int coreNum = omp_get_num_procs();
  ROS_INFO_STREAM("coreNum: " << coreNum);
  while(input_file){
    std::getline(input_file, line);
    if(!line.compare(0,15,";LAYER_COUNT:99")){
      check = 1;
    }
    if(check == 1){
      if(!line.compare(0,2,"G0") || !line.compare(0,2,"G1")){
        size_t colon_pos_X = line.find('X');
        if(colon_pos_X < 100){
          end_effector_target_vol.data[0] = stod(line.substr(colon_pos_X+1))*1e-3;
        }
        size_t colon_pos_Y = line.find('Y');
        if(colon_pos_Y < 100){
          end_effector_target_vol.data[1] = (stod(line.substr(colon_pos_Y+1))*1e-3)+0.30;
        }
        size_t colon_pos_Z = line.find('Z');
        if(colon_pos_Z < 100){
          end_effector_target_vol.data[2] = stod(line.substr(colon_pos_Z+1))*1e-3;
        }
        KDL::Frame end_effector_pose(end_effector_target_rot, end_effector_target_vol);
        rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result, target_bounds);
        if(rc < 0){
          rc = tracik_solver2.CartToJnt(nominal, end_effector_pose, result, target_bounds);
          second_execution++;
          if(rc < 0){
            rc = tracik_solver3.CartToJnt(nominal, end_effector_pose, result, target_bounds);
            third_execution++;
          }
        }
        motor_setep_convert(result.data);
        output_file << line[0] << line[1];
        for(int i = 0;i < chain.getNrOfJoints(); i++){
          static const char joint_code[5] = {'J', 'A', 'B', 'C', 'D'};
          output_file << " " << joint_code[i] << int(result.data(i));
        }
        for(int j = 2;j < line.length(); j++){
          output_file << line[j];
        }
        output_file << std::endl;
      }
      else{
        output_file << line << std::endl;
      }
    }
  }
  end_ = ros::WallTime::now();
  double execution_time = (end_ - start_).toNSec() * 1e-9;
  ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
  ROS_INFO_STREAM("second_execution: " << second_execution);
  ROS_INFO_STREAM("third_execution: " << third_execution);
  ros::shutdown();
  return 0;
}
