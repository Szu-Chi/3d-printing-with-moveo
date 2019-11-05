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
  static const double joint_division[5] = {200*32*10.533*0.95, 200*16*5.71428*0.95, 1028.57143*16*4.523809*0.95, 200*32, 200*16*4.666};
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
  double timeout,timeout_second,timeout_third,eps;
  int num_threads;
  node_handle.param("chain_start", chain_start, std::string(""));
  node_handle.param("chain_end", chain_end, std::string(""));
  if (chain_start == "" || chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }
  node_handle.param("timeout", timeout, 0.05);
  node_handle.param("timeout_second", timeout_second, 0.1);
  node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
  node_handle.param("eps", eps, 1e-5);
  node_handle.param("num_threads", num_threads, omp_get_num_procs()*2);
  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits
  TRAC_IK::TRAC_IK** tracik_solver = new TRAC_IK::TRAC_IK*[num_threads];
  for(int i = 0; i < num_threads; i++){
    tracik_solver[i] = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, eps, TRAC_IK::Speed);
    if(!check_trac_ik_valid(*tracik_solver[i], chain, ll, ul)) return -1;
  }
  TRAC_IK::TRAC_IK tracik_solver_onecore(chain_start, chain_end, urdf_param, timeout_second, eps, TRAC_IK::Speed);
  if(!check_trac_ik_valid(tracik_solver_onecore, chain, ll, ul)) return -1;

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());
  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());
  //-----------------------------
  //Getting Basic Information
  //-----------------------------
  KDL::Vector target_bounds_rot(0, 0, 2* M_PI), target_bounds_vel(0,0,0);
  const KDL::Twist target_bounds(target_bounds_vel, target_bounds_rot);

  KDL::Vector end_effector_target_vol;
  KDL::Rotation end_effector_target_rot;
  
  std::string gcode_in, gcode_out;
  node_handle.param("gcode_in", gcode_in, std::string("/gcode_in"));
  node_handle.param("gcode_out", gcode_out, std::string("/gcode_out"));

  std::ifstream input_file(gcode_in);
  if(!input_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_in);
  
  std::string line;
  //int line_num = 0;
  //while(input_file){
  //  line_num++;
  //  std::getline(input_file, line);
  //}
  //std::cout << "line_num = " << line_num << std::endl;
  //input_file.close();
  //input_file.open(gcode_in);
  //if(!input_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_in);

  std::ofstream output_file(gcode_out);
  if(!output_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_out);
  bool check = 0;
  int all_line = 0;
  int second_execution = 0;
  std::vector<std::string> save;
  save.reserve(1000);
  std::vector<KDL::JntArray> save_result;
  save_result.reserve(1000);
  std::vector<KDL::Vector> find_end_effector_target_vol;
  find_end_effector_target_vol.reserve(1000);
  std::vector<int> save_place;
  save_place.reserve(1000);
  std::vector<int> use_onecore;
  use_onecore.reserve(1000);
  std::vector<int> save_use_onecore;
  use_onecore.reserve(1000);
  while(ros::ok()){
    while(input_file){
      std::getline(input_file, line);
      save.push_back(line);
      all_line++;
      if(all_line % 1000 == 0 || !input_file){
        for(int i = 0;i < all_line ;i++){
          line = save[i];
          if(!line.compare(0,8,";LAYER:0")){
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
                end_effector_target_vol.data[1] = (stod(line.substr(colon_pos_Y+1))*1e-3)+0.40;
              }
              size_t colon_pos_Z = line.find('Z');
              if(colon_pos_Z < 100){
                end_effector_target_vol.data[2] = (stod(line.substr(colon_pos_Z+1))*1e-3);
              }
              find_end_effector_target_vol.push_back(end_effector_target_vol);
              save_place.push_back(i);
            }
          }
        }
        omp_set_num_threads(num_threads);
        #pragma omp parallel for
        for(int j = 0;j < save_place.size();j++){
          int rc = -1;
          int thread_num = omp_get_thread_num();
          KDL::JntArray result;
          KDL::Frame end_effector_pose(end_effector_target_rot, find_end_effector_target_vol[j]);
          rc = tracik_solver[thread_num]->CartToJnt(nominal, end_effector_pose, result, target_bounds);
          if(rc < 0){
            save_use_onecore[j] = 1;
          }
          else{
            motor_setep_convert(result.data);
            save_result[j] = result;
            save_use_onecore[j] = 0;
          }
        }
        for(int k = 0;k < 1000;k++){
          if(save_use_onecore[k] == 1){
            use_onecore.push_back(k);
          }
        }
        save_use_onecore.clear();
        for(int l = 0;l < use_onecore.size();l++){
          int rc = -1;
          second_execution++;
          KDL::JntArray result;
          KDL::Frame end_effector_pose(end_effector_target_rot, find_end_effector_target_vol[use_onecore[l]]);
          rc = tracik_solver_onecore.CartToJnt(nominal, end_effector_pose, result, target_bounds);
          if(rc < 0){
            ROS_ERROR_STREAM("Threr is no solution found in " << timeout_second << "s");
            ros::shutdown;
            return -1;
          }
          motor_setep_convert(result.data);
          save_result[use_onecore[l]] = result;
        }
        find_end_effector_target_vol.clear();
        use_onecore.clear();
        int pop_out = 0;
        for(int m = 0;m < all_line ;m++){
          line = save[m];
          if((save_place[pop_out] == m) && (pop_out < save_place.size())){
            output_file << line[0] << line[1];
            for(int i = 0;i < chain.getNrOfJoints(); i++){
              static const char joint_code[5] = {'J', 'A', 'B', 'C', 'D'};
              output_file << " " << joint_code[i] << int(save_result[pop_out].data(i));
            }
            for(int j = 2;j < line.length(); j++){
              output_file << line[j];
            }
            output_file << std::endl;
            pop_out++;
          }
          else{
            output_file << line << std::endl;
          }
        }
        save.clear();
        save_place.clear();
        save_result.clear();
        all_line = 0;
      }
    }
    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-9;
    ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
    for(int i = 0; i < num_threads; i++){
      delete tracik_solver[i];
    }
    delete tracik_solver;
    ROS_INFO_STREAM("second_execution: " << second_execution);
    ros::shutdown();
  }
  return 0;
}
