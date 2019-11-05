
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
  namespace rvt = rviz_visual_tools;
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
  ROS_INFO_STREAM("There are threads:" << num_threads);
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
    for (uint j = 0; j < nominal.data.size(); j++)
  {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }
  KDL::Vector target_bounds_rot(0, 0, 2* M_PI), target_bounds_vel(0,0,0);
  const KDL::Twist target_bounds(target_bounds_vel, target_bounds_rot);

  //-----------------------------
  //Getting Basic Information
  //-----------------------------
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  KDL::Vector end_effector_target_vol;
  KDL::Rotation end_effector_target_rot;
  
  geometry_msgs::Point draw_point;

  std::string gcode_out;
  node_handle.param("gcode_out", gcode_out, std::string("/gcode_out"));

  std::ofstream output_file(gcode_out);
  if(!output_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_out);

  std::vector<KDL::Vector> find_end_effector_target_vol;
  find_end_effector_target_vol.reserve(788);

  std::vector<int> save_place;
  save_place.reserve(788);

  //std::vector<int> use_onecore;
  //use_onecore.reserve(620);

  //std::vector<int> save_use_onecore;
  //save_use_onecore.reserve(620);

  int second_execution = 0;
  while(ros::ok()){
    for(int x = 0;x <= 550;x++){
      end_effector_target_vol.data[0] = x * 0.001;
      for(int y = 0;y <= 550;y++){
        end_effector_target_vol.data[1] = y * 0.001;
        for(int z = -182;z <=605;z++){
          end_effector_target_vol.data[2] = z * 0.001;
          find_end_effector_target_vol.push_back(end_effector_target_vol);
        }
        omp_set_num_threads(num_threads);
        #pragma omp parallel for
        for(int i = 0;i < 788;i++){
          int rc = -1;
          int thread_num = omp_get_thread_num();
          KDL::JntArray result;
          KDL::Frame end_effector_pose(end_effector_target_rot, find_end_effector_target_vol[i]);
          rc = tracik_solver[thread_num]->CartToJnt(nominal, end_effector_pose, result, target_bounds);
          if(rc < 0){
            //save_use_onecore[i] = 1;
            save_place[i] = 0;
          }
          else{
            save_place[i] = 1;
            //save_use_onecore[i] = 0;
          }
        }
        //for(int j = 0;j < 620;j++){
        //  if(save_use_onecore[j] == 1){
        //    use_onecore.push_back(j);
        //  }
        //}
        //save_use_onecore.clear();
        //for(int k = 0;k < use_onecore.size();k++){
        //  int rc = -1;
        //  second_execution++;
        //  KDL::JntArray result;
        //  KDL::Frame end_effector_pose(end_effector_target_rot, find_end_effector_target_vol[use_onecore[k]]);
        //  rc = tracik_solver_onecore.CartToJnt(nominal, end_effector_pose, result, target_bounds);
        //  if(rc < 0){
        //    save_place[use_onecore[k]] = 0;
        //  }
        //  else{
        //    save_place[use_onecore[k]] = 1;
        //  }
        //}
        //use_onecore.clear();
        std::vector< geometry_msgs::Point > save_draw_point(788);
        save_draw_point.reserve(788);
        bool save_check = 0;
        int times = 0;
        for(int l = 0;l < 788 ;l++){
          if(save_place[l] == 1){
            output_file << find_end_effector_target_vol[l].data[0] << "," << find_end_effector_target_vol[l].data[1] << "," << find_end_effector_target_vol[l].data[2] << std::endl;
            save_draw_point[times].x = find_end_effector_target_vol[l].data[0];
            save_draw_point[times].y = find_end_effector_target_vol[l].data[1];
            save_draw_point[times].z = find_end_effector_target_vol[l].data[2];
            times++;
          }
        }
        visual_tools.publishSpheres(save_draw_point, rvt::colors::GREEN, rvt::scales::MEDIUM);
        visual_tools.trigger();
        find_end_effector_target_vol.clear();
        save_place.clear();
        save_draw_point.clear();
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