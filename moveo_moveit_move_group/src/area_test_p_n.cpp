#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>//25.5change
#include <moveit/kinematic_constraints/utils.h>

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
                                          // steps * micro_steps * belt * error
  static const double joint_division[6] = {200          * 32  * 10       * 1,     //J 64000
                                           200          * 128 * 5.5      * 1,     //A 140800
                                           19810.111813 * 4   * 4.357143 * 1,     //B 345261.960060921
                                           5370.24793   * 32  * 1        * 1,     //C 171847.93376
                                           1036.36364   * 16  * 4.5      * 1      //D 74618.18208
                                           };
  for(int i = 0; i < 5; i++){
    data(i) = data(i)/(M_PI)*180 * joint_division[i]/360;
  }
}

ros::WallTime start_, end_;
int main(int argc, char **argv)
{
  start_ = ros::WallTime::now();
  ros::init(argc, argv, "area_test_p_n");
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
  KDL::Vector target_bounds_rot(0, 0, 2*M_PI), target_bounds_vel(0,0,0);
  const KDL::Twist target_bounds(target_bounds_vel, target_bounds_rot);

  //-----------------------------
  //Getting Basic Information
  //-----------------------------
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  collision_request.group_name = "arm";

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  KDL::Vector end_effector_target_vol;
  KDL::Rotation end_effector_target_rot = KDL::Rotation::Quaternion(0,0,1,0);
  
  geometry_msgs::Point draw_point;

  std::string gcode_out;
  node_handle.param("gcode_out", gcode_out, std::string("/gcode_out"));

  std::ofstream output_file(gcode_out);
  if(!output_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_out);

  std::vector<KDL::Vector> find_end_effector_target_vol;
  find_end_effector_target_vol.reserve(175);

  std::vector<int> save_place;
  save_place.reserve(175);

  //std::vector<int> use_onecore;
  //use_onecore.reserve(620);

  //std::vector<int> save_use_onecore;
  //save_use_onecore.reserve(620);

  int second_execution = 0;
  while(ros::ok()){
      //end_effector_target_vol.data[0] = 0;
      for(int z = -300;z <= 300;z++){
        end_effector_target_vol.data[2] = z * 0.001;
        //for(int x = 0;x <= 300;x++){
          //end_effector_target_vol.data[0] = x * 0.001;
        //end_effector_target_vol.data[2] = 0;
        for(int y = 81;y <= 255;y++){
          end_effector_target_vol.data[1] = y * 0.001;
          //end_effector_target_vol.data[1] = 0;
          find_end_effector_target_vol.push_back(end_effector_target_vol);
        }
        std::vector<KDL::JntArray> save_result(175);
        omp_set_num_threads(num_threads);
        #pragma omp parallel for
        for(int i = 0;i < 175;i++){
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
            //ROS_INFO_STREAM("Num" << i);
            save_place[i] = 1;
            save_result.at(i) = result;
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
        std::vector< geometry_msgs::Point > save_draw_point(175);
        save_draw_point.reserve(175);
        int times = 0;
        for(int l = 0;l < 175 ;l++){
          if(save_place[l] == 1){
            std::vector<double> joint_values(chain.getNrOfJoints());
            for(int i = 0;i < chain.getNrOfJoints(); i++){
              joint_values.at(i) = save_result.at(l).data(i);
            }
            const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("arm");
            current_state.setJointGroupPositions(joint_model_group, joint_values);
            collision_request.contacts = true;
            collision_request.max_contacts = 1000;
            collision_result.clear();
            planning_scene.checkSelfCollision(collision_request, collision_result);
            if(collision_result.collision == 0){
              motor_setep_convert(save_result.at(l).data);
              if((int(save_result.at(l).data(3)) > 100)){
                if(times > 0){
                  if(find_end_effector_target_vol[l].data[1] == save_draw_point[0].y + 0.001*times){
                    save_draw_point[times].x = find_end_effector_target_vol[l].data[0];
                    save_draw_point[times].y = find_end_effector_target_vol[l].data[1];
                    save_draw_point[times].z = find_end_effector_target_vol[l].data[2];
                  }
                }
                else{
                  save_draw_point[0].x = find_end_effector_target_vol[l].data[0];
                  save_draw_point[0].y = find_end_effector_target_vol[l].data[1];
                  save_draw_point[0].z = find_end_effector_target_vol[l].data[2];
                }
                output_file << save_draw_point[times].x << "," << save_draw_point[times].y << "," << save_draw_point[times].z << std::endl;
                times++;
              }
            }
            else{
              collision_detection::CollisionResult::ContactMap::const_iterator it;
              for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
              {
                ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
              }
            }
            joint_values.clear();
          }
        }
        for(int m = 0;m < 175-times;m++){
          save_draw_point.pop_back();
        }
        if(!save_draw_point.empty()){
          if(save_draw_point.size() == 1){
            save_draw_point.push_back(save_draw_point[0]);
          }
          visual_tools.publishSpheres(save_draw_point, rvt::colors::GREEN, rvt::scales::MEDIUM);
          visual_tools.trigger();
        }
        find_end_effector_target_vol.clear();
        save_place.clear();
        save_draw_point.clear();
        save_result.clear();
      }
    //}
    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-9;
    ROS_INFO_STREAM("Exectution time (s): " << execution_time);
    for(int i = 0; i < num_threads; i++){
      delete tracik_solver[i];
    }
    delete tracik_solver;
    ROS_INFO_STREAM("second_execution: " << second_execution);
    ros::shutdown();
  }
  return 0;
}