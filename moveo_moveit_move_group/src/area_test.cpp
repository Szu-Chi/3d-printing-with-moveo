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

void motor_steps_convert(Eigen::VectorXd &data){
                                          // steps * micro_steps * belt * error
  static const double joint_division[5] = {200          * 64  * 10       * 1,     //J 128000
                                           200          * 128 * 5.5      * 1,     //A 140800
                                           19810.111813 * 4   * 4.357143 * 1,     //B 345261.960060921
                                           5370.24793   * 32  * 1        * 1,     //C 171847.93376
                                           1036.36364   * 16  * 4.5      * 1      //D 74618.18208
                                           };
  for(int i = 0; i < 5; i++){
    data(i) = data(i)/(M_PI)*180 * joint_division[i]/360;
  }
}

void show_error_point_and_joint(KDL::Vector &end_effector_target_vol, KDL::Chain &chain, KDL::JntArray &result){
  ROS_ERROR_STREAM("===============Position===============");
  ROS_ERROR_STREAM("X : " << end_effector_target_vol.data[0]);
  ROS_ERROR_STREAM("Y : " << end_effector_target_vol.data[1]);
  ROS_ERROR_STREAM("Z : " << end_effector_target_vol.data[2]);
  ROS_ERROR_STREAM("=============Joint Values=============");
  //motor_steps_convert(result.data);
  for(int i = 0;i < chain.getNrOfJoints(); i++){
    static const char joint_code[6] = {'J', 'A', 'B', 'C', 'D'};
    ROS_ERROR_STREAM(joint_code[i] << " : " << result.data(i));
  }
}

bool comp_x(geometry_msgs::Point &a,geometry_msgs::Point &b){
  return a.x < b.x;
}

bool comp_y(geometry_msgs::Point &a,geometry_msgs::Point &b){
  return a.y < b.y;
}

ros::WallTime start_, end_;
int main(int argc, char **argv){
  start_ = ros::WallTime::now();
  ros::init(argc, argv, "area_test_outermost");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //----------------------------
  //Setup
  //----------------------------
  namespace rvt = rviz_visual_tools;
  std::string chain_start, chain_end, urdf_param, urdf_param_turn;
  double timeout,timeout_second,timeout_third,eps;
  int xy, yz, xz;
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
  node_handle.param("urdf_param_turn", urdf_param_turn, std::string("/robot_description2"));
  node_handle.param("eps", eps, 1e-5);
  node_handle.param("xy_plane", xy, 0);
  node_handle.param("yz_plane", yz, 1);
  node_handle.param("xz_plane", xz, 0);
  if((xy && yz) || (yz && xz) || (xy && xz)){
    ROS_ERROR_STREAM("Don't be stupid");
    return -1;
  }
  node_handle.param("num_threads", num_threads, omp_get_num_procs()*2);
  ROS_INFO_STREAM("There are threads:" << num_threads);
  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  // Joint4 doesn't need to turn
  TRAC_IK::TRAC_IK** tracik_solver = new TRAC_IK::TRAC_IK*[num_threads];
  for(int i = 0; i < num_threads; i++){
    tracik_solver[i] = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, eps, TRAC_IK::Speed);
    if(!check_trac_ik_valid(*tracik_solver[i], chain, ll, ul)) return -1;
  }
  TRAC_IK::TRAC_IK tracik_solver_onecore(chain_start, chain_end, urdf_param, timeout_second, eps, TRAC_IK::Speed);
  if(!check_trac_ik_valid(tracik_solver_onecore, chain, ll, ul)) return -1;

  // Joint4 needs to turn
  TRAC_IK::TRAC_IK** tracik_solver_turn = new TRAC_IK::TRAC_IK*[num_threads];
  for(int i = 0; i < num_threads; i++){
    tracik_solver_turn[i] = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param_turn, timeout, eps, TRAC_IK::Speed);
    if(!check_trac_ik_valid(*tracik_solver_turn[i], chain, ll, ul)) return -1;
  }
  TRAC_IK::TRAC_IK tracik_solver_onecore_turn(chain_start, chain_end, urdf_param_turn, timeout_second, eps, TRAC_IK::Speed);
  if(!check_trac_ik_valid(tracik_solver_onecore_turn, chain, ll, ul)) return -1;

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());
    for (uint j = 0; j < nominal.data.size(); j++)
  {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  KDL::Vector target_bounds_rot(0, 0, M_PI*2), target_bounds_vel(0,0,0);
  const KDL::Twist target_bounds(target_bounds_vel, target_bounds_rot);

  // Setting robot collision
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
  
  geometry_msgs::Point draw_point;
  
  std::string outermost_point;
  node_handle.param("outermost_point", outermost_point, std::string("/outermost_point"));
  std::string closest_point;
  node_handle.param("closest_point", closest_point, std::string("/closest_point"));
  std::string all_points;
  node_handle.param("all_points", all_points, std::string("/all_points"));

  std::ofstream outermost_file(outermost_point);
  if(!outermost_file.is_open()) ROS_ERROR_STREAM("Can't open " << outermost_point);
  std::ofstream closest_file(closest_point);
  if(!closest_file.is_open()) ROS_ERROR_STREAM("Can't open " << closest_point);
  std::ofstream all_points_file(all_points);
  if(!all_points_file.is_open()) ROS_ERROR_STREAM("Can't open " << all_points);

  int second_execution = 0;
  while(ros::ok()){
    end_effector_target_vol.data[0] = 0;
    end_effector_target_vol.data[1] = 0;
    end_effector_target_vol.data[2] = 0;
    for(int num_A = 0;num_A <= 442;num_A++){
      if(yz || xz){
        end_effector_target_vol.data[2] = num_A * 0.001;
        if(num_A == 441) num_A = 442;   // Z only 44.1 cm
      }
      else end_effector_target_vol.data[0] = num_A * 0.001;
      std::vector<KDL::Vector> find_end_effector_target_vol;
      find_end_effector_target_vol.reserve(483);
      for(int num_B = 0;num_B <= 482;num_B++){
        if(xz) end_effector_target_vol.data[0] = num_B * 0.001;
        else end_effector_target_vol.data[1] = num_B * 0.001;
        if(xy && num_B == 442) num_B == 482; // If z == 0 then x and y outermost_point is 44.2 cm
        find_end_effector_target_vol.push_back(end_effector_target_vol);
      }
      std::vector<int> save_place(483);
      //std::vector<int> save_use_onecore(483);
      std::vector<KDL::JntArray> save_result(483);
      omp_set_num_threads(num_threads);
      #pragma omp parallel for
      for(int i = 0;i < 483;i++){
        int rc = -1;
        int thread_num = omp_get_thread_num();
        KDL::JntArray result;
        KDL::Rotation end_effector_target_rot;
        // Rotate joint4 when distance <= 35cm  (we test)
        if(sqrt(pow(find_end_effector_target_vol.at(i).data[0]*100,2)+pow(find_end_effector_target_vol.at(i).data[1]*100,2)) <= 35){
          end_effector_target_rot =  KDL::Rotation::Quaternion(0,0,1,0);
          KDL::Frame end_effector_pose(end_effector_target_rot, find_end_effector_target_vol.at(i));
          rc = tracik_solver_turn[thread_num]->CartToJnt(nominal, end_effector_pose, result, target_bounds);
        }
        else{
          end_effector_target_rot =  KDL::Rotation::Quaternion(0,0,0,1);
          KDL::Frame end_effector_pose(end_effector_target_rot, find_end_effector_target_vol.at(i));
          rc = tracik_solver[thread_num]->CartToJnt(nominal, end_effector_pose, result, target_bounds);
        }
        if(rc < 0){
          //save_use_onecore.at(i) = 1;
          save_place.at(i) = 0;
        }
        else{
          save_place.at(i) = 1;
          save_result.at(i) = result;
          //save_use_onecore.at(i) = 0;
        }
      }
      //----------------------
      // Use onecore to slove
      //----------------------
      /*
      std::vector<int> use_onecore;
      use_onecore.reserve(483);
      for(int j = 0;j < 483;j++){
        if(save_use_onecore.at(j) == 1){
          use_onecore.push_back(j);
        }
      }
      save_use_onecore.clear();
      for(int k = 0;k < use_onecore.size();k++){
        int rc = -1;
        second_execution++;
        KDL::JntArray result;
        KDL::Rotation end_effector_target_rot;
        if(sqrt(pow(find_end_effector_target_vol.at(use_onecore.at(k)).data[0]*100,2)+pow(find_end_effector_target_vol.at(use_onecore.at(k)).data[1]*100,2)) <= 35){
          end_effector_target_rot =  KDL::Rotation::Quaternion(0,0,1,0);
          KDL::Frame end_effector_pose(end_effector_target_rot, find_end_effector_target_vol.at(use_onecore.at(k)));
          rc = tracik_solver_onecore_turn.CartToJnt(nominal, end_effector_pose, result, target_bounds);
        }
        else{
          end_effector_target_rot =  KDL::Rotation::Quaternion(0,0,0,1);
          KDL::Frame end_effector_pose(end_effector_target_rot, find_end_effector_target_vol.at(use_onecore.at(k)));
          rc = tracik_solver_onecore.CartToJnt(nominal, end_effector_pose, result, target_bounds);
        }
        if(rc < 0){
          //ROS_ERROR_STREAM("Can't use one core to find joint values");
          //show_error_point_and_joint(find_end_effector_target_vol.at(use_onecore.at(k)), chain, result);
          save_place.at(use_onecore.at(k)) = 0;
        }
        else{
          save_place.at(use_onecore.at(k)) = 1;
          save_result.at(use_onecore.at(k)) = result;
        }
      }
      use_onecore.clear();
      */
      std::vector< geometry_msgs::Point > save_positive_draw_point(483); // Draw y > 35cm
      save_positive_draw_point.reserve(483);
      std::vector< geometry_msgs::Point > save_negative_draw_point(483); // Draw y <= 35cm
      save_negative_draw_point.reserve(483);
      int times_positive = 0;
      int times_negative = 0;
      for(int l = 0;l < 483 ;l++){
        if(save_place.at(l) == 1){
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
            motor_steps_convert(save_result.at(l).data);
            if(save_result.at(l).data(3) <= 2500){
              save_positive_draw_point.at(times_positive).x = find_end_effector_target_vol.at(l).data[0];
              save_positive_draw_point.at(times_positive).y = find_end_effector_target_vol.at(l).data[1];
              save_positive_draw_point.at(times_positive).z = find_end_effector_target_vol.at(l).data[2];
              times_positive++;
            }
            else{
              save_negative_draw_point.at(times_negative).x = find_end_effector_target_vol.at(l).data[0];
              save_negative_draw_point.at(times_negative).y = find_end_effector_target_vol.at(l).data[1];
              save_negative_draw_point.at(times_negative).z = find_end_effector_target_vol.at(l).data[2];
              times_negative++;
            }
          }
          //--------------------------
          // Tell you where collision
          //--------------------------
          /*
          else{
            collision_detection::CollisionResult::ContactMap::const_iterator it;
            for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
            {
              ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
            }
            //show_error_point_and_joint(find_end_effector_target_vol.at(l), chain, save_result.at(l));
          }
          */
          joint_values.clear();
        }
      }
      for(int m = 0;m < 483-times_positive;m++){
        save_positive_draw_point.pop_back();
      }
      for(int n = 0;n < 483-times_negative;n++){
        save_negative_draw_point.pop_back();
      }
      if(!save_positive_draw_point.empty()){
        if(save_positive_draw_point.size() == 1){
          save_positive_draw_point.push_back(save_positive_draw_point.at(0));
        }
        // Joint4 not turn 
        visual_tools.publishSpheres(save_positive_draw_point, rvt::colors::BLUE, rvt::scales::XXXXSMALL);//XXXXSMALL = 0.001
        visual_tools.trigger();
      }
      if(!save_negative_draw_point.empty()){
        if(save_negative_draw_point.size() == 1){
          save_negative_draw_point.push_back(save_negative_draw_point.at(0));
        }
        // Joint4 turn
        visual_tools.publishSpheres(save_negative_draw_point, rvt::colors::GREEN, rvt::scales::XXXXSMALL);//XXXXSMALL = 0.001
        visual_tools.trigger();
      }
      ROS_INFO_STREAM("OUTPUT");
      // Outmost point
      if(save_positive_draw_point.at(times_positive-1).y > save_negative_draw_point.at(times_negative-1).y){
        outermost_file << save_positive_draw_point.at(times_positive-1).x << "," << save_positive_draw_point.at(times_positive-1).y << "," << save_positive_draw_point.at(times_positive-1).z << std::endl;
      }
      else{
        outermost_file << save_negative_draw_point.at(times_negative-1).x << "," << save_negative_draw_point.at(times_negative-1).y << "," << save_negative_draw_point.at(times_negative-1).z << std::endl;
      }
      // Closest point
      if((save_positive_draw_point.at(0).y < save_negative_draw_point.at(0).y && save_positive_draw_point.at(0).y != 0) || save_negative_draw_point.at(0).y == 0){
        closest_file << save_positive_draw_point.at(0).x << "," << save_positive_draw_point.at(0).y << "," << save_positive_draw_point.at(0).z << std::endl;  
      }
      else{
        closest_file << save_negative_draw_point.at(0).x << "," << save_negative_draw_point.at(0).y << "," << save_negative_draw_point.at(0).z << std::endl;  
      }
      // All points
      save_positive_draw_point.insert(save_positive_draw_point.end(), save_negative_draw_point.begin(), save_negative_draw_point.end());
      if(xz) sort(save_positive_draw_point.begin(), save_positive_draw_point.end(), comp_x);
      else sort(save_positive_draw_point.begin(), save_positive_draw_point.end(), comp_y);
      for(int p = 0;p < (times_positive+times_negative); p++){
        all_points_file << save_positive_draw_point.at(p).x << "," << save_positive_draw_point.at(p).y << "," << save_positive_draw_point.at(p).z << std::endl;
      }
      // Clear all
      find_end_effector_target_vol.clear();
      save_positive_draw_point.clear();
      save_negative_draw_point.clear();
      save_place.clear();
      save_result.clear();
    }
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