#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>

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
  int num_samples;
  std::string chain_start, chain_end, urdf_param;
  double timeout,eps;
  int num_threads;
  node_handle.param("num_samples", num_samples, 1000);
  node_handle.param("chain_start", chain_start, std::string(""));
  node_handle.param("chain_end", chain_end, std::string(""));
  node_handle.param("num_threads", num_threads, omp_get_num_procs()*2);
 
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
  node_handle.param("num_threads", num_threads, omp_get_num_procs()*2);
  
  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  TRAC_IK::TRAC_IK** tracik_solver = new TRAC_IK::TRAC_IK*[num_threads];
  for(int i = 0; i < num_threads; i++){
    tracik_solver[i] = new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, timeout, eps, TRAC_IK::Speed);
    if(!check_trac_ik_valid(*tracik_solver[i], chain, ll, ul)) return -1;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO("Using %d joints", chain.getNrOfJoints());
  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());

  //-----------------------------
  //Getting Basic Information
  //-----------------------------

  //-----------------------------
  //Planning to a Pose Goal
  //-----------------------------

  //Plan a motion for this group to a desired pose for end-effector
  // hardcode desired position here before running node in a separate terminal
 
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
 
  

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

  
  ROS_INFO_STREAM("*** Testing TRAC-IK with " << num_samples << " random samples");
  ros::WallTime start, end;
  for(int k = 1; k <= num_threads; k++){
    std::string ofile_name;
    ofile_name = "use_" + std::to_string(k) + "_threads";
    std::ofstream fout(ofile_name);
    if(!fout.is_open())ROS_ERROR_STREAM("Can't open " << ofile_name);
    double totle_time = 0, max_time = 0, min_time = 10;
    uint success = 0;
    omp_set_num_threads(k);
    start = ros::WallTime::now();
    #pragma omp parallel for
    for (uint i = 0; i < num_samples; i++)
    {
      int thread_num = omp_get_thread_num();
      KDL::Frame end_effector_pose;
      KDL::JntArray result;
      int rc;
      KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
      fk_solver.JntToCart(JointList[i], end_effector_pose);
      ros::WallTime start_, end_;
      start_ = ros::WallTime::now();
      rc = tracik_solver[thread_num]->CartToJnt(nominal, end_effector_pose, result);
      end_ = ros::WallTime::now();
      double execution_time = (end_ - start_).toNSec() * 1e-6;
      //ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
      fout << execution_time << std::endl;
      totle_time += execution_time;
      max_time = (execution_time > max_time)?execution_time:max_time;
      min_time = (execution_time < min_time)?execution_time:min_time;
      
      if (rc >= 0){
        success++;
      }
    }
    end = ros::WallTime::now();
    double execution_time = (end - start).toNSec() * 1e-6;
    ROS_INFO_STREAM("threads num : " << k);
    ROS_INFO_STREAM("TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples << "\%)");
    ROS_INFO_STREAM("totle_time (ms): " << execution_time);
    ROS_INFO_STREAM("avg_time (ms): " << totle_time/num_samples);
    ROS_INFO_STREAM("max_time (ms): " << max_time);
    ROS_INFO_STREAM("min_time (ms): " << min_time << std::endl << std::endl);
    fout << "threads num : " << k << std::endl;
    fout << "TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples << "\%)" << std::endl;
    fout << "totle_time (ms): " << execution_time << std::endl;
    fout << "avg_time (ms): " << totle_time/num_samples << std::endl;
    fout << "max_time (ms): " << max_time << std::endl;
    fout << "min_time (ms): " << min_time << std::endl;
    fout.close();
  }
  
  ros::shutdown();  
  return 0;
}
