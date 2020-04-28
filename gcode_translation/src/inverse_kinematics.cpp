#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <omp.h>
#include <std_msgs/Float64MultiArray.h>
#include <include/Mesh/circle_new.h>
#include <include/Joint_division/joint_division.hpp>

// Let gocde look more clearly 
int decimal_point(double &A){
  std::string change = std::to_string(A);
  bool point = false;
  for(int i = 0;i<change.size();i++){
    if(change[i] == '.') point = true;
    if(point == true){
      if(change[i+3] == '0'){
        if(change[i+2] == '0'){
          if(change[i+1] == '0') return 0;
          return 1;
        }
        return 2;
      }
      return 3;
    }
  }
}

// Use tf to translate 
tf::Quaternion euler_to_quaternion(double yaw,double pitch,double roll){
  tf::Quaternion q;
  q.setRPY(yaw,pitch,roll);
  return q;
}

// Chrck the basic setting
bool check_trac_ik_valid(TRAC_IK::TRAC_IK &tracik_solver,KDL::Chain &chain, KDL::JntArray &ll, KDL::JntArray &ul){
  bool valid = tracik_solver.getKDLChain(chain);
  //ROS_INFO_NAMED("moveo", "TRAC-IK setup");
  if (!valid){
    ROS_ERROR("There was no valid KDL chain found");
    return false;
  }
  valid = tracik_solver.getKDLLimits(ll, ul);
  if (!valid){
    ROS_ERROR("There were no valid KDL joint limits found");
    return false;
  }
  return true;
}

// Translate the data from angle to steps
void motor_steps_convert(Eigen::VectorXd &data, bool &judge, KDL::Chain &chain, std::vector<double> &joint_division){
  if(!judge){
    for(int i = 0; i < chain.getNrOfJoints(); i++){
      data(i) = data(i)/(M_PI)*180*joint_division[i]/360;
    }
  }
  else{
    for(int i = 0; i < chain.getNrOfJoints(); i++){
      data(i) = data(i)*(M_PI)/180/joint_division[i]*360;
    }
  }
}

// Write output joint values(step) in the gcode
void write_file_joint_value(std::ofstream &output_file, KDL::JntArray &result, KDL::Chain &chain, std::vector<double> &joint_division){
  bool judge = false;
  motor_steps_convert(result.data, judge, chain, joint_division);
  for(int i = 0;i < chain.getNrOfJoints(); i++){
    static const char joint_code[6] = {'J', 'A', 'B', 'C', 'D'};
    output_file << " " << joint_code[i] << int(result.data(i));
  }
}

// If the line is not G1 or G0, that we just write it's original data
void write_file_origin_line(std::ofstream &output_file, std::string &line){
  for(int j = 2;j < line.length(); j++){
    output_file << line[j];
  }
}

// If joint4 has change 180 degrees, we have to creat a new line that the point is the same in gcode.
// That means, we have to rotate the joint4 but, it must stay at the same position
void write_file_new_point(std::ofstream &output_file, KDL::Vector &output_end_effector_target_vol, double &X_offset, double &Y_offset){
  double decimal_x = (output_end_effector_target_vol.data[0]-X_offset)*1e3;
  double decimal_y = (output_end_effector_target_vol.data[1]-Y_offset)*1e3;
  output_file << std::fixed << std::setprecision(decimal_point(decimal_x)) << " X" << decimal_x << std::defaultfloat;
  output_file << std::fixed << std::setprecision(decimal_point(decimal_y)) << " Y" << decimal_y << std::defaultfloat;
}

// Write the position that we give the joint values
void customize_joint(std::ofstream &output_file, KDL::Frame &end_effector_pose){
  double decimal_x = end_effector_pose.p.x()*1000;
  double decimal_y = end_effector_pose.p.y()*1000;
  double decimal_z = end_effector_pose.p.z()*1000;
  output_file << std::fixed << std::setprecision(decimal_point(decimal_x)) << " X" << decimal_x << std::defaultfloat;
  output_file << std::fixed << std::setprecision(decimal_point(decimal_y)) << " Y" << decimal_y << std::defaultfloat;
  output_file << std::fixed << std::setprecision(decimal_point(decimal_z)) << " Z" << decimal_z << std::defaultfloat << " ";
}

// Write customize_joint to gcode
void write_customize_joint(std::ofstream &output_file, std::string &line, KDL::Chain &chain, std::vector<double> &joint_division){
  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::JntArray Joint(chain.getNrOfJoints());
  KDL::Frame end_effector_pose;
  output_file << line[0] << line[1];
  if(line.find('F') != std::string::npos && stoi(line.substr(line.find('F')+1)) != 0) output_file << " F" << stoi(line.substr(line.find('F')+1));
  static const char joint_code[6] = {'J', 'A', 'B', 'C', 'D'};
  for(int i = 0;i < chain.getNrOfJoints();i++){
    Joint(i) = stod(line.substr(line.find(joint_code[i])+1));
    output_file << " " << joint_code[i] << int(Joint(i));
  }
  bool judge = true;
  motor_steps_convert(Joint.data, judge, chain, joint_division);
  fk_solver.JntToCart(Joint, end_effector_pose);
  customize_joint(output_file, end_effector_pose);
  if(line.find(';') != std::string::npos) for(int i = line.find(';');i < line.length();i++) output_file << line[i];
  output_file << std::endl;
}

// If the translation has some error, that we will write it position, joint value and quaternion
void show_error_point_and_joint(KDL::Vector &end_effector_target_vol, KDL::Chain &chain, KDL::JntArray &result, tf::Quaternion &q, int &p_n, std::vector<double> &joint_division){
  ROS_ERROR_STREAM("===============Position===============");
  ROS_ERROR_STREAM("X : " << end_effector_target_vol.data[0]);
  ROS_ERROR_STREAM("Y : " << end_effector_target_vol.data[1]);
  ROS_ERROR_STREAM("Z : " << end_effector_target_vol.data[2]);
  ROS_ERROR_STREAM("=============Joint Values=============");
  //bool judge = false;
  //motor_steps_convert(result.data, judge, chain, joint_division);
  for(int i = 0;i < chain.getNrOfJoints(); i++){
    static const char joint_code[6] = {'J', 'A', 'B', 'C', 'D'};
    ROS_ERROR_STREAM(joint_code[i] << " : " << result.data(i));
  }
  ROS_ERROR_STREAM("==============Quaternion==============");
  ROS_ERROR_STREAM(q[0] << " , " << q[1] << " , " << q[2] << " , " << q[3]);
  ROS_ERROR_STREAM("==================PN==================");
  ROS_ERROR_STREAM(p_n);
}

// Send data to the progressbar
void publisher_to_progressbar(ros::Publisher &translation_pub, int &save_all_line, int &save_all_read_line_count, double &need_times){
  std_msgs::Float64MultiArray push_data;
  push_data.data.resize(3);
  push_data.data[0] = save_all_line;               // gcode all lines
  push_data.data[1] = save_all_read_line_count;    // now we write how many lines
  push_data.data[2] = need_times;                  // writing 1000 lines needs times (average)
  translation_pub.publish(push_data);              // push to progressbar.cpp 
}

// If split finish, that we can start this code
bool start_judge = false;
void start(const std_msgs::Float64MultiArray& receive){
  start_judge = true;
}

ros::WallTime start_, end_;
ros::WallTime start_first_, end_first_;
int main(int argc, char **argv){
  ros::init(argc, argv, "inverse_kinematicks");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher translation_pub = node_handle.advertise<std_msgs::Float64MultiArray>("progress", 1000); // push time needed to progressbar
  ros::Publisher start_pub = node_handle.advertise<std_msgs::Float64MultiArray>("progress_start", 1000); // push start to progressbar
  ros::Publisher respond_split_pub = node_handle.advertise<std_msgs::Float64MultiArray>("respond", 1000);
  ros::Subscriber split_sub = node_handle.subscribe("/gcode_translation/start", 100000, start); // get start from gcode_point_split

  // Wait split finish
  while(!start_judge){
    if(ros::ok()) ros::spinOnce();
    else return -1;
  }
  start_first_ = ros::WallTime::now();

  std_msgs::Float64MultiArray push;
  push.data.resize(1);
  push.data[0] = 1;
  respond_split_pub.publish(push);
  start_pub.publish(push);

  //----------------------------
  //Setup
  //----------------------------
  std::string chain_start, chain_end, urdf_param, urdf_param_turn;
  double timeout,timeout_second,eps;
  double X_offset, Y_offset, Z_offset;
  int num_threads;
  node_handle.param("chain_start", chain_start, std::string(""));
  node_handle.param("chain_end", chain_end, std::string(""));
  if (chain_start == "" || chain_end == ""){
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }
  node_handle.param("timeout", timeout, 0.05);
  node_handle.param("timeout_second", timeout_second, 0.1);
  node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
  node_handle.param("urdf_param_turn", urdf_param_turn, std::string("/robot_description2"));
  node_handle.param("eps", eps, 1e-5);
  node_handle.param("num_threads", num_threads, omp_get_num_procs()*2);
  node_handle.param("X_offset", X_offset, 0.0);
  node_handle.param("Y_offset", Y_offset, 0.0);
  node_handle.param("Z_offset", Z_offset, 0.0);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  //----------------------------
  //TRAC_IK setup
  //----------------------------
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

  TRAC_IK::TRAC_IK tracik_solver_p_n(chain_start, chain_end, urdf_param_turn, timeout_second, eps, TRAC_IK::Speed);
  if(!check_trac_ik_valid(tracik_solver_p_n, chain, ll, ul)) return -1;

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());
  for (uint j = 0; j < nominal.data.size(); j++){
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  //----------------------------
  // Setting robot collision
  //----------------------------
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  collision_request.group_name = "arm";

  // Read joint division
  std::vector<double> joint_division;
  std::string joint_division_name;
  node_handle.param("joint_division", joint_division_name, std::string("/joint_division"));
  joint_division = read_joint_division(joint_division_name, chain.getNrOfJoints());

  std::string line;                     // getline used
  KDL::Vector end_effector_target_vol;  // Read gcode xyz
  // Read all lines of gcode
  std::string gcode_in;
  node_handle.param("gcode_in", gcode_in, std::string("/gcode_in"));
  std::ifstream input_file(gcode_in);
  if(!input_file.is_open()) ROS_ERROR_STREAM("Can't open " << gcode_in);
  int save_all_line = 0;
  bool check_first_point = false;
  while(input_file){
    std::getline(input_file, line);
    if(!check_first_point){
      // Find the first point position
      if((!line.compare(0,2,"G0") || !line.compare(0,2,"G1")) && line.find('X') != std::string::npos && line.find('Y') != std::string::npos && line.find('Z') != std::string::npos){
        check_first_point = true;
        end_effector_target_vol.data[0] = stod(line.substr(line.find('X')+1));
        end_effector_target_vol.data[1] = stod(line.substr(line.find('Y')+1));
        end_effector_target_vol.data[2] = stod(line.substr(line.find('Z')+1))+25;
        save_all_line++;
      }
    }
    save_all_line++;
  }
  input_file.close();
  ROS_INFO_STREAM("Read all lines success");

  // Read gcode
  input_file.open(gcode_in);

  // Write gcode
  std::string gcode_out;
  node_handle.param("gcode_out", gcode_out, std::string("/gcode_out"));
  std::ofstream output_file(gcode_out);
  if(!output_file.is_open()) ROS_ERROR_STREAM("Can't open " << gcode_out);

  std::vector<std::string> save;  // Save original gcode lines
  save.reserve(1000);
  KDL::Vector previous_end_effector_target_vol;
  int previous_save_p_or_n;
  float previous_angle;

  KDL::Vector target_bounds_rot(0, 0, M_PI*2);
  KDL::Vector target_bounds_vel(0,0,0);
  const KDL::Twist target_bounds(target_bounds_vel, target_bounds_rot);

  bool first_point = true;
  int read_line_count = 0;
  int save_count = 0;
  int second_execution = 0;
  int save_all_read_line_count = 0;
  double z_init = 0;
  double save_all_need_times = 0;

  Load_Mesh(11.75);
  Calc_abcd();

  while(input_file){
    if(ros::ok()){
      std::getline(input_file, line);
      // Creat one position that is 25mm taller than first position
      if(!line.compare(0,4,"M400") && check_first_point){
        check_first_point = false;
        std::ostringstream store;
        store << "G1 F2000 X" << end_effector_target_vol.data[0] << " Y" << end_effector_target_vol.data[1] << " Z" << end_effector_target_vol.data[2];
        save.emplace_back(std::move(store.str()));
        store.clear();
        read_line_count++;
      }
      save.emplace_back(std::move(line));
      read_line_count++;
      if(read_line_count % 1000 == 0 || !input_file){ // Read 1000 lines or end of file
        start_ = ros::WallTime::now();
        std::vector<KDL::Vector> find_end_effector_target_vol;  // Filtrate not G1 or G0
        find_end_effector_target_vol.reserve(1000);
        std::vector<float> find_angle;                          // Each point angle
        find_angle.reserve(1000);
        std::vector<int> save_place;                            // Find position of G1 and G0 
        save_place.reserve(1000);
        std::vector<int> save_p_or_n;                           // Save joint4 needs to turn or not 
        save_p_or_n.reserve(1000);
        for(int i = 0;i < read_line_count ;i++){
          line = save[i];
          if((!line.compare(0,2,"G0") || !line.compare(0,2,"G1")) && (line.find('J') == std::string::npos)){
            if(line.find('X') != std::string::npos) end_effector_target_vol.data[0] = (stod(line.substr(line.find('X')+1))*1e-3)+X_offset;
            if(line.find('Y') != std::string::npos) end_effector_target_vol.data[1] = (stod(line.substr(line.find('Y')+1))*1e-3)+Y_offset;
            if(line.find('Z') != std::string::npos) z_init = (stod(line.substr(line.find('Z')+1))*1e-3)+Z_offset;
            
            double z_pos = calc_z_rectangle(end_effector_target_vol.data[0], end_effector_target_vol.data[1], true); // Unit: m -> mm
            if (z_pos == (std::numeric_limits<double>::max)()){
              ROS_INFO_STREAM("Out of rectangle area");
              return -1;
            }
            // 9.18 nozzle high
            end_effector_target_vol.data[2] = z_init + z_pos;
            find_end_effector_target_vol.emplace_back(std::move(end_effector_target_vol));
            save_place.emplace_back(std::move(i));
            // Calculate position angle
            float position_angel = asin(end_effector_target_vol.data[0] / sqrt(pow(end_effector_target_vol.data[0],2)+pow(end_effector_target_vol.data[1],2)));
            
            if(end_effector_target_vol.data[1] > 0) position_angel = position_angel * (-1);
            else position_angel = position_angel + M_PI;
            if(sqrt(pow(end_effector_target_vol.data[0]*100,2)+pow(end_effector_target_vol.data[1]*100,2)) <= 35){
              position_angel = position_angel + M_PI;
              save_p_or_n.emplace_back(std::move(0));
            }
            else save_p_or_n.emplace_back(std::move(1));
            find_angle.emplace_back(std::move(position_angel));
          }
        }
        std::vector<KDL::JntArray> save_result(1000);
        std::vector<int> save_use_onecore(1000);
        // Use paralleled TRAC-IK to calculate
        omp_set_num_threads(num_threads);
        #pragma omp parallel for
        for(int j = 0;j < save_place.size();j++){
          int rc = -1;
          int thread_num = omp_get_thread_num();
          KDL::JntArray result;
          tf::Quaternion q = euler_to_quaternion(0.0,0.0,find_angle[j]);
          KDL::Rotation end_effector_target_rot = KDL::Rotation::Quaternion(q[0],q[1],q[2],q[3]);
          KDL::Frame end_effector_pose(end_effector_target_rot, find_end_effector_target_vol[j]);

          if(save_p_or_n[j] == 0) rc = tracik_solver_turn[thread_num]->CartToJnt(nominal, end_effector_pose, result, target_bounds); // Need turn
          else rc = tracik_solver[thread_num]->CartToJnt(nominal, end_effector_pose, result, target_bounds);

          if(rc < 0) save_use_onecore[j] = 1; // Parallel TRAC-IK can't calculate
          else{
            save_use_onecore[j] = 0;
            save_result[j] = result;
          }
        }
        if(save_use_onecore.size() > 0){
          std::vector<int> use_onecore;
          use_onecore.reserve(1000);
          // Calculate which line need to use one core to calculate 
          for(auto k = save_use_onecore.begin();k < save_use_onecore.end();k++){
            if(*k == 1) use_onecore.emplace_back(std::move(*k));
          }
          save_use_onecore.clear();
          save_use_onecore.shrink_to_fit();
          // Use one core to calculate
          for(auto l = use_onecore.begin();l < use_onecore.end();l++){
            int rc = -1;
            second_execution++;
            KDL::JntArray result;
            tf::Quaternion q = euler_to_quaternion(0.0,0.0,find_angle[*l]);
            KDL::Rotation end_effector_target_rot = KDL::Rotation::Quaternion(q[0],q[1],q[2],q[3]);
            KDL::Frame end_effector_pose(end_effector_target_rot, find_end_effector_target_vol[*l]);

            if(save_p_or_n[*l] == 0) rc = tracik_solver_onecore_turn.CartToJnt(nominal, end_effector_pose, result, target_bounds);
            else rc = tracik_solver_onecore.CartToJnt(nominal, end_effector_pose, result, target_bounds);

            if(rc < 0){
              ROS_ERROR_STREAM("Can't use one core to find joint values");
              show_error_point_and_joint(find_end_effector_target_vol[*l], chain, result, q, save_p_or_n[*l], joint_division);
              return -1;
            }
            save_result[*l] = result;
          }
          use_onecore.clear();
          use_onecore.shrink_to_fit();
        }
        int pop_out = 0;
        // Write gcode
        for(int m = 0;m < read_line_count ;m++){
          line = save[m];
          // Translate the joint values to cartesian coordinates
          if((!line.compare(0,2,"G0") || !line.compare(0,2,"G1")) && (line.find('J') != std::string::npos)) write_customize_joint(output_file, line, chain, joint_division);
          else if(save_place.size() - pop_out > 0){
            if(save_place[pop_out] == m){
              output_file << line[0] << line[1];
              std::vector<double> joint_values(chain.getNrOfJoints());
              for(int i = 0;i < chain.getNrOfJoints(); i++){
                joint_values[i] = save_result[pop_out].data(i);
              }
              // Judge collision
              const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("arm");
              current_state.setJointGroupPositions(joint_model_group, joint_values);
              collision_request.contacts = true;
              collision_request.max_contacts = 1000;
              collision_result.clear();
              planning_scene.checkSelfCollision(collision_request, collision_result);
              if(collision_result.collision == 0){
                if(pop_out > 0){
                  previous_save_p_or_n = save_p_or_n[pop_out-1];
                  previous_end_effector_target_vol = find_end_effector_target_vol[pop_out-1];
                  previous_angle = find_angle[pop_out-1];
                }
                // If joint4 turns back, we need to add a point
                if(first_point == false && (previous_save_p_or_n != save_p_or_n[pop_out])){
                  int rc = -1;
                  KDL::JntArray result;
                  KDL::Rotation end_effector_target_rot;
                  KDL::Vector output_end_effector_target_vol;
                  tf::Quaternion q;
                  // chose one point to let joint4 turn back
                  if(save_p_or_n[pop_out] == 0){
                    output_end_effector_target_vol = previous_end_effector_target_vol;
                    q = euler_to_quaternion(0.0,0.0,(previous_angle+M_PI));
                  }
                  else{
                    output_end_effector_target_vol = find_end_effector_target_vol[pop_out];
                    q = euler_to_quaternion(0.0,0.0,(find_angle[pop_out]+M_PI));
                  }
                  end_effector_target_rot = KDL::Rotation::Quaternion(q[0],q[1],q[2],q[3]);
                  KDL::Frame end_effector_pose(end_effector_target_rot, output_end_effector_target_vol);
                  rc = tracik_solver_p_n.CartToJnt(nominal, end_effector_pose, result, target_bounds);
                  if(rc < 0){
                    ROS_ERROR_STREAM("Turn back can't find joint values");
                    show_error_point_and_joint(output_end_effector_target_vol, chain, result, q, save_p_or_n[pop_out], joint_division);
                    return -1;
                  }
                  else{
                    write_file_joint_value(output_file, result, chain, joint_division);
                    if(save_p_or_n[pop_out] == 0){
                      write_file_new_point(output_file, output_end_effector_target_vol, X_offset, Y_offset);
                      output_file << std::endl;
                      output_file << line[0] << line[1];
                      write_file_joint_value(output_file, save_result[pop_out], chain, joint_division);
                      write_file_origin_line(output_file, line);
                    }
                    else{
                      write_file_origin_line(output_file, line);
                      output_file << std::endl;
                      output_file << line[0] << line[1];
                      write_file_joint_value(output_file, save_result[pop_out], chain, joint_division);
                      write_file_new_point(output_file, output_end_effector_target_vol, X_offset, Y_offset);
                    }
                    output_file << std::endl;
                  }
                }
                else{
                  write_file_joint_value(output_file, save_result[pop_out], chain, joint_division);
                  write_file_origin_line(output_file, line);
                  output_file << std::endl;
                }
                if(pop_out <= save_place.size() - 1){
                  pop_out++;
                  first_point = false;
                }
              }
              else{
                collision_detection::CollisionResult::ContactMap::const_iterator it;
                for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it){
                  ROS_ERROR_STREAM("Contact between : " << it->first.first.c_str() << " and " << it->first.second.c_str());
                }
                tf::Quaternion q = euler_to_quaternion(0.0,0.0,find_angle[pop_out]);
                show_error_point_and_joint(find_end_effector_target_vol[pop_out], chain, save_result[pop_out], q, save_p_or_n[pop_out], joint_division);
                return -1;
              }
            }
            else output_file << line << std::endl;
          }
          else output_file << line << std::endl;
        } 
        end_ = ros::WallTime::now();
        save_count++;
        double execution_time = (end_ - start_).toNSec() * 1e-6;
        // Push to progressbar
        save_all_need_times = save_all_need_times + execution_time;
        save_all_read_line_count = save_all_read_line_count + read_line_count;
        double average_times = save_all_need_times/save_count;
        if(save_all_read_line_count == save_all_line) save_all_read_line_count--;
        publisher_to_progressbar(translation_pub, save_all_line, save_all_read_line_count, average_times);
        // Clear all and save the last point of these 1000 lines
        if(save_place.size() > 0){
          previous_end_effector_target_vol = find_end_effector_target_vol[pop_out-1];
          previous_angle = find_angle[pop_out-1];
          previous_save_p_or_n = save_p_or_n[pop_out-1];
        }
        find_end_effector_target_vol.clear();
        find_angle.clear();
        save.clear();
        save_p_or_n.clear();
        save_place.clear();
        save_result.clear();
        read_line_count = 0;
      }
    }
    else return -1;
  }
  end_first_ = ros::WallTime::now();
  double execution_time_first = (end_first_ - start_first_).toNSec() * 1e-9;
  ROS_INFO_STREAM("Exectution time (s): " << execution_time_first);
  // Translation finish
  for(int i = 0; i < num_threads; i++){
    delete tracik_solver[i];
  }
  delete tracik_solver;
  joint_division.clear();
  ROS_INFO_STREAM("second_execution: " << second_execution);
  input_file.close();
  output_file.close();
  // Write success file for cura
  std::string check_success;
  node_handle.param("check_success", check_success, std::string("/check_success"));
  output_file.open(check_success);
  if(!output_file.is_open()) ROS_ERROR_STREAM("Can't open " << check_success);
  output_file << "Success";
  output_file.close();
  // Wait for all steps finish
  while(ros::ok()){
    double average_times = 0.0;
    publisher_to_progressbar(translation_pub, save_all_line, save_all_line, average_times);
    ros::spinOnce();
  }
  return 0;
}