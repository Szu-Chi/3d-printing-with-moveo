#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <time.h>
#include <math.h>
#include <qprogressdialog.h>
#include <QApplication>
#include <QString>
#include "std_msgs/Float64MultiArray.h"

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

bool send_judge = false;
void close(const std_msgs::Float64MultiArray& receive){
  send_judge = true;
}

ros::WallTime start_, end_;
int main(int argc, char **argv){
  start_ = ros::WallTime::now();
  ros::init(argc, argv, "gcode_point_split");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher IK_pub = node_handle.advertise<std_msgs::Float64MultiArray>("start", 1000);
  ros::Subscriber inverse_kinematics_respond_sub = node_handle.subscribe("/inverse_kinematics/respond", 100000, close);
  //----------------------------
  //Setup
  //----------------------------
  bool split;
  std::string gcode_in, register_gcode_out;
  node_handle.param("gcode_in", gcode_in, std::string("/gcode_in"));
  node_handle.param("register_gcode_out", register_gcode_out, std::string("/register_gcode_out"));
  node_handle.param("split", split, true);

  int all_lines = 0;
  std::string line;
  std::ifstream input_file(gcode_in);
  if(!input_file.is_open()) ROS_ERROR_STREAM("Can't open " <<gcode_in);
  while(input_file){
    std::getline(input_file, line);
    all_lines++;
  }
  input_file.close();
  input_file.open(gcode_in);

  // Write check_success
  std::string check_success;
  node_handle.param("check_success", check_success, std::string("/check_success"));
  std::ofstream output_file(check_success);
  if(!output_file.is_open()) ROS_ERROR_STREAM("Can't open " << check_success);
  output_file << "Error";
  output_file.close();

  ROS_INFO_STREAM("Write check success");
  output_file.open(register_gcode_out);
  if(!output_file.is_open()) ROS_ERROR_STREAM("Can't open " << register_gcode_out);
  
  QApplication app(argc, argv);
  QProgressDialog dialog("Connecting", "Cancel", 0, all_lines);
  dialog.setWindowTitle("Connecting");
  dialog.setWindowModality(Qt::WindowModal);
  dialog.show();
  dialog.setLabelText("Connecting to Gcode Translation");

  int now_line = 0;
  bool check_distance = false;
  double x = 0;
  double y = 0;
  double z = 0;
  double E = 0;
  double pre_x = 0;
  double pre_y = 0;
  double pre_z = 0;
  double pre_E = 0;
  while(input_file){
    if(ros::ok()){
      std::getline(input_file, line);
      now_line++;
      if(!line.compare(0,6,"G92 E0")) pre_E = 0;
      if((!line.compare(0,2,"G0") || !line.compare(0,2,"G1"))){
        if(line.find('X') != std::string::npos) x = stod(line.substr(line.find('X')+1));
        if(line.find('Y') != std::string::npos) y = stod(line.substr(line.find('Y')+1));
        if(line.find('Z') != std::string::npos) z = stod(line.substr(line.find('Z')+1));
        if(line.find('E') != std::string::npos) E = stod(line.substr(line.find('E')+1));

        if(check_distance == true){
          output_file << line[0] << line[1];
          double X_diff = (x - pre_x);
          double Y_diff = (y - pre_y);
          double E_diff = (E - pre_E);
          int cut_part = ceil(sqrt(pow(X_diff,2)+pow(Y_diff,2))/10);
          if(cut_part > 1 && split){
            std::ostringstream store;
            bool check_inside = false;
            if(line.find('F') != std::string::npos && stoi(line.substr(line.find('F')+1)) != 0) output_file << " F" << stoi(line.substr(line.find('F')+1));
            for(int i = 1; i < cut_part; i++){
              pre_x += X_diff/cut_part;
              pre_y += Y_diff/cut_part;
              pre_E += E_diff/cut_part;
              if(sqrt(pow(pre_x,2)+pow(pre_y,2)) < 175){
                check_inside = true;
                break;
              }
              store << std::fixed << std::setprecision(decimal_point(pre_x)) << " X" << pre_x << std::defaultfloat;
              store << std::fixed << std::setprecision(decimal_point(pre_y)) << " Y" << pre_y << std::defaultfloat;
              if(line.find('E') != std::string::npos && stod(line.substr(line.find('E')+1)) != 0) store << std::fixed << std::setprecision(5) << " E" << pre_E << std::defaultfloat;
              store << std::endl;
              store << line[0] << line[1];
            }
            if(!check_inside) output_file << store.str();
            store.clear();
            if(line.find('F') != std::string::npos){
              if(line.find('X') != std::string::npos) output_file << std::fixed << std::setprecision(decimal_point(x)) << " X" << x << std::defaultfloat;
              if(line.find('Y') != std::string::npos) output_file << std::fixed << std::setprecision(decimal_point(y)) << " Y" << y << std::defaultfloat;
              if(line.find('Z') != std::string::npos) output_file << std::fixed << std::setprecision(decimal_point(z)) << " Z" << z << std::defaultfloat;
              if(line.find('E') != std::string::npos) output_file << std::fixed << std::setprecision(5) << " E" << E << std::defaultfloat;
            }
            else{
              for(int j = 2;j < line.length(); j++){
                output_file << line[j];
              }
            }
          }
          else{
            for(int j = 2;j < line.length(); j++){
              output_file << line[j];
            }    
          }
          output_file << std::endl;
        }
        else output_file << line << std::endl;
        pre_x = x;
        pre_y = y;
        pre_E = E;
        check_distance = true;
      }
      else output_file << line << std::endl;
      dialog.setValue(now_line);
      QCoreApplication::processEvents();
      if(dialog.wasCanceled()) ros::shutdown();
    }
    else return -1;
  }
  end_ = ros::WallTime::now();
  double execution_time = (end_ - start_).toNSec() * 1e-9;
  ROS_INFO_STREAM("Exectution time (s): " << execution_time);
  input_file.close();
  output_file.close();
  // Set transfer information
  std_msgs::Float64MultiArray push;
  push.data.resize(1);
  push.data[0] = 1;
  // Wait for all steps finish
  ros::Rate loop_rate(10);
  while(ros::ok()){
    // Inverse_kinematics start
    if(!send_judge) IK_pub.publish(push);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}