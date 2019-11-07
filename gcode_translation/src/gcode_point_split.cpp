#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <time.h>
#include <math.h>
int decimal_point(double &A){
  std::string change = std::to_string(A);
  bool point = 0;
  for(int i = 0;i<change.size();i++){
    if(change[i] == '.'){
      point = 1;
    }
    if(point == 1){
      if(change[i+3] == '0'){
        if(change[i+2] == '0'){
          if(change[i+1] == '0'){
            return 0;
          }
          return 1;
        }
        return 2;
      }
      return 3;
    }
  }
}
ros::WallTime start_, end_;
int main(int argc, char **argv){
  start_ = ros::WallTime::now();
  ros::init(argc, argv, "gcode_translation");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //----------------------------
  //Setup
  //----------------------------

  std::string gcode_in, gcode_out;
  node_handle.param("gcode_in", gcode_in, std::string("/gcode_in"));
  node_handle.param("gcode_out", gcode_out, std::string("/gcode_out"));

  std::ifstream input_file(gcode_in);
  if(!input_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_in);

  std::ofstream output_file(gcode_out);
  if(!output_file.is_open())ROS_ERROR_STREAM("Can't open " <<gcode_out);
  
  std::string line;
  bool check = 0;
  bool check_distance = 0;
  int second_execution = 0;
  double x = 0;
  double y = 0;
  double z = 0;
  double pre_x = 0;
  double pre_y = 0;
  double pre_z = 0;
  double F_out = 0;
  double E_out = 0;
  double pre_E_out = 0;
  while(input_file){
    std::getline(input_file, line);
    if(!line.compare(0,2,"G0") && check ==0){
      check = 1;
      output_file << line << std::endl;
    }
    if(check == 1 && (!line.compare(0,2,"G0") || !line.compare(0,2,"G1"))){
      size_t colon_pos_X = line.find('X');
      if(colon_pos_X < 100){
        x = stod(line.substr(colon_pos_X+1));
      }
      size_t colon_pos_Y = line.find('Y');
      if(colon_pos_Y < 100){
        y = stod(line.substr(colon_pos_Y+1));
      }
      size_t colon_pos_Z = line.find('Z');
      if(colon_pos_Z < 100){
        z = stod(line.substr(colon_pos_Z+1));
      }
      size_t colon_pos_E = line.find('E');
      if(colon_pos_E < 100){
        if(stod(line.substr(colon_pos_E+1)) != 0){
          E_out = stod(line.substr(colon_pos_E+1));
        }
      }
      if(check_distance == 1){
        output_file << line[0] << line[1];
        double X_diff = (x - pre_x);
        double Y_diff = (y - pre_y);
        double E_diff = E_out - pre_E_out;
        int cut_part = 1;
        while(sqrt(pow(X_diff/cut_part,2)+pow(Y_diff/cut_part,2)) > 15){
          cut_part++;
        }
        if(cut_part > 1){
          size_t colon_pos_F = line.find('F');
          if(colon_pos_F < 100){
            if(stoi(line.substr(colon_pos_F+1)) != 0){
              output_file << " F" << stoi(line.substr(colon_pos_F+1));
            }
          }
          for(int i = 1; i < cut_part; i++){
            pre_x += X_diff/cut_part;
            pre_y += Y_diff/cut_part;
            pre_E_out += E_diff/cut_part;
            output_file << std::fixed << std::setprecision(decimal_point(pre_x)) << " X" << pre_x << std::defaultfloat;
            output_file << std::fixed << std::setprecision(decimal_point(pre_y)) << " Y" << pre_y << std::defaultfloat;
            if(colon_pos_E < 100){
              if(stod(line.substr(colon_pos_E+1)) != 0){
                output_file << std::fixed << std::setprecision(5) << " E" << pre_E_out << std::defaultfloat;
              }
            }
            output_file << std::endl;
            output_file << line[0] << line[1];
          }
          if(line.find('F') < 100){
            if(colon_pos_X < 100){
              output_file << std::fixed << std::setprecision(decimal_point(x)) << " X" << x << std::defaultfloat;
            }
            if(colon_pos_Y < 100){
              output_file << std::fixed << std::setprecision(decimal_point(y)) << " Y" << y << std::defaultfloat;
            }
            if(colon_pos_Z < 100){
              output_file << std::fixed << std::setprecision(decimal_point(z)) << " Z" << z << std::defaultfloat;
            }  
            if(colon_pos_E < 100){
              if(stod(line.substr(colon_pos_E+1)) != 0){
                output_file << std::fixed << std::setprecision(5) << " E" << E_out << std::defaultfloat;
              }
            }
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
      pre_x = x;
      pre_y = y;
      pre_E_out = E_out;
      check_distance = 1;
    }
    else{
      output_file << line << std::endl;
    }
  }
  end_ = ros::WallTime::now();
  double execution_time = (end_ - start_).toNSec() * 1e-9;
  ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
  input_file.close();
  output_file.close();
  ros::shutdown();
  return 0;
}