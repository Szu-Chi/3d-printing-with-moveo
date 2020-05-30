#include <ros/package.h>

std::vector<double> read_joint_division(std::string &input_file_name, const unsigned int &chain_nr_of_joints){
  std::vector<double> joint_division(chain_nr_of_joints);
  std::string line;
  std::ifstream input_file(input_file_name);
  if(!input_file.is_open()) ROS_ERROR_STREAM("Can't open " << input_file_name);
  for(auto i = joint_division.begin(); i != joint_division.end();++i){
    std::getline(input_file, line);
    while(!line.compare(0,2,"//")) std::getline(input_file, line);
    std::istringstream templine(line);
	  std::string data;
    double result = 1.0;
	  while (getline(templine, data, ',')) result = result * stod(data);
    *i = result;
    ROS_INFO_STREAM("moveo_joint" << result); //check joint value
  }
  input_file.close();
  return(joint_division);
}