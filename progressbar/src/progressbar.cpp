#include <ros/ros.h>
#include <qprogressdialog.h>
#include <QApplication>
#include <QString>
#include "std_msgs/Float64MultiArray.h"

int now_value = 0; 
QString a = "N/A";

void receive(const std_msgs::Float64MultiArray& transfre_data){
  double remind = transfre_data.data[2]/1000.0*int(transfre_data.data[0]-transfre_data.data[1]);
  int base = 10;
  int change_min = int(remind/1000/60);
  int change_sec = (int(remind/1000) % 60);
  QString min;
  min.setNum(change_min, base);
  QString sec;
  sec.setNum(change_sec, base);
  a = min + " min " + sec + " sec";
  now_value = int(transfre_data.data[1]/transfre_data.data[0]*10000);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "progressbar");
  ros::NodeHandle node_handle("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Publisher shutdown_pub = node_handle.advertise<std_msgs::Float64MultiArray>("shutdown", 1000);
  ros::Subscriber gcode_translation_sub = node_handle.subscribe("/inverse_kinematics/progress", 100000, receive);
  QApplication app(argc, argv);
  QProgressDialog dialog("Progress", "Cancel", 0, 10000);
  dialog.setWindowTitle("Gcode Translation");
  dialog.setWindowModality(Qt::WindowModal);
  dialog.show();

  while(ros::ok()){
    dialog.setValue(now_value);
    dialog.setLabelText(a);
    QCoreApplication::processEvents();
    if(dialog.wasCanceled()){
      std_msgs::Float64MultiArray push;
      push.data.resize(1);
      push.data[0] = 1;
      shutdown_pub.publish(push);
      break;
    }
    if(now_value == 10000){
      break;
    }
  }
  ros::shutdown();  
  return 0;
}
