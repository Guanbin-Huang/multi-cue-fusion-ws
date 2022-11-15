#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>

Q_DECLARE_METATYPE(std::vector<nav_msgs::Odometry>)
Q_DECLARE_METATYPE(std::vector<Eigen::Vector3d>)

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"flourish_mapping_node");
  
  QApplication a(argc, argv);
  
  qRegisterMetaType<std::vector<nav_msgs::Odometry> >();
  qRegisterMetaType<std::vector<Eigen::Vector3d> >();
  
  MainWindow w;
  
  if(argc==3)
  {
    w.readGraphParamsFromFile(std::string(argv[1])); 
    w.readOptimizerParamsFromFile(std::string(argv[2])); 
  }
  
  w.show();

  return a.exec();
}
