#pragma once

#include <ros/ros.h>
#include <iostream>
#include <unordered_map>
#include <eigen3/Eigen/Dense>

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <ostream>
#include <queue>
#include <string>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <rosgraph_msgs/Clock.h>

class TSS;

typedef Eigen::Matrix<double,6,6> CovarianceMatrixd;

class SensorReadingID {
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  SensorReadingID(){};
  SensorReadingID( const std::string& type, const ros::Time& time ) : topic_type_(type), time_stamp_(time){};
  ~SensorReadingID(){};

  std::string getTopicType(){return topic_type_;};
  ros::Time getTime(){return time_stamp_;};
    
private:
  std::string topic_type_;
  ros::Time time_stamp_;
};


class TemporalNode{
  
  friend class TSS;
  
public:
  TemporalNode(const Eigen::Vector3d& inp_trans, const Eigen::Quaterniond& inp_quat, const ros::Time& time);
  TemporalNode(const Eigen::Isometry3d& pose, const ros::Time& time);
  TemporalNode(const geometry_msgs::Pose& pose, const ros::Time& time);
  TemporalNode(const TemporalNode& other);
  ~TemporalNode();
  
  void addMeasurement(const std::string& name, const SensorReadingID& sensor_reading);
  void addMeasurement(const rosbag::MessageInstance& message);
  void removeMeasurement(const std::string&);
  SensorReadingID getMeasurement(const std::string&);
  void setCovarianceMatrix(const CovarianceMatrixd&);
  
  inline void setId(int idx){id_ = idx;}
  inline Eigen::Isometry3d getPose(){return pose_;};
  inline ros::Time getTime(){return pose_time_;};
  inline Eigen::Vector3d getTranslation(){return pose_.translation(); };
  inline Eigen::Matrix3d getLinear(){return pose_.linear();};
  inline CovarianceMatrixd getCovariance(){return cov_matx_;};
  inline bool hasCovariance(){return has_covariance_;};
  inline int getNodeId(){return id_;};
  inline bool hasMeasurements(){return !sensor_readings_.empty();};
  inline bool hasMeasurement(const std::string& topic_name){return sensor_readings_[topic_name].getTopicType()!="";};
  
private:
  
  void init(const Eigen::Vector3d& inp_trans, const Eigen::Quaterniond& inp_quat, const ros::Time& time);
  
  std::unordered_map<std::string, SensorReadingID> sensor_readings_;
  Eigen::Isometry3d pose_;
  bool has_covariance_;
  CovarianceMatrixd cov_matx_;
  ros::Time pose_time_;
  unsigned int id_;
};







