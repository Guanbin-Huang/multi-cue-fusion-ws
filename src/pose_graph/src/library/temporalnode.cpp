#include <pose_graph/temporalnode.h>

using namespace std;

TemporalNode::TemporalNode( const Eigen::Vector3d& inp_trans, const Eigen::Quaterniond& inp_quat,
                            const ros::Time& time)
{
    init(inp_trans,inp_quat, time);
    id_=-1;
}

TemporalNode::TemporalNode(const Eigen::Isometry3d& pose, const ros::Time& time) 
{
  Eigen::Vector3d in_tr=pose.translation();
  Eigen::Quaterniond in_q=Eigen::Quaterniond(pose.linear());
  init(in_tr,in_q, time);
  id_=-1;
}

TemporalNode::TemporalNode(const geometry_msgs::Pose& pose, const ros::Time& time) 
{
  Eigen::Vector3d position(pose.position.x,
                           pose.position.y,
                           pose.position.z);
  Eigen::Quaterniond orientation(pose.orientation.w,
                                 pose.orientation.x,
                                 pose.orientation.y,
                                 pose.orientation.z);
  init(position,orientation, time);
  id_=-1;
}

TemporalNode::TemporalNode(const TemporalNode& other)
{
  pose_=other.pose_;
  has_covariance_=other.has_covariance_;
  cov_matx_=other.cov_matx_;
  pose_time_=other.pose_time_;
  sensor_readings_=other.sensor_readings_;
  id_=other.id_;
}

void TemporalNode::init( const Eigen::Vector3d& inp_trans, const Eigen::Quaterniond& inp_quat,
                         const ros::Time& time)
{
  pose_.linear() = inp_quat.toRotationMatrix();
  pose_.translation() = inp_trans;
  has_covariance_ = false;
  cov_matx_.setZero();
  pose_time_ = time;
}

TemporalNode::~TemporalNode(){}

void TemporalNode::addMeasurement(const std::string& name, const SensorReadingID& sensor_reading){

//   sensor_readings_.insert( std::pair<std::string, SensorReadingID>(name, sensor_reading) );
  sensor_readings_[name]=sensor_reading;
}

void TemporalNode::removeMeasurement(const std::string& name){
  sensor_readings_.erase(name);
}

SensorReadingID TemporalNode::getMeasurement(const std::string& name){
  return sensor_readings_.at(name);
}

void TemporalNode::setCovarianceMatrix(const CovarianceMatrixd& cov){
  cov_matx_ = cov;
  has_covariance_ = true;
}

void TemporalNode::addMeasurement(const rosbag::MessageInstance& message){
  string topic_name = message.getTopic();
  string topic_type = message.getDataType();

  
  if (topic_type == "rosgraph_msgs/Clock") {
    rosgraph_msgs::Clock::ConstPtr msg_ptr =  message.instantiate<rosgraph_msgs::Clock>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->clock);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "tf2_msgs/TFMessage") {
    tf2_msgs::TFMessage::ConstPtr msg_ptr =  message.instantiate<tf2_msgs::TFMessage>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->transforms[0].header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  
  
  if (topic_type == "nav_msgs/Odometry") {
    nav_msgs::Odometry::ConstPtr msg_ptr =  message.instantiate<nav_msgs::Odometry>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "sensor_msgs/Image") {
    sensor_msgs::Image::ConstPtr msg_ptr =  message.instantiate<sensor_msgs::Image>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "sensor_msgs/NavSatFix") {
    sensor_msgs::NavSatFix::ConstPtr msg_ptr =  message.instantiate<sensor_msgs::NavSatFix>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "sensor_msgs/Imu") {
    sensor_msgs::Imu::ConstPtr msg_ptr =  message.instantiate<sensor_msgs::Imu>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "sensor_msgs/CameraInfo") {
    sensor_msgs::CameraInfo::ConstPtr msg_ptr =  message.instantiate<sensor_msgs::CameraInfo>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "sensor_msgs/LaserScan") {
    sensor_msgs::LaserScan::ConstPtr msg_ptr =  message.instantiate<sensor_msgs::LaserScan>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "sensor_msgs/PointCloud") {
    sensor_msgs::PointCloud::ConstPtr msg_ptr =  message.instantiate<sensor_msgs::PointCloud>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "sensor_msgs/PointCloud2") {
    sensor_msgs::PointCloud2::ConstPtr msg_ptr =  message.instantiate<sensor_msgs::PointCloud2>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "geometry_msgs/PointStamped") {
    geometry_msgs::PointStamped::ConstPtr msg_ptr =  message.instantiate<geometry_msgs::PointStamped>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "geometry_msgs/PoseStamped") {
    geometry_msgs::PoseStamped::ConstPtr msg_ptr =  message.instantiate<geometry_msgs::PoseStamped>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "geometry_msgs/PoseWithCovarianceStamped") {
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg_ptr =  message.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "geometry_msgs/PoseArray") {
    geometry_msgs::PoseArray::ConstPtr msg_ptr =  message.instantiate<geometry_msgs::PoseArray>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "geometry_msgs/TwistWithCovarianceStamped") {
    geometry_msgs::TwistWithCovarianceStamped::ConstPtr msg_ptr =  message.instantiate<geometry_msgs::TwistWithCovarianceStamped>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
  if (topic_type == "geometry_msgs/TwistStamped") {
    geometry_msgs::TwistStamped::ConstPtr msg_ptr =  message.instantiate<geometry_msgs::TwistStamped>();
    SensorReadingID temp_sens_RD(topic_type,
                                 msg_ptr->header.stamp);
    addMeasurement(topic_name, temp_sens_RD);
    return;
  }
}