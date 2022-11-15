#pragma once

#include <pose_graph/temporalnode.h>
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
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

class TSS {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief TSS is an acronym for Temporal Sensory Sequence bla bla
   */
  TSS();
  TSS(const std::string& graph_dir);
  ~TSS();

  /**
   * @brief Append a new node to the end of the sequence
   *
   * @param[in] new_node The node to be appended
   *
   * [If you want, add here a longer description]
   */
  void append(TemporalNode& new_node);
  void save(const std::string& file_name);
  void load(const std::string& yaml_file_name);
  void setBag(std::string bag_in_name);
  void setBag(rosbag::Bag& bag_in);
  void closeBag();
  void extractBag(std::string bag_out_name);
  void extractBag(rosbag::Bag& output_bag);

  TemporalNode createNodeFromMessage(const rosbag::MessageInstance& msg);

  inline TemporalNode& getNode(const int& i) { return ts_queue_[i]; };
  inline int size() { return ts_queue_.size(); };

  template <typename T>
  boost::shared_ptr<const T> getNodeData(const int& idx,
                                         const std::string& topic_name) {
    std::vector<std::string> topics(1, topic_name);
    int node_id = ts_queue_[idx].id_;
    SensorReadingID sr = ts_queue_[idx].sensor_readings_[topic_name];
    if (sr.getTopicType() == "") return 0;
    ros::Time msg_time = sr.getTime();

    rosbag::View view(bag_, rosbag::TopicQuery(topics),
                      msg_time - ros::Duration(.001));
//     std::cout<<"degub: "<<idx<<" "<<msg_time.toNSec()<<" "<<ts_queue_[idx].getTime().toNSec()<<std::endl;
    for (rosbag::View::iterator it = view.begin(); it != view.end(); it++) 
    {
      rosbag::MessageInstance message(*it);
      boost::shared_ptr<const T> m = message.instantiate<T>();
      
      if (hash_table_[topic_name][m->header.stamp.toNSec()] == (node_id))
      { 
        return m;
      }
    }
    return 0;
  };

  inline void setNodePose(const int& idx, const Eigen::Isometry3d& pose) {
    ts_queue_[idx].pose_ = pose;
  };

  inline void getNodePose(const int& idx, Eigen::Isometry3d& pose) {
    pose = ts_queue_[idx].getPose();
  };
  
  inline void getNodeTime(const int& idx, ros::Time& t) {
    t = ts_queue_[idx].getTime();
  };
  
  inline void getNodeCovariance(const int& idx, CovarianceMatrixd& cov) {
    cov = ts_queue_[idx].getCovariance();
  };

  inline void addNodeMeasurement(const int& idx, const std::string& topic_name,
                                 const SensorReadingID& sensor_reading) {
    ts_queue_[idx].addMeasurement(topic_name, sensor_reading);
  };

  void addNodeMeasurementFromMessage(const int& idx, 
    const rosbag::MessageInstance& message);

  void addNodeMeasurementFromMessage( 
    const rosbag::MessageInstance& message);

  inline bool nodeHasMeasurements(const int& idx) {
    return ts_queue_[idx].hasMeasurements();
  };
  inline bool nodeHasMeasurement(const int& idx,
                                 const std::string& topic_name) {
    return ts_queue_[idx].hasMeasurement(topic_name);
  };
  inline bool nodeHasCovariance(const int& idx) {
    return ts_queue_[idx].has_covariance_;
  };

  void clear();
  
  bool writeOnBag(const rosbag::MessageInstance& message, rosbag::Bag& out_bag, bool flag_hash=true);
  
 private:
  Eigen::Vector3d calculateAngleError(
      const Eigen::Quaterniond&
          inp);  // Calculate rpy Angle Error w.r.t. the last Node attitude

  void computeHashTable();
  void computeDataTypeList();
  void computeTopicList();
  void computeHashTables();
  void updadteHashTables(const int& id,
                         std::unordered_map<std::string, SensorReadingID>& sr);

  template <typename MsgType>
  bool writeMsgOnBag(const rosbag::MessageInstance& message,
                     const std::string topic_name, rosbag::Bag& out_bag, bool flag_hash);
  bool writeClockMsgOnBag(const rosbag::MessageInstance& message,
                        const std::string topic_name, rosbag::Bag& out_bag);
  bool writeTFMsgOnBag(const rosbag::MessageInstance& message,
                        const std::string topic_name, rosbag::Bag& out_bag);
  template <typename K, typename V>
  static void hash2KeyVector(std::unordered_map<K, V>& hash,
                             std::vector<K>& vec);

  std::string pose_graph_dir_, bag_dir_;
  std::vector<TemporalNode> ts_queue_;
  rosbag::Bag bag_;
  std::string bag_filename_;

  // HASH TABLES
  std::unordered_map<std::string, std::unordered_map<uint64_t, int> >
      hash_table_;
  std::unordered_map<std::string, bool> data_type_list_;
  std::unordered_map<std::string, bool> topic_list_;
};