#include <pose_graph/temporalgraph.h>

using namespace std;

TSS::TSS() {
  clear();

  pose_graph_dir_ = ros::package::getPath("pose_graph");
  pose_graph_dir_ += "/graphs/";
  bag_dir_ = ros::package::getPath("pose_graph");
  bag_dir_ += "/bags/";

  boost::filesystem::path _dir1(pose_graph_dir_);
  if (!is_directory(_dir1)) boost::filesystem::create_directory(_dir1);

  boost::filesystem::path _dir2(bag_dir_);
  if (!is_directory(_dir2)) boost::filesystem::create_directory(_dir2);
}

TSS::TSS(const string& graph_dir) {
  clear();

  pose_graph_dir_ = graph_dir;
//   pose_graph_dir_ += "/graphs/";
  bag_dir_ = graph_dir;
//   bag_dir_ += "/bags/";

  boost::filesystem::path _dir1(pose_graph_dir_);
  if (!is_directory(_dir1)) boost::filesystem::create_directory(_dir1);

  boost::filesystem::path _dir2(bag_dir_);
  if (!is_directory(_dir2)) boost::filesystem::create_directory(_dir2);
}

TSS::~TSS() {}

void TSS::clear() {
  ts_queue_.clear();
  hash_table_.clear();
  topic_list_.clear();
  data_type_list_.clear();
}

void TSS::append(TemporalNode& new_node) {
  ts_queue_.push_back(new_node);
  // Non si deve usare size()-1 per via del valore di ritorno di default della
  // unordered_map
  ts_queue_[ts_queue_.size() - 1].setId(ts_queue_.size());
  updadteHashTables(ts_queue_.size(), new_node.sensor_readings_);
}

void TSS::save(const string& file_name) {
  YAML::Node out_graph;

  for (int i = 0; i < ts_queue_.size(); i++) {
    cout << "saving node " << i << " out of " << ts_queue_.size() << endl;
    Eigen::Quaterniond temp_quat_(ts_queue_[i].getLinear());

    YAML::Node position;
    position.push_back(ts_queue_[i].getTranslation()(0));
    position.push_back(ts_queue_[i].getTranslation()(1));
    position.push_back(ts_queue_[i].getTranslation()(2));
    YAML::Node orientation;
    orientation.push_back(temp_quat_.w());
    orientation.push_back(temp_quat_.x());
    orientation.push_back(temp_quat_.y());
    orientation.push_back(temp_quat_.z());

    YAML::Node cov;
    if (ts_queue_[i].hasCovariance()) {
      CovarianceMatrixd m = ts_queue_[i].getCovariance();
      for (int k = 0; k < 6; k++)
        for (int j = 0; j < 6; j++) cov.push_back(m(k, j));
    }

    YAML::Node sm;
    int iter = 0;
    for (std::unordered_map<std::string, SensorReadingID>::iterator
             it = ts_queue_[i].sensor_readings_.begin();
         it != ts_queue_[i].sensor_readings_.end(); it++, iter++) {
      stringstream ss_;
      ss_ << iter << endl;
      string temp_sensor_meas_;
      ss_ >> temp_sensor_meas_;
      YAML::Node nsm;
      nsm.push_back(it->first);
      nsm.push_back(it->second.getTopicType());
      nsm.push_back(it->second.getTime().toNSec());
      sm.push_back(nsm);
    }

    YAML::Node temp;
    temp["id"] = ts_queue_[i].id_;
    temp["position"] = (position);
    temp["orientation"] = orientation;
    temp["pose_time"] = ts_queue_[i].getTime().toNSec();
    temp["has_covariance"] = ts_queue_[i].hasCovariance();
    if (ts_queue_[i].hasCovariance()) temp["covariance"] = cov;
    temp["sensor_measurements"] = sm;

    out_graph.push_back(temp);
  }

  ofstream o(pose_graph_dir_ + file_name);
  o << out_graph << std::endl;
}

void TSS::load(const string& yaml_file_name) {
  clear();
  YAML::Node in_yaml = YAML::LoadFile(pose_graph_dir_ + yaml_file_name);

  int node_size = in_yaml.size();
  ts_queue_.reserve(node_size);
  for (int i = 0; i < node_size; i++) {
    cout << "loading node " << i << " out of " << node_size << endl;
    stringstream sst;
    sst << i;
    string sstr = sst.str();

    YAML::Node n = in_yaml[i];

    YAML::Node aux = n["position"];
    Eigen::Vector3d position(aux[0].as<double>(), aux[1].as<double>(),
                             aux[2].as<double>());

    aux = n["orientation"];
    Eigen::Quaterniond orientation(aux[0].as<double>(), aux[1].as<double>(),
                                   aux[2].as<double>(), aux[3].as<double>());

    ros::Time pose_time;
    pose_time.fromNSec(n["pose_time"].as<uint64_t>());

    TemporalNode node(position, orientation, pose_time);
    node.id_ = n["id"].as<int>();

    bool has_cov = n["has_covariance"].as<bool>();
    if (has_cov) {
      int iter = 0;
      CovarianceMatrixd cov;
      aux = n["covariance"];
      for (unsigned int k = 0; k < 6; k++) {
        for (unsigned int j = 0; j < 6; j++, iter++)
          cov(k, j) = aux[iter].as<double>();
      }
      node.setCovarianceMatrix(cov);
    }

    aux = n["sensor_measurements"];
    for (unsigned int _i = 0; _i < aux.size(); _i++) {
      ros::Time sens_time;
      node.addMeasurement(
          aux[_i][0].as<string>(),
          SensorReadingID(aux[_i][1].as<string>(),
                          sens_time.fromNSec(aux[_i][2].as<uint64_t>())));
    }

    ts_queue_.push_back(node);
  }

  computeHashTables();
}

void TSS::computeHashTable() {
  hash_table_.clear();
  for (int i = 0; i < ts_queue_.size(); i++)
    for (std::unordered_map<std::string, SensorReadingID>::iterator it =
             ts_queue_[i].sensor_readings_.begin();
         it != ts_queue_[i].sensor_readings_.end(); it++)
      hash_table_[it->first][it->second.getTime().toNSec()] = ts_queue_[i].id_;
}

void TSS::computeDataTypeList() {
  data_type_list_.clear();
  for (int i = 0; i < ts_queue_.size(); i++)
    for (std::unordered_map<std::string, SensorReadingID>::iterator it =
             ts_queue_[i].sensor_readings_.begin();
         it != ts_queue_[i].sensor_readings_.end(); it++)
      data_type_list_[it->second.getTopicType()] = true;
}

void TSS::computeTopicList() {
  topic_list_.clear();
  for (int i = 0; i < ts_queue_.size(); i++)
    for (std::unordered_map<std::string, SensorReadingID>::iterator it =
             ts_queue_[i].sensor_readings_.begin();
         it != ts_queue_[i].sensor_readings_.end(); it++)
      topic_list_[it->first] = true;
}

void TSS::computeHashTables() {
  computeHashTable();
  computeDataTypeList();
  computeTopicList();
}

void TSS::updadteHashTables(
    const int& id, std::unordered_map<std::string, SensorReadingID>& sr) {
  for (std::unordered_map<std::string, SensorReadingID>::iterator it =
           sr.begin();
       it != sr.end(); it++) {
    hash_table_[it->first][it->second.getTime().toNSec()] = id;
    data_type_list_[it->second.getTopicType()] = true;
    topic_list_[it->first] = true;
  }
}

void TSS::setBag(string bag_in_name) {
  closeBag();
  computeHashTables();
  bag_.open(bag_dir_ + bag_in_name, rosbag::bagmode::Read);
}

void TSS::setBag(rosbag::Bag& bag_in) {
  computeHashTables();
  std::string b_name=bag_in.getFileName();
  bag_in.close();
  closeBag();
  bag_.open(b_name, rosbag::bagmode::Read);
}

void TSS::closeBag() { bag_.close(); }

void TSS::addNodeMeasurementFromMessage(const int& idx, 
    const rosbag::MessageInstance& message) {
  ts_queue_[idx].addMeasurement(message);
}

void TSS::addNodeMeasurementFromMessage( 
    const rosbag::MessageInstance& message) {
  ts_queue_[ts_queue_.size()-1].addMeasurement(message);
}

template <typename K, typename V>
void TSS::hash2KeyVector(std::unordered_map<K, V>& hash, std::vector<K>& vec) {
  for (typename std::unordered_map<string, V>::iterator it = hash.begin();
       it != hash.end(); it++)
    vec.push_back(it->first);
}

template <typename MsgType>
bool TSS::writeMsgOnBag(const rosbag::MessageInstance& message,
                        const string topic_name, rosbag::Bag& out_bag, bool flag_hash) {
  boost::shared_ptr<const MsgType> m = message.instantiate<MsgType>();
  uint64_t time_stamp = m->header.stamp.toNSec();
  if(!flag_hash)
  {
    out_bag.write(topic_name, m->header.stamp, m);
    return true;
  }
  if (hash_table_[topic_name][time_stamp] > 0) {
    out_bag.write(topic_name, m->header.stamp, m);
    return true;
  }
  return false;
}

bool TSS::writeClockMsgOnBag(const rosbag::MessageInstance& message,
                        const string topic_name, rosbag::Bag& out_bag) {
  boost::shared_ptr<const rosgraph_msgs::Clock> m = message.instantiate<rosgraph_msgs::Clock>();
  uint64_t time_stamp = m->clock.toNSec();
  out_bag.write(topic_name, m->clock, m);
  return true;
}

bool TSS::writeTFMsgOnBag(const rosbag::MessageInstance& message,
                        const string topic_name, rosbag::Bag& out_bag) {
  boost::shared_ptr<const tf2_msgs::TFMessage> m = message.instantiate<tf2_msgs::TFMessage>();
  uint64_t time_stamp = m->transforms[0].header.stamp.toNSec();
  out_bag.write(topic_name, m->transforms[0].header.stamp, m);
  return true;
}

bool TSS::writeOnBag(const rosbag::MessageInstance& message,
                     rosbag::Bag& out_bag, bool flag_hash) {
  string topic_name = message.getTopic();
  string topic_type = message.getDataType();

  if (flag_hash&&(!data_type_list_[topic_type] || !topic_list_[topic_name])) return false;

  
  if (topic_type == "rosgraph_msgs/Clock") {
    if (writeClockMsgOnBag(message, topic_name, out_bag))
      return true;
  }
  if (topic_type == "tf2_msgs/TFMessage") {
    if (writeTFMsgOnBag(message, topic_name, out_bag))
      return true;
  }
  
  
  if (topic_type == "nav_msgs/Odometry") {
    if (writeMsgOnBag<nav_msgs::Odometry>(message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "sensor_msgs/Image") {
    if (writeMsgOnBag<sensor_msgs::Image>(message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "sensor_msgs/NavSatFix") {
    if (writeMsgOnBag<sensor_msgs::NavSatFix>(message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "sensor_msgs/Imu") {
    if (writeMsgOnBag<sensor_msgs::Imu>(message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "sensor_msgs/CameraInfo") {
    if (writeMsgOnBag<sensor_msgs::CameraInfo>(message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "sensor_msgs/LaserScan") {
    if (writeMsgOnBag<sensor_msgs::LaserScan>(message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "sensor_msgs/PointCloud") {
    if (writeMsgOnBag<sensor_msgs::PointCloud>(message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "sensor_msgs/PointCloud2") {
    if (writeMsgOnBag<sensor_msgs::PointCloud2>(message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "geometry_msgs/PoseStamped") {
    if (writeMsgOnBag<geometry_msgs::PoseStamped>(message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "geometry_msgs/PointStamped") {
    if (writeMsgOnBag<geometry_msgs::PointStamped>(message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "geometry_msgs/PoseWithCovarianceStamped") {
    if (writeMsgOnBag<geometry_msgs::PoseWithCovarianceStamped>(
            message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "geometry_msgs/PoseArray") {
    if (writeMsgOnBag<geometry_msgs::PoseArray>(message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "geometry_msgs/TwistWithCovarianceStamped") {
    if (writeMsgOnBag<geometry_msgs::TwistWithCovarianceStamped>(
            message, topic_name, out_bag, flag_hash))
      return true;
  }
  if (topic_type == "geometry_msgs/TwistStamped") {
    if (writeMsgOnBag<geometry_msgs::TwistStamped>(message, topic_name,
                                                   out_bag, flag_hash))
      return true;
  }

  return false;
}

void TSS::extractBag(string bag_out_name) {
  computeHashTables();
  rosbag::Bag output_bag;
  output_bag.open(bag_dir_ + bag_out_name, rosbag::bagmode::Write);

  std::vector<string> topics;
  hash2KeyVector<string, bool>(topic_list_, topics);
  rosbag::View view(bag_, rosbag::TopicQuery(topics));

  int iter = 0;
  for (rosbag::View::iterator it = view.begin(); it != view.end(); it++) {
    rosbag::MessageInstance message(*it);
    if (writeOnBag(message, output_bag)) {
      iter++;
      cout << "storing ros msg " << iter << endl;
    }
  }
  cout << "ros msg stored: " << iter << endl;
  cout << "closing bag..." << endl;

  output_bag.close();
}

void TSS::extractBag(rosbag::Bag& output_bag) {
  computeHashTables();
//   output_bag.open(bag_dir_ + bag_out_name, rosbag::bagmode::Write);

  std::vector<string> topics;
  hash2KeyVector<string, bool>(topic_list_, topics);
  rosbag::View view(bag_, rosbag::TopicQuery(topics));

  int iter = 0;
  for (rosbag::View::iterator it = view.begin(); it != view.end(); it++) {
    rosbag::MessageInstance message(*it);
    if (writeOnBag(message, output_bag)) {
      iter++;
      cout << "storing ros msg " << iter << endl;
    }
  }
  cout << "ros msg stored: " << iter << endl;
}

Eigen::Vector3d TSS::calculateAngleError(const Eigen::Quaterniond& inp_) {
  Eigen::Matrix3d angle_error_matrix =
      0.5 *
      (ts_queue_.back().getLinear().transpose() * inp_.toRotationMatrix() -
       inp_.toRotationMatrix().transpose() * ts_queue_.back().getLinear());
  return Eigen::Vector3d(angle_error_matrix(2, 1), angle_error_matrix(0, 2),
                         angle_error_matrix(1, 0));
}