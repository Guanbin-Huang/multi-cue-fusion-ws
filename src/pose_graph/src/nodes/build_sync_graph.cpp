#include <pose_graph/temporalgraph.h>
#include <ros/ros.h>
#include <string>
#include <vector>

using namespace std;

template <typename MsgType>
int computeClosestOdometry(vector<nav_msgs::Odometry>& odom, MsgType& msg)
{
  double t_diff_min=100000000;
  int idx=-1;
  for(int i=0; i<odom.size(); i++)
  {
    double t_diff=fabs(odom[i].header.stamp.toSec()-msg.header.stamp.toSec());
    if(t_diff<t_diff_min)
    {
      t_diff_min=t_diff; 
      idx=i;
    }
  }
  return idx;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "build_graph");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  // Read Parameters
  string bagfile_path, bagfile_name, output_graph_folder, outfile_name;
  string odom_topic, gps_topic, gps2_topic, camera_topic, camera2_topic, camera3_topic, jai_topic, gt_topic, imu_topic, velodyne_topic;
  nh.getParam("bagfile_path", bagfile_path);
  nh.getParam("bagfile_name", bagfile_name);
  nh.getParam("output_graph_folder", output_graph_folder);
  nh.getParam("outfile_name", outfile_name);
  nh.getParam("odom_topic", odom_topic);
  nh.getParam("gps_topic", gps_topic);
  nh.getParam("gps2_topic", gps2_topic);
  nh.getParam("camera_topic", camera_topic);
  nh.getParam("camera2_topic", camera2_topic);
  nh.getParam("camera3_topic", camera3_topic);
  nh.getParam("jai_topic", jai_topic);
  nh.getParam("gt_topic", gt_topic);
  nh.getParam("imu_topic", imu_topic);
  nh.getParam("velodyne_topic", velodyne_topic);

  rosbag::Bag bag;

  bag.open(bagfile_path + bagfile_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(odom_topic);
  topics.push_back(camera_topic);
  topics.push_back(camera2_topic);
  topics.push_back(camera3_topic);
  topics.push_back(jai_topic);
  topics.push_back(gps_topic);
  topics.push_back(gps2_topic);
  topics.push_back(gt_topic);
  topics.push_back(imu_topic);
  topics.push_back(velodyne_topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  TSS graph(output_graph_folder);

//   sensor_msgs::Image::ConstPtr image_ptr;
  
  vector<nav_msgs::Odometry> wo, gps, gps2, gt, vo, vo2, vo3, imu;
  vector<sensor_msgs::Image> jai;
  vector<sensor_msgs::PointCloud2> velod;

  /* Loop over the bag */
  int iter = 0;
  for (rosbag::View::iterator it = view.begin(); it != view.end(); it++) {
    rosbag::MessageInstance message(*it);
    /* Get pose message */
    if (message.getTopic() == odom_topic) 
    {
      nav_msgs::Odometry::ConstPtr pose_ptr =
          message.instantiate<nav_msgs::Odometry>();
      wo.push_back(*pose_ptr);
    }
    if (message.getTopic() == gps_topic) 
    {
      nav_msgs::Odometry::ConstPtr pose_ptr =
          message.instantiate<nav_msgs::Odometry>();
      gps.push_back(*pose_ptr);
    }
    if (message.getTopic() == gps2_topic) 
    {
      nav_msgs::Odometry::ConstPtr pose_ptr =
          message.instantiate<nav_msgs::Odometry>();
      gps2.push_back(*pose_ptr);
    }
    if (message.getTopic() == camera_topic) 
    {
      nav_msgs::Odometry::ConstPtr pose_ptr =
          message.instantiate<nav_msgs::Odometry>();
      vo.push_back(*pose_ptr);
    }
    if (message.getTopic() == camera2_topic) 
    {
      nav_msgs::Odometry::ConstPtr pose_ptr =
          message.instantiate<nav_msgs::Odometry>();
      vo2.push_back(*pose_ptr);
    }
    if (message.getTopic() == camera3_topic) 
    {
      nav_msgs::Odometry::ConstPtr pose_ptr =
          message.instantiate<nav_msgs::Odometry>();
      vo3.push_back(*pose_ptr);
    }
    if (message.getTopic() == jai_topic) 
    {
      sensor_msgs::Image::ConstPtr img_ptr =
          message.instantiate<sensor_msgs::Image>();
      jai.push_back(*img_ptr);
    }
    if (message.getTopic() == gt_topic) 
    {
      nav_msgs::Odometry::ConstPtr pose_ptr =
          message.instantiate<nav_msgs::Odometry>();
      gt.push_back(*pose_ptr);
    }
    if (message.getTopic() == imu_topic) 
    {
      nav_msgs::Odometry::ConstPtr pose_ptr =
          message.instantiate<nav_msgs::Odometry>();
      imu.push_back(*pose_ptr);
    }
    if (message.getTopic() == velodyne_topic) 
    {
      sensor_msgs::PointCloud2::ConstPtr cloud_ptr =
          message.instantiate<sensor_msgs::PointCloud2>();
      velod.push_back(*cloud_ptr);
    }
  }
  
  
  vector<int> gps_map_idx, gps2_map_idx, vo_map_idx, vo2_map_idx, vo3_map_idx, jai_map_idx, gt_map_idx, imu_map_idx, velod_map_idx;
  for(int i=0; i<gps.size(); i++)
  {
    int idx=computeClosestOdometry(wo, gps[i]); 
    gps_map_idx.push_back(idx);
  }
  for(int i=0; i<gps2.size(); i++)
  {
    int idx=computeClosestOdometry(wo, gps2[i]); 
    gps2_map_idx.push_back(idx);
  }
  for(int i=0; i<vo.size(); i++)
  {
    int idx=computeClosestOdometry(wo, vo[i]); 
    vo_map_idx.push_back(idx);
  }
  for(int i=0; i<vo2.size(); i++)
  {
    int idx=computeClosestOdometry(wo, vo2[i]); 
    vo2_map_idx.push_back(idx);
  }
  for(int i=0; i<vo3.size(); i++)
  {
    int idx=computeClosestOdometry(wo, vo3[i]); 
    vo3_map_idx.push_back(idx);
  }
  for(int i=0; i<jai.size(); i++)
  {
    int idx=computeClosestOdometry(wo, jai[i]); 
    jai_map_idx.push_back(idx);
  }
  for(int i=0; i<gt.size(); i++)
  {
    int idx=computeClosestOdometry(wo, gt[i]); 
    gt_map_idx.push_back(idx);
  }
  for(int i=0; i<imu.size(); i++)
  {
    int idx=computeClosestOdometry(wo, imu[i]); 
    imu_map_idx.push_back(idx);
  }
  for(int i=0; i<velod.size(); i++)
  {
    int idx=computeClosestOdometry(wo, velod[i]); 
    velod_map_idx.push_back(idx);
  }
  
  
  /* create tss */
  for(int i=0; i<wo.size(); i++)
  {
    TemporalNode temp(wo[i].pose.pose, wo[i].header.stamp);
    graph.append(temp);
    cout << "node " << i << " added" << endl;
    SensorReadingID sr("nav_msgs/Odometry", wo[i].header.stamp);
    graph.addNodeMeasurement(i, odom_topic, sr);
    cout << "node " << i << " wo measurement added" << endl;
  }
  for(int i=0; i<gps.size(); i++)
  {
    int idx=gps_map_idx[i];
    if(idx<0) continue;
    
    SensorReadingID sr("nav_msgs/Odometry", gps[i].header.stamp);
    graph.addNodeMeasurement(idx, gps_topic, sr);
    cout << "node " << idx << " gps measurement added" << endl;
  }
  for(int i=0; i<gps2.size(); i++)
  {
    int idx=gps2_map_idx[i];
    if(idx<0) continue;
    
    SensorReadingID sr("nav_msgs/Odometry", gps2[i].header.stamp);
    graph.addNodeMeasurement(idx, gps2_topic, sr);
    cout << "node " << idx << " gps2 measurement added" << endl;
  }
  for(int i=0; i<vo.size(); i++)
  {
    int idx=vo_map_idx[i];
    if(idx<0) continue;
    
    SensorReadingID sr("nav_msgs/Odometry", vo[i].header.stamp);
    graph.addNodeMeasurement(idx, camera_topic, sr);
    cout << "node " << idx << " vo measurement added" << endl;
  }
  for(int i=0; i<vo2.size(); i++)
  {
    int idx=vo2_map_idx[i];
    if(idx<0) continue;
    
    SensorReadingID sr("nav_msgs/Odometry", vo2[i].header.stamp);
    graph.addNodeMeasurement(idx, camera2_topic, sr);
    cout << "node " << idx << " vo2 measurement added" << endl;
  }
  for(int i=0; i<vo3.size(); i++)
  {
    int idx=vo3_map_idx[i];
    if(idx<0) continue;
    
    SensorReadingID sr("nav_msgs/Odometry", vo3[i].header.stamp);
    graph.addNodeMeasurement(idx, camera3_topic, sr);
    cout << "node " << idx << " vo3 measurement added" << endl;
  }
  for(int i=0; i<jai.size(); i++)
  {
    int idx=jai_map_idx[i];
    if(idx<0) continue;
    
    SensorReadingID sr("sensor_msgs/Image", jai[i].header.stamp);
    graph.addNodeMeasurement(idx, jai_topic, sr);
    cout << "node " << idx << " jai measurement added" << endl;
  }
  for(int i=0; i<gt.size(); i++)
  {
    int idx=gt_map_idx[i];
    if(idx<0) continue;
    
    SensorReadingID sr("nav_msgs/Odometry", gt[i].header.stamp);
    graph.addNodeMeasurement(idx, gt_topic, sr);
    cout << "node " << idx << " gt measurement added" << endl;
  }
  for(int i=0; i<imu.size(); i++)
  {
    int idx=imu_map_idx[i];
    if(idx<0) continue;
    
    SensorReadingID sr("nav_msgs/Odometry", imu[i].header.stamp);
    graph.addNodeMeasurement(idx, imu_topic, sr);
    cout << "node " << idx << " imu measurement added" << endl;
  }
  for(int i=0; i<velod.size(); i++)
  {
    int idx=velod_map_idx[i];
    if(idx<0) continue;
    
    SensorReadingID sr("sensor_msgs/PointCloud2", velod[i].header.stamp);
    graph.addNodeMeasurement(idx, velodyne_topic, sr);
    cout << "node " << idx << " velodyne measurement added" << endl;
  }

  cout << "saving in " << output_graph_folder + outfile_name << "..." << endl;
  graph.save(outfile_name);
  cout << "DONE" << endl;
  return 0;
}