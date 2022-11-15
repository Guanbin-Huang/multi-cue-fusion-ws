#include <pose_graph/temporalgraph.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "build_graph");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  // Read Parameters
  string bagfile_path, bagfile_name, output_graph_folder, outfile_name;
  string odom_topic, gps_topic, camera_topic;
  nh.getParam("bagfile_path", bagfile_path);
  nh.getParam("bagfile_name", bagfile_name);
  nh.getParam("output_graph_folder", output_graph_folder);
  nh.getParam("outfile_name", outfile_name);
  nh.getParam("odom_topic", odom_topic);
  nh.getParam("gps_topic", gps_topic);
  nh.getParam("camera_topic", camera_topic);

  rosbag::Bag bag;

  bag.open(bagfile_path + bagfile_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(odom_topic);
  topics.push_back(camera_topic);
  topics.push_back(gps_topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  TSS graph(output_graph_folder);

  sensor_msgs::Image::ConstPtr image_ptr;

  /* Loop over the bag */
  int iter = 0;
  for (rosbag::View::iterator it = view.begin(); it != view.end(); it++) {
    rosbag::MessageInstance message(*it);
    /* Get pose message */
    if (message.getTopic() == odom_topic) {
      nav_msgs::Odometry::ConstPtr pose_ptr_ =
          message.instantiate<nav_msgs::Odometry>();

      Eigen::Vector3d position(pose_ptr_->pose.pose.position.x,
                               pose_ptr_->pose.pose.position.y,
                               pose_ptr_->pose.pose.position.z);
      Eigen::Quaterniond orientation(pose_ptr_->pose.pose.orientation.w,
                                     pose_ptr_->pose.pose.orientation.x,
                                     pose_ptr_->pose.pose.orientation.y,
                                     pose_ptr_->pose.pose.orientation.z);

      /* Create a pose node */
      TemporalNode temp(position, orientation, pose_ptr_->header.stamp);
      //       CovarianceMatrixd cov;
      //       cov.setZero ( 6,6 );
      //       temp.setCovarianceMatrix ( cov );

      /* Append node to the pose graph */
      graph.append(temp);
      cout << "node " << iter << " added" << endl;

      iter++;
    }

    /* Get image topic */
    else if (message.getTopic() == camera_topic) {
      if (iter == 0) continue;

      sensor_msgs::Image::ConstPtr image_ptr =
          message.instantiate<sensor_msgs::Image>();

      /* Associate a sensor reading to the current pose node */
      SensorReadingID temp_sens_RD("sensor_msgs/Image",
                                   image_ptr->header.stamp);
      graph.addNodeMeasurement(iter - 1, camera_topic, temp_sens_RD);
    }

    /* Get gps topic */
    else if (message.getTopic() == gps_topic) {
      if (iter == 0) continue;

      sensor_msgs::NavSatFix::ConstPtr gps_ptr =
          message.instantiate<sensor_msgs::NavSatFix>();

      /* Associate a sensor reading to the current pose node */
      SensorReadingID temp_sens_RD("sensor_msgs/NavSatFix",
                                   gps_ptr->header.stamp);
      graph.addNodeMeasurement(iter - 1, gps_topic, temp_sens_RD);
    }
  }

  cout << "saving in " << output_graph_folder + outfile_name << "..." << endl;
  graph.save(outfile_name);
  cout << "DONE" << endl;
  return 0;
}