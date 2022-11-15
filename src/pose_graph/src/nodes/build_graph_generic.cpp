#include <pose_graph/temporalgraph.h>
#include <ros/ros.h>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "build_graph");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  // Read Parameters
  string bagfile_path, bagfile_name, output_graph_folder, outfile_name;
  string odom_topic, gps_topic, camera_topic, gt_topic;
  nh.getParam("bagfile_path", bagfile_path);
  nh.getParam("bagfile_name", bagfile_name);
  nh.getParam("output_graph_folder", output_graph_folder);
  nh.getParam("outfile_name", outfile_name);
  nh.getParam("odom_topic", odom_topic);
  nh.getParam("gps_topic", gps_topic);
  nh.getParam("camera_topic", camera_topic);
  nh.getParam("gt_topic", gt_topic);

  rosbag::Bag bag;

  bag.open(bagfile_path + bagfile_name, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(odom_topic);
  topics.push_back(camera_topic);
  topics.push_back(gps_topic);
  topics.push_back(gt_topic);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  TSS graph(output_graph_folder);

//   sensor_msgs::Image::ConstPtr image_ptr;

  /* Loop over the bag */
  int iter = 0;
  for (rosbag::View::iterator it = view.begin(); it != view.end(); it++) {
    rosbag::MessageInstance message(*it);
    /* Get pose message */
    if (message.getTopic() == odom_topic) {
      nav_msgs::Odometry::ConstPtr pose_ptr_ =
          message.instantiate<nav_msgs::Odometry>();
          
      /* Create a pose node */
      TemporalNode temp(pose_ptr_->pose.pose, pose_ptr_->header.stamp);
      //       CovarianceMatrixd cov;
      //       cov.setZero ( 6,6 );
      //       temp.setCovarianceMatrix ( cov );
      
      if(iter>0&&(graph.getNode(iter-1).getTime()==temp.getTime()))
        continue;
 
      /* Append node to the pose graph */
      graph.append(temp);
      cout << "node " << iter << " added" << endl;

      iter++;
    }
    if(iter==0) continue;
    graph.addNodeMeasurementFromMessage(message);

  }

  cout << "saving in " << output_graph_folder + outfile_name << "..." << endl;
  graph.save(outfile_name);
  cout << "DONE" << endl;
  return 0;
}