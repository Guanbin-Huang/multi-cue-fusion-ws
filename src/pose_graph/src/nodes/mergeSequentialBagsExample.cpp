#include <pose_graph/temporalgraph.h>
#include <pose_graph/tss_utils.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "sequential_bags_merging");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  // Read Parameters
  string out_bag_name;
  nh.getParam("out_bag_name", out_bag_name);
  
  std::string tmp;
  
  std::vector<std::string> bags_names;
  nh.getParam ( "bags_names", tmp);
  boost::erase_all(tmp, " ");
  boost::split ( bags_names, tmp, boost::is_any_of(","));
  
  std::vector<std::string> topics_names;
  nh.getParam ( "topics_names", tmp);
  boost::erase_all(tmp, " ");
  boost::split ( topics_names, tmp, boost::is_any_of(","));
  
  
  tss_utils::mergeSequentialBags(bags_names, topics_names, out_bag_name);

  return 0;
}