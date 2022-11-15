#include <pose_graph/temporalgraph.h>
#include <string>

using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_graph_node");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  // Read Parameters
  string bagfile_path, bagfile_name, graph_folder, graph_file_name,
      reduced_bagfile_name;
  nh.getParam("bagfile_path", bagfile_path);
  nh.getParam("bagfile_name", bagfile_name);
  nh.getParam("reduced_bagfile_name", reduced_bagfile_name);
  nh.getParam("graph_folder", graph_folder);
  nh.getParam("graph_file_name", graph_file_name);
  

  TSS in_graph(graph_folder);
  in_graph.load(graph_file_name);
  in_graph.setBag(bagfile_name);
  in_graph.extractBag(reduced_bagfile_name);
  in_graph.closeBag();

  return 0;
}