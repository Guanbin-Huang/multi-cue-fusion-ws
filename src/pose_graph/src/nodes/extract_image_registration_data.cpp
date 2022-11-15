#include <pose_graph/temporalgraph.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;

int main ( int argc, char** argv )
{

  ros::init ( argc, argv, "pose_graph_node" );
//   ros::NodeHandle nh;
//   ros::NodeHandle nh_private ( "~" );

  TSS graph;
  
  graph.load(argv[1]);
  graph.setBag(argv[2]);
  
  string dir=argv[4];
  
  boost::filesystem::path _dir(dir);
  boost::filesystem::create_directory(_dir);
  
  ofstream out_txt;
  out_txt.open( (dir+"out.txt").c_str() );
  
  for(int i=0; i<graph.size(); i++)
  {
    sensor_msgs::ImageConstPtr im_msg=graph.getNodeData<sensor_msgs::Image>(i, argv[3]);
    if(!im_msg) continue;
    
    Eigen::Isometry3d pose;
    graph.getNodePose(i, pose);
    
    cv::Mat curr_img = cv_bridge::toCvShare( im_msg, im_msg->encoding)->image;
    string im_name;
    stringstream ss; ss<<"image_"<<i<<".png";
    im_name=ss.str();
    string img_path=dir+im_name;
    cout<<img_path<<endl;
    cv::imwrite(img_path, curr_img);
    Eigen::Matrix3d rot=pose.linear().inverse();
    Eigen::Vector3d rpy=rot.eulerAngles(0,1,2);
    out_txt << im_name << " " << pose.translation().transpose()<<" "<<rpy.transpose()<< endl;
  }
  
  graph.closeBag();

  return 0;
}
