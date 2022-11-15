#pragma once

#include "temporalgraph.h"
#include <gps_common/conversions.h>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Config.h>
#include <theora_image_transport/Packet.h>

#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace std;

namespace tss_utils
{
  static void LatLong2Eigen(const sensor_msgs::NavSatFixConstPtr& fix, Eigen::Isometry3d& pose, CovarianceMatrixd& cov)
  {
    double northing, easting;
    std::string zone;
    double rot_cov=999999.0;

    gps_common::LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

    pose.translation()(0) = easting;
    pose.translation()(1) = northing;
    pose.translation()(2) = fix->altitude;

    Eigen::Quaterniond q(1,0,0,0);
    pose.linear()=Eigen::Matrix3d(q);
    
    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};
    
    cov<< fix->position_covariance[0],
          fix->position_covariance[1],
          fix->position_covariance[2],
          0, 0, 0,
          fix->position_covariance[3],
          fix->position_covariance[4],
          fix->position_covariance[5],
          0, 0, 0,
          fix->position_covariance[6],
          fix->position_covariance[7],
          fix->position_covariance[8],
          0, 0, 0,
          0, 0, 0, rot_cov, 0, 0,
          0, 0, 0, 0, rot_cov, 0,
          0, 0, 0, 0, 0, rot_cov;
  };
  
  static void computeProximityGraph(TSS& g_in, const Eigen::Vector3d& position, const double& distance, TSS& g_out)
  {
    g_out.clear();
    for (int i=0; i<g_in.size(); i++)
    {
      Eigen::Isometry3d pose;
      g_in.getNodePose(i, pose);
      if(pose.translation().norm()<distance)
      {
        TemporalNode node(g_in.getNode(i));
        g_out.append(node);
      }
    }
  };
  
  
  static void mergeSequentialBags(const vector<string>& bags_names, const vector<string>& topics_names, string out_bag_name)
  {
    rosbag::Bag output_bag, in_bag;
    output_bag.open( out_bag_name, rosbag::bagmode::Write );
    
    for(int i=0; i<bags_names.size(); i++)
    {
      
      TSS graph;
      Eigen::Isometry3d dummy;
//       TemporalNode dummy_node(dummy, ros::Time::now());
//       graph.append(dummy_node);
      
      in_bag.open ( bags_names[i], rosbag::bagmode::Read );
      rosbag::View view ( in_bag, rosbag::TopicQuery ( topics_names ) );
      
      for ( rosbag::View::iterator it = view.begin(); it!=view.end(); it++ )
      {
        TemporalNode dummy_node(dummy, ros::Time::now());
        graph.append(dummy_node);
        rosbag::MessageInstance message ( *it );
        graph.addNodeMeasurementFromMessage(message);
      }
      cout<<"ready to store messages of bag "<<bags_names[i]<<endl;
//       in_bag.close();
//       in_bag.open ( bags_names[i], rosbag::bagmode::Read );
      graph.setBag(in_bag);
      graph.extractBag(output_bag);
      in_bag.close();
      graph.closeBag();
    }

    output_bag.close(); 
  };
  
  
  static void mergeBags(const string& bag_1, const string& bag_2, const vector<string>& topics_names, string out_bag_name)
  {
    rosbag::Bag output_bag, in_bag1, in_bag2;
    output_bag.open( out_bag_name, rosbag::bagmode::Write );
    
    ros::Time curr_time, msg1_time, msg2_time;

    in_bag1.open ( bag_1, rosbag::bagmode::Read );
    rosbag::View view1 ( in_bag1 );
    in_bag2.open ( bag_2, rosbag::bagmode::Read );
    rosbag::View view2 ( in_bag2, rosbag::TopicQuery ( topics_names ) );
    
    rosbag::View::iterator it1 = view1.begin();
    rosbag::View::iterator it2 = view2.begin();
    
    TSS graph;
    Eigen::Isometry3d dummy;
    
    while(it1!=view1.end()&&it2!=view2.end())
    {
      TemporalNode dummy_node1(dummy, ros::Time::now());
      TemporalNode dummy_node2(dummy, ros::Time::now());

      rosbag::MessageInstance message1 ( *it1 );
      string topic1=message1.getTopic();
      rosbag::MessageInstance message2 ( *it2 );
      string topic2=message2.getTopic();
      dummy_node1.addMeasurement(message1);
      dummy_node2.addMeasurement(message2);
      
      msg1_time=dummy_node1.getMeasurement(topic1).getTime();
      msg2_time=dummy_node2.getMeasurement(topic2).getTime();
      
      if(msg1_time<msg2_time)
      {
        graph.writeOnBag(message1, output_bag, false);
        if(it1!=view1.end())
          it1++;
      }
      else
      {
        graph.writeOnBag(message2, output_bag, false);
        if(it2!=view2.end())
          it2++;
      }   
    }
    
    while(it1!=view1.end())
    {
      TemporalNode dummy_node1(dummy, ros::Time::now());
      rosbag::MessageInstance message1 ( *it1 );
      string topic1=message1.getTopic();
        graph.writeOnBag(message1, output_bag, false);
        if(it1!=view1.end())
          it1++;
    }
    
    while(it2!=view2.end())
    {
      TemporalNode dummy_node2(dummy, ros::Time::now());
      rosbag::MessageInstance message2 ( *it2 );
      string topic2=message2.getTopic();
        graph.writeOnBag(message2, output_bag, false);
        if(it2!=view2.end())
          it2++;
    }
    
    in_bag1.close();
    in_bag2.close();
    output_bag.close(); 
  };
  
  static void poseMsg2EigenPose(const geometry_msgs::Pose& pose, Eigen::Isometry3d& eigen_pose)
  {
    Eigen::Vector3d position(pose.position.x,
                            pose.position.y,
                            pose.position.z);
    Eigen::Quaterniond orientation(pose.orientation.w,
                                  pose.orientation.x,
                                  pose.orientation.y,
                                  pose.orientation.z);
    eigen_pose.translation()=position;
    eigen_pose.linear()=orientation.toRotationMatrix();
  }
  
  static void poseWithCovarianceMsg2EigenPoseCov(const geometry_msgs::PoseWithCovariance& pose, Eigen::Isometry3d& eigen_pose, CovarianceMatrixd& cov)
  {
    Eigen::Vector3d position(pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z);
    Eigen::Quaterniond orientation(pose.pose.orientation.w,
                                  pose.pose.orientation.x,
                                  pose.pose.orientation.y,
                                  pose.pose.orientation.z);
    eigen_pose.translation()=position;
    eigen_pose.linear()=orientation.toRotationMatrix();
    
    double rot_cov_0, rot_cov_1, rot_cov_2;
    (pose.covariance[21]==0)? rot_cov_0=999999: rot_cov_0=pose.covariance[21];
    (pose.covariance[28]==0)? rot_cov_1=999999: rot_cov_1=pose.covariance[28];
    (pose.covariance[35]==0)? rot_cov_2=999999: rot_cov_2=pose.covariance[35];
    
    cov<< pose.covariance[0], pose.covariance[1], pose.covariance[2], pose.covariance[3], pose.covariance[4], pose.covariance[5],
          pose.covariance[6], pose.covariance[7], pose.covariance[8], pose.covariance[9], pose.covariance[10], pose.covariance[11],
          pose.covariance[12], pose.covariance[13], pose.covariance[14], pose.covariance[15], pose.covariance[16], pose.covariance[17],
          pose.covariance[18], pose.covariance[19], pose.covariance[20], rot_cov_0, pose.covariance[22], pose.covariance[23],
          pose.covariance[24], pose.covariance[25], pose.covariance[26], pose.covariance[27], rot_cov_1, pose.covariance[29],
          pose.covariance[30], pose.covariance[31], pose.covariance[32], pose.covariance[33], pose.covariance[34], rot_cov_2;

  }
  
  static void odometry2EigenPoseList(const std::vector<nav_msgs::Odometry>& odom, std::vector<Eigen::Isometry3d>& pose_list)
  {
    pose_list.clear();
    for(int i=0; i<odom.size(); i++)
    {
      if(odom[i].header.seq<=0)
        continue;
      CovarianceMatrixd cov;
      Eigen::Isometry3d pose;
      poseWithCovarianceMsg2EigenPoseCov(odom[i].pose, pose, cov);
      pose_list.push_back(pose);
    }
  }
  
  static bool imageMsg2CvMat(const sensor_msgs::Image& msg, cv::Mat& img)
  {
    if(msg.header.seq<=0)
      return false;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    img=cv_ptr->image.clone();
    return true;
  }
  
  static void cloudMsg2PCL(const sensor_msgs::PointCloud2& msg, pcl::PointCloud<pcl::PointXYZ>& cloud)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,cloud);
  }
  
  static void pclPointcloud2PointVector(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<Eigen::Vector3f>& points)
  {
    points.clear();
    points.resize(cloud.points.size());
    for(int i=0; i<points.size(); i++)
    {
      pcl::PointXYZ p=cloud.points[i];
      points[i]=Eigen::Vector3f((float)p.x, (float)p.y, (float)p.z);
    }
  }
  
  
}