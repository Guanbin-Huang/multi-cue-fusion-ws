#ifndef INTERPOLATION_UTILS_H
#define INTERPOLATION_UTILS_H

#include "paramsUtils.h"
#include "pose_graph/temporalgraph.h"
#include "pose_graph/tss_utils.h"

struct interpolatedMeasures
{
  vector<nav_msgs::Odometry> wo;
  unordered_map<int, nav_msgs::Odometry> gps_map;
  unordered_map<int, nav_msgs::Odometry> vo_map;
  unordered_map<int, nav_msgs::Odometry> gt_map;
  
  void clear()
  {
    wo.clear();
    gps_map.clear();
    vo_map.clear();
    gt_map.clear();
  }
};

static bool getPrevNextMeasures(  vector< nav_msgs::Odometry >& msg_list, 
                           double& ref_time, 
                          nav_msgs::Odometry& prev_msg, 
                          nav_msgs::Odometry& next_msg)
{
  bool flag=false;
  double prev_time=-1;
  double next_time=-1;
  for(int i=0; i<msg_list.size(); i++)
  {
    double curr_time=msg_list[i].header.stamp.toSec();
    if(ref_time>curr_time)
    {
      prev_time=curr_time;
      prev_msg=msg_list[i];
    }
    else
    {
      if(prev_time<0)
        return false;
      next_time=curr_time;
      next_msg=msg_list[i];
      return true;
    }
  }
  return false;
}

static bool getPrevNextMeasures(  vector< nav_msgs::Odometry >& msg_list, 
                           double& ref_time,
                           const int idx,
                          nav_msgs::Odometry& prev_msg, 
                          nav_msgs::Odometry& next_msg)
{
  bool flag=false;
  double prev_time=-1;
  double next_time=-1;
  for(int i=idx-1; i<idx+2; i++)
  {
    if(i<0||i>=msg_list.size())
      continue;
    double curr_time=msg_list[i].header.stamp.toSec();
    if(ref_time>curr_time)
    {
      prev_time=curr_time;
      prev_msg=msg_list[i];
    }
    else
    {
      if(prev_time<0)
        return false;
      next_time=curr_time;
      next_msg=msg_list[i];
      return true;
    }
  }
  return false;
}

static void interpolateMeasures(  nav_msgs::Odometry& prev_msg,
                           nav_msgs::Odometry& next_msg, 
                           double& ref_time, 
                          nav_msgs::Odometry& interpolated_msg)
{
  double prev_t=prev_msg.header.stamp.toSec();
  double next_t=next_msg.header.stamp.toSec();
  double diff_t=next_t-prev_t;
  double diff_ref_t=ref_time-prev_t;
  double w_next=diff_ref_t/diff_t;
  double w_prev=1-w_next;
  
//   std::cout<<"ciaoo: "<<diff_t<<std::endl;
  
  Eigen::Isometry3d prev_pose, next_pose;
  CovarianceMatrixd prev_cov, next_cov;
  tss_utils::poseWithCovarianceMsg2EigenPoseCov(prev_msg.pose, prev_pose,prev_cov);
  tss_utils::poseWithCovarianceMsg2EigenPoseCov(next_msg.pose, next_pose,next_cov);
  Eigen::Vector3d t_inter=w_prev*prev_pose.translation() + w_next*next_pose.translation();
  CovarianceMatrixd cov_inter=w_prev*prev_cov + w_next*next_cov;
  
  interpolated_msg=prev_msg;
  interpolated_msg.header.stamp.fromSec(ref_time);
  
  interpolated_msg.pose.pose.position.x=t_inter(0);
  interpolated_msg.pose.pose.position.y=t_inter(1);
  interpolated_msg.pose.pose.position.z=t_inter(2);
  
  interpolated_msg.pose.covariance[0]=cov_inter(0,0);
  interpolated_msg.pose.covariance[7]=cov_inter(1,1);
  interpolated_msg.pose.covariance[14]=cov_inter(2,2);
  interpolated_msg.pose.covariance[21]=cov_inter(3,3);
  interpolated_msg.pose.covariance[28]=cov_inter(4,4);
  interpolated_msg.pose.covariance[35]=cov_inter(5,5);
}


// static void interpolateTSS(TSS* temporalgraph,
//                     const string& topic_name,
//                     std::vector<nav_msgs::Odometry>& interpolated_measures
//                     )
// {
//   std::cout<<"TSS interpolation ..."<<std::endl;
//   vector<nav_msgs::Odometry> msg_list;
//   
//   for(int i=0; i<temporalgraph->size(); i++)
//   {
//     nav_msgs::OdometryConstPtr odom_msg;
//     odom_msg=temporalgraph->getNodeData<nav_msgs::Odometry>(i, topic_name);
//     if(odom_msg)
//       msg_list.push_back(*odom_msg);
//   }
//   
//   interpolated_measures.resize(temporalgraph->size());
//   
//   for(int i=0; i<temporalgraph->size(); i++)
//   {
//     ros::Time stamp;
//     temporalgraph->getNodeTime(i, stamp);
//     double ref_time=stamp.toSec();
//     
//     interpolated_measures[i].header.stamp.fromSec(0);
//     
//     nav_msgs::Odometry prev, next, interpolated;
//     if(!getPrevNextMeasures(msg_list, ref_time, prev, next))
//       continue;
//     
//     interpolateMeasures(prev, next, ref_time, interpolated);
//     
//     interpolated_measures[i]=interpolated;
//   } 
//   std::cout<<"TSS interpolation ... DONE!"<<std::endl;
// }

static void interpolateTSS(TSS* temporalgraph,
                    const string& wo_topic,
                    const string& gps_topic,
                    const string& vo_topic,
                    const string& gt_topic,
                    interpolatedMeasures& interpolated_measures
                    )
{
  std::cout<<"TSS interpolation ..."<<std::endl;
  interpolated_measures.clear();
  
  vector<nav_msgs::Odometry> msg_list;
  
  for(int i=0; i<temporalgraph->size(); i++)
  {
    nav_msgs::OdometryConstPtr odom_msg;
    odom_msg=temporalgraph->getNodeData<nav_msgs::Odometry>(i, wo_topic);
    if(odom_msg)
      msg_list.push_back(*odom_msg);
  }
  
  for(int i=0; i<temporalgraph->size(); i++)
  {
    nav_msgs::OdometryConstPtr odom_msg;
    odom_msg=temporalgraph->getNodeData<nav_msgs::Odometry>(i, wo_topic);
    if(odom_msg)
    {
      nav_msgs::OdometryConstPtr gps_msg, vo_msg, gt_msg;
      nav_msgs::Odometry prev, next, interpolated;
      
//       gps_msg=temporalgraph->getNodeData<nav_msgs::Odometry>(i, gps_topic);
//       if(gps_msg)
//       {
//         double ref_time=gps_msg->header.stamp.toSec();
//         if(!getPrevNextMeasures(msg_list, ref_time, i, prev, next))
//           continue;
//         interpolateMeasures(prev, next, ref_time, interpolated);
//         interpolated_measures.wo.push_back(interpolated);
//         interpolated_measures.gps_map[interpolated_measures.wo.size()-1]=*gps_msg;      
//       }
      vo_msg=temporalgraph->getNodeData<nav_msgs::Odometry>(i, vo_topic);
      if(vo_msg)
      {
        double ref_time=vo_msg->header.stamp.toSec();
        if(!getPrevNextMeasures(msg_list, ref_time, i, prev, next))
          continue;
        interpolateMeasures(prev, next, ref_time, interpolated);
        interpolated_measures.wo.push_back(interpolated);
        interpolated_measures.vo_map[interpolated_measures.wo.size()-1]=*vo_msg;        
      }
//       gt_msg=temporalgraph->getNodeData<nav_msgs::Odometry>(i, gt_topic);
//       if(gt_msg)
//       {
//         double ref_time=gt_msg->header.stamp.toSec();
//         if(!getPrevNextMeasures(msg_list, ref_time, i, prev, next))
//           continue;
//         interpolateMeasures(prev, next, ref_time, interpolated);
//         interpolated_measures.wo.push_back(interpolated);
//         interpolated_measures.gt_map[interpolated_measures.wo.size()-1]=*gt_msg;
//       }
    }
  }
  std::cout<<"TSS interpolation ... DONE!"<<std::endl;
}

#endif