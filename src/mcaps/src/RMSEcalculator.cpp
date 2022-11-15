#include "RMSEcalculator.h"

using namespace std;

RMSEcalculator::RMSEcalculator(const std::string& out_folder)
{
  out_folder_=out_folder;
}

void RMSEcalculator::computeFromInterpolation()
{
//   TSS* temporalgraph = new TSS(tss_main_dir_);
//   temporalgraph->load(tss_yaml_name_);
//   temporalgraph->setBag(tss_bag_name_);
// 
//   std::vector<nav_msgs::Odometry> interpolated_gt_measures;
//   
//   interpolateTSS(temporalgraph, gt_topic_name_, interpolated_gt_measures);
//   
//   ifstream ifs(graph_name_);
//   ofstream ofs(tss_main_dir_+out_file_);
//   
//   ofstream poses_ofs(poses_file_);
//   
//   string tag;
//   
//   double RMSE=0;
//   double iter=0;
//   
//   ros::Time ref_time;
//   
//   while(!ifs.eof())
//   {
//      
//     ifs>>tag;
//     if(strcmp(tag.c_str(),"VERTEX_SE3:QUAT")==0)
//     {     
//       Eigen::Isometry3d pose;
//       int i;
//       ifs>>i;
//       if(i>temporalgraph->size()) continue;
//       
//       double tx,ty,tz, qw,qx,qy,qz;
//       ifs>>tx;ifs>>ty;ifs>>tz;
//       pose.translation()=Eigen::Vector3d(tx,ty,tz);
//       ifs>>qw;ifs>>qx;ifs>>qy;ifs>>qz;
//       Eigen::Quaterniond q; q.w()=qw; q.x()=qx; q.y()=qy; q.z()=qz;
//       pose.linear()=q.toRotationMatrix();
//       
//       ros::Time node_time;
//       temporalgraph->getNodeTime(i, node_time);
//       
//       if(iter==0)
//         ref_time=node_time;
//       
//       double diff_time=(node_time-ref_time).toSec();
//       
//       poses_ofs<<diff_time<<" "<<pose.translation().transpose()<<" "<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;
//       
//       /////////COmpute ERROR///////////////
//       cout<<"computing RMSE "<<i<<endl;
//       nav_msgs::Odometry leica_gt=interpolated_gt_measures[i];
//       if(leica_gt.header.stamp.toSec()<10) continue;
// 
//       Eigen::Isometry3d pose_gt;
//       CovarianceMatrixd cov_gt;
//       tss_utils::poseWithCovarianceMsg2EigenPoseCov(leica_gt.pose, pose_gt, cov_gt);
//       Eigen::Isometry3d err_T=pose_gt.inverse()*pose;
//       
//       double sq_norm=err_T.translation().squaredNorm();
//       if(sq_norm>1000) continue;
//       
//       RMSE+=sq_norm;
//       double err_x=fabs(err_T.translation()(0));
//       double err_y=fabs(err_T.translation()(1));
//       double err_z=fabs(err_T.translation()(2));
//       double err_norm=err_T.translation().norm();
//       Eigen::Vector4d err(err_x, err_y, err_z, err_norm);
//       iter++;
//       
//       ofs<<diff_time<<" "<<err.transpose()<<endl;
//       cout<<err.transpose()<<endl;
//       ////////////////////////////////////
//     }
//   }
//   
//   cout<<"RMSE opt: "<<sqrt(RMSE/iter)<<endl;
//   
//   ofs.close();
//   poses_ofs.close();
//   
//   temporalgraph->closeBag();
}

double RMSEcalculator::compute(const std::vector<nav_msgs::Odometry>& poses,
                             const std::vector<nav_msgs::Odometry>& gt_poses,
                             std::vector<nav_msgs::Odometry>& used_poses,
                             std::vector<nav_msgs::Odometry>& used_gt_poses
                            ) 
{
  
  ofstream ofs(out_folder_+"error.txt");
  
  string tag;
  
  double RMSE=0;
  double iter=0;
  
  ros::Time ref_time;
  
  used_gt_poses.clear();
  used_poses.clear();
  
  for(int i=0; i<poses.size(); i++)
  {
     
    nav_msgs::Odometry pose_msg=poses[i];
    nav_msgs::Odometry gt_msg=gt_poses[i];
    
    if(pose_msg.header.seq<=0 || gt_msg.header.seq<=0) continue;
    
    ros::Time node_time=pose_msg.header.stamp;
    
    if(iter==0)
      ref_time=pose_msg.header.stamp;
      
    double diff_time=(node_time-ref_time).toSec();
    
    Eigen::Isometry3d pose;
    CovarianceMatrixd cov;
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(pose_msg.pose, pose, cov);
    
    Eigen::Quaterniond q(pose.linear());
    
    /////////COmpute ERROR///////////////
//     cout<<"computing RMSE "<<i<<endl;
    Eigen::Isometry3d pose_gt;
    CovarianceMatrixd cov_gt;
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(gt_msg.pose, pose_gt, cov_gt);
    
    Eigen::Isometry3d err_T=pose_gt.inverse()*pose;
    
    double sq_norm=err_T.translation().squaredNorm();
    if(sq_norm>1000) continue;
    
    RMSE+=sq_norm;
    double err_x=fabs(err_T.translation()(0));
    double err_y=fabs(err_T.translation()(1));
    double err_z=fabs(err_T.translation()(2));
    double err_norm=err_T.translation().norm();
    Eigen::Vector4d err(err_x, err_y, err_z, err_norm);
    iter++;
    
    ofs<<fixed<<node_time.toSec()<<" "<<err.transpose()<<endl;
//     cout<<fixed<<node_time.toSec()<<" "<<err.transpose()<<endl;
    
    used_gt_poses.push_back(gt_msg);
    used_poses.push_back(pose_msg);
    ////////////////////////////////////
  }
  double rmse=sqrt(RMSE/iter);
  cout<<"RMSE opt: "<<rmse<<endl;
  
  ofs.close();
  
  return rmse;
}
