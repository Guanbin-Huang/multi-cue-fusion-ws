#include "graph_creator.h"
// #include "nicp_utils.h"
#include "iostream"

#include "cov_func_point_to_point.h"
// #include "cov_func_point_to_plane.h"
#include "cbshot.h"

using namespace std;
using namespace g2o;

Eigen::Isometry3d GraphCreator::getPosesDiff(const Eigen::Isometry3d &pos1,
                                             const Eigen::Isometry3d &pos2)
{
  return pos2.inverse() * pos1;
}

void computeUtilezedOdometries(const std::vector<nav_msgs::Odometry> &wo,
                               const std::vector<nav_msgs::Odometry> &gps,
                               const std::vector<nav_msgs::Odometry> &vo,
                               //                                   const std::vector<nav_msgs::Odometry>& gt,
                               const std::vector<Eigen::Vector3d> &elevation_grid,
                               std::vector<nav_msgs::Odometry> &used_wo,
                               std::vector<nav_msgs::Odometry> &used_gps,
                               std::vector<nav_msgs::Odometry> &used_vo,
                               //                                  std::vector<nav_msgs::Odometry>& used_gt,
                               std::vector<Eigen::Vector3d> &used_elevation_grid)
{
}


void GraphCreator::writeGraph(const std::string name,
                              const std::vector<g2o::VertexSE3 *> &verteces,
                              const std::vector<g2o::VertexSE3 *> &vo_verteces,
                              const std::vector<g2o::VertexSE3 *> &svo_verteces,
                              const std::vector<g2o::EdgeSE3 *> &edges,
                              const std::vector<g2o::EdgeSE3 *> &vo_edges,
                              const std::vector<g2o::EdgeSE3 *> &svo_edges,
                              const std::vector<g2o::EdgeSE3Prior *> &prior_edges)
{
  std::ofstream file_graph(name.c_str());

  g2o::ParameterSE3Offset *offset_param_gps = new g2o::ParameterSE3Offset();
  //   offset_param_gps->setOffset();;
  offset_param_gps->setId(0);

  string parTag = Factory::instance()->tag(offset_param_gps);
  file_graph << parTag << " " << offset_param_gps->id() << " ";
  offset_param_gps->write(file_graph);
  file_graph << endl;

  for (size_t i = 0; i < verteces.size(); ++i)
  {
    VertexSE3 *v = verteces[i];
    string vertexTag = Factory::instance()->tag(v);
    file_graph << vertexTag << " " << v->id() << " ";
    v->write(file_graph);
    file_graph << endl;
  }
  //   for (size_t i = 0; i < vo_verteces.size(); ++i) {
  //     VertexSE3* v = vo_verteces[i];
  //     string vertexTag = Factory::instance()->tag(v);
  //     file_graph << vertexTag << " " << v->id() << " ";
  //     v->write(file_graph);
  //     file_graph << endl;
  //   }
  //   for (size_t i = 0; i < svo_verteces.size(); ++i) {
  //     VertexSE3* v = svo_verteces[i];
  //     string vertexTag = Factory::instance()->tag(v);
  //     file_graph << vertexTag << " " << v->id() << " ";
  //     v->write(file_graph);
  //     file_graph << endl;
  //   }
  for (size_t i = 0; i < edges.size(); ++i)
  {
    EdgeSE3 *e = edges[i];

    string edgeTag = Factory::instance()->tag(e);
    VertexSE3 *from = static_cast<VertexSE3 *>(e->vertex(0));
    VertexSE3 *to = static_cast<VertexSE3 *>(e->vertex(1));
    file_graph << edgeTag << " " << from->id() << " " << to->id() << " ";

    e->write(file_graph);

    file_graph << endl;
  }
  for (size_t i = 0; i < vo_edges.size(); ++i)
  {
    EdgeSE3 *e = vo_edges[i];

    string edgeTag = Factory::instance()->tag(e);
    VertexSE3 *from = static_cast<VertexSE3 *>(e->vertex(0));
    VertexSE3 *to = static_cast<VertexSE3 *>(e->vertex(1));
    file_graph << edgeTag << " " << from->id() << " " << to->id() << " ";

    e->write(file_graph);

    file_graph << endl;
  }
  for (size_t i = 0; i < svo_edges.size(); ++i)
  {
    EdgeSE3 *e = svo_edges[i];

    string edgeTag = Factory::instance()->tag(e);
    VertexSE3 *from = static_cast<VertexSE3 *>(e->vertex(0));
    VertexSE3 *to = static_cast<VertexSE3 *>(e->vertex(1));
    file_graph << edgeTag << " " << from->id() << " " << to->id() << " ";

    e->write(file_graph);

    file_graph << endl;
  }
  for (size_t i = 0; i < prior_edges.size(); ++i)
  {
    EdgeSE3Prior *e = prior_edges[i];
    string edgeTag = Factory::instance()->tag(e);
    VertexSE3 *v = static_cast<VertexSE3 *>(e->vertex(0));
    file_graph << edgeTag << " " << v->id() << " "
               << " ";
    //     e->write(file_graph);
    //     file_graph << endl;
    file_graph << 0 << " ";
    Vector7d meas = g2o::internal::toVectorQT(e->measurement());
    for (int h = 0; h < 7; h++)
      file_graph << meas[h] << " ";
    for (int k = 0; k < e->information().rows(); k++)
      for (int j = k; j < e->information().cols(); j++)
      {
        file_graph << e->information()(k, j) << " ";
      }
    file_graph << endl;
  }

  cout << "Graph saved in: " << name << endl;
}





/* major called method ************************************* */



void GraphCreator::create(const vector< nav_msgs::Odometry >& sw,
                          const vector< nav_msgs::Odometry >& wo, 
                          const vector< nav_msgs::Odometry >& gps, 
                          const vector< nav_msgs::Odometry >& vo, 
                          const vector< nav_msgs::Odometry >& gt, 
                          const vector< Eigen::Vector3d >& elevation_grid, 
                          const int& start_idx)
{
        int asdf = 1;
//     vector<VertexSE3*> verteces, vo_verteces, svo_verteces;
//     vector<EdgeSE3*> edges, vo_edges, svo_edges;
//     vector<EdgeSE3Prior*> prior_edges;
//     std::unordered_map<int, Eigen::Vector3d> vo_p_list;
//     for(int i=0; i<sw.size(); i++)
//     {
//       double global_idx=start_idx+i;
//       nav_msgs::Odometry odom_msg;
//       odom_msg=sw[i];
//       Eigen::Isometry3d pose;
//       CovarianceMatrixd cov;
//       tss_utils::poseWithCovarianceMsg2EigenPoseCov(odom_msg.pose, pose,cov);
//       VertexSE3* v = new VertexSE3;
//       v->setEstimate(pose);
//       v->setId(i);
//       verteces.push_back(v);
      
//       nav_msgs::Odometry gps_msg;
//       gps_msg=gps[global_idx];
// //       if(gps_msg.header.seq>0) 
// //       {
//         Eigen::Isometry3d gps_pose;
//         tss_utils::poseWithCovarianceMsg2EigenPoseCov(gps_msg.pose,gps_pose,cov);
//         EdgeSE3Prior* e_gps_prior=new EdgeSE3Prior;
//         e_gps_prior->setVertex(0,v);
//         e_gps_prior->setMeasurement(gps_pose);
//         cov.topLeftCorner(2,2)/=params_.GPS_scale_txy;
//         cov(2,2)/=params_.GPS_scale_tz;
//         e_gps_prior->setInformation(cov.inverse());
//         prior_edges.push_back(e_gps_prior);
//         if(params_.enable_elevation)
//         {
//           EdgeSE3Prior* e_elev_prior=new EdgeSE3Prior;
//           e_elev_prior->setVertex(0,v);
//           Eigen::Isometry3d pos; pos.setIdentity();
//           pos.translation()(2)=computeGmapElevation(gps_pose.translation(), elevation_grid);
//           e_elev_prior->setMeasurement(pos);
//           CovarianceMatrixd elev_info; 
//           elev_info.setZero(6,6);
//           elev_info(2,2)=params_.elevation_scale;
//           e_elev_prior->setInformation(elev_info);
//           prior_edges.push_back(e_elev_prior);
//         }          
// //       }
        
//       if(i>0)
//       {
//         VertexSE3* prev = verteces[i-1];
    
        
//         Eigen::Isometry3d T=prev->estimate().inverse() * v->estimate();
//         EdgeSE3* e = new EdgeSE3;
//         e->setVertex(0, prev);
//         e->setVertex(1, v);
//         e->setMeasurement(T);
//         cov.setIdentity(6,6);
//         cov/=params_.sw_scale;
//         e->setInformation(cov.inverse());
//         edges.push_back(e);
        
        
//         nav_msgs::Odometry prev_wo_msg, wo_msg;
//         prev_wo_msg=wo[global_idx-1]; wo_msg=wo[global_idx];
//         Eigen::Isometry3d prev_wo_pose, wo_pose;
//         tss_utils::poseWithCovarianceMsg2EigenPoseCov(prev_wo_msg.pose, prev_wo_pose,cov);
//         tss_utils::poseWithCovarianceMsg2EigenPoseCov(wo_msg.pose, wo_pose,cov);
//         Eigen::Isometry3d T_wo=prev_wo_pose.inverse() * wo_pose;
//         EdgeSE3* e_wo = new EdgeSE3;
//         e_wo->setVertex(0, prev);
//         e_wo->setVertex(1, v);
//         e_wo->setMeasurement(T_wo);
//         cov.setIdentity(6,6);
//         cov.topLeftCorner(2,2)/=params_.WO_scale_t;
//         cov(2,2)=1000000;
//         cov(3,3)=1000000;
//         cov(4,4)=1000000;
//         cov(5,5)/=params_.WO_scale_r;
//         e_wo->setInformation(cov.inverse());
//         edges.push_back(e_wo);

        
//         nav_msgs::Odometry prev_vo_msg, vo_msg;
//         vo_msg=vo[global_idx]; prev_vo_msg=vo[global_idx-1];
//         if(vo_msg.header.seq>0&&prev_vo_msg.header.seq>0)
//         {
//           Eigen::Isometry3d vo_pose, prev_vo_pose;
//           tss_utils::poseWithCovarianceMsg2EigenPoseCov(vo_msg.pose,vo_pose,cov);
//           tss_utils::poseWithCovarianceMsg2EigenPoseCov(prev_vo_msg.pose,prev_vo_pose,cov);
//           Eigen::Isometry3d diff_SVO_T=(prev_vo_pose.inverse() * vo_pose) ;
//           EdgeSE3* edge=new EdgeSE3;
//           edge->setVertex(0, prev);
//           edge->setVertex(1,v);
//           edge->setMeasurement(diff_SVO_T);
//           cov.setIdentity(6,6);
//           cov.topLeftCorner(2,2)/=params_.SVO_scale_t;
//           cov(2,2)/=params_.SVO_scale_tz;
//           cov.bottomRightCorner(3,3)/=params_.SVO_scale_r;
//           edge->setInformation(cov.inverse());     
//           svo_edges.push_back(edge);
          
//           vo_p_list[i]=vo_pose.translation();
//         }
//       }
//     }
//     if(params_.enable_MA)
//     {  
//       for(unordered_map<int, Eigen::Vector3d>::iterator it=vo_p_list.begin(); it!=vo_p_list.end();)
//       {
//         Eigen::Vector3d ref_p=it->second;
//         int ref_i=it->first;
//         VertexSE3* ref_v = verteces[ref_i];
//         it++;   
//         for(unordered_map<int, Eigen::Vector3d>::iterator it2=it; it2!=vo_p_list.end(); it2++)
//         {
//           Eigen::Vector3d curr_p=it2->second;
//           int curr_i=it2->first;
//           double norm=(curr_p.head(2)-ref_p.head(2)).norm();
//           if(norm<params_.MA_radius)
//           {
//             VertexSE3* curr_v = verteces[curr_i];
//             Eigen::Vector3d diff=(curr_p-ref_p);
//             double z_diff=fabs(diff(2));
//     //         if(z_diff<.25) continue;
//             double k=params_.MA_scale/norm;
            
//             EdgeSE3* edge=new EdgeSE3;
//             edge->setVertex(0, ref_v);
//             edge->setVertex(1,curr_v);
//             Eigen::Isometry3d I; I.setIdentity();
//   //             I.translation().head(2)=-(curr_p.head(2)-ref_p.head(2));
//             edge->setMeasurement(I);
//             CovarianceMatrixd info; info.setZero(6,6);
//   //             info(0,0)=k/100;
//   //             info(1,1)=k/100;
//             info(2,2)=k;
//             edge->setInformation(info);     
//             svo_edges.push_back(edge);
//           }
//         }
//       }
//     }
    
//     GraphCreator::writeGraph(params_.graph_filename, verteces, vo_verteces, svo_verteces, edges, vo_edges, svo_edges, prior_edges);
}



void GraphCreator::create(const std::vector<nav_msgs::Odometry>& wo,
                          const std::vector<nav_msgs::Odometry>& gps,
                          const std::vector<nav_msgs::Odometry>& vo,
                          const std::vector<nav_msgs::Odometry>& gt,
                          const std::vector<Eigen::Vector3d>& elevation_grid, 
                          const std::vector<nav_msgs::Odometry>& imu,
                          const std::vector<sensor_msgs::PointCloud2>& clouds,
                          vector<int>& temporal_WO_indices
                          )
{
  
  temporal_WO_indices.clear();
  vector<VertexSE3*> verteces, vo_verteces, svo_verteces;
  vector<EdgeSE3*> edges, vo_edges, svo_edges;
  vector<EdgeSE3Prior*> prior_edges;
  
  double iter=0;
  Eigen::Isometry3d last_pose; last_pose.setIdentity();
  
  int prev_vertex_idx=-1;
  
  last_pose.translation()=Eigen::Vector3d(-100000, -10000, -10000);
  for(int i=0; i<wo.size(); i++)
  {
    /// FOR Leica filtering
    nav_msgs::Odometry leica_gt=gt[i];
    Eigen::Isometry3d gt_pose;
    CovarianceMatrixd gt_cov;
    if(leica_gt.header.seq>0)
    {
      tss_utils::poseWithCovarianceMsg2EigenPoseCov(leica_gt.pose, gt_pose,gt_cov);
      if(gt_pose.translation().norm()>1000)
      {
        cout<<"leica filtered out... translation: "<<gt_pose.translation().transpose()<<endl;
        continue;
      }
    }
    
    nav_msgs::Odometry wheel_odom_msg;
    wheel_odom_msg=wo[i];
    Eigen::Isometry3d pose;
    CovarianceMatrixd cov;
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(wheel_odom_msg.pose, pose,cov);
    
    // cout<<params_.WO_step<<" "<<GraphCreator::getPosesDiff(pose, last_pose).translation().norm()<<endl;

    if(GraphCreator::getPosesDiff(pose, last_pose).translation().norm()>params_.WO_step&&
      (gps[i].header.seq>0)&&
      (imu[i].header.seq>0)&&
      (vo[i].header.seq>0))
    {      
      temporal_WO_indices.push_back(i);
      cout << "aaaaaaaaaaa " << endl;
      g2o::VertexSE3* v = new g2o::VertexSE3();
      cout << "bbbbbbbbbbbb " << endl;
      v->setEstimate(pose);
      cout << "ccccccc " << endl;
      v->setId(i);
      if(verteces.size()==0)
        v->setFixed(true);
      verteces.push_back(v);
      iter++;
      last_pose=pose;
     
      
      if(params_.enable_imu)
      {
        nav_msgs::Odometry imu_msg;
        imu_msg=imu[i];
        if(imu_msg.header.seq>0)
        {
          Eigen::Isometry3d pose;
          CovarianceMatrixd info_imu;
          tss_utils::poseWithCovarianceMsg2EigenPoseCov(imu_msg.pose,pose,info_imu);
          g2o::EdgeSE3Prior* e_imu_prior=new g2o::EdgeSE3Prior();
          e_imu_prior->setVertex(0,v);
          e_imu_prior->setMeasurement(pose);
          info_imu.setIdentity(6,6);
          info_imu.topLeftCorner(3,3)*=0;
          info_imu.bottomRightCorner(3,3)*=params_.IMU_scale;
          info_imu(5,5)=0;
          
          e_imu_prior->setInformation(info_imu);
          prior_edges.push_back(e_imu_prior);
        }
      }
      
      if(verteces.size()>1)
      {
        VertexSE3* prev = verteces[verteces.size()-2];
    
        Eigen::Isometry3d T=prev->estimate().inverse() * v->estimate();
        g2o::EdgeSE3* e = new g2o::EdgeSE3();
        e->setVertex(0, prev);
        e->setVertex(1, v);
        e->setMeasurement(T);
        cov.setIdentity(6,6);
        cov.topLeftCorner(2,2)/=params_.WO_scale_t;
        cov(2,2)=1000000;
        cov(3,3)=1000000;
        cov(4,4)=1000000;
        cov(5,5)/=params_.WO_scale_r;
        e->setInformation(cov.inverse());
        edges.push_back(e);
        
      }
    }

  }



  if(params_.enable[1])
  {    
    std::unordered_map<int, Eigen::Vector3d> vo_p_list;

    iter=0;
    last_pose.setIdentity();
    last_pose.translation()=Eigen::Vector3d(-100000, -10000, -10000);

    cout << "bbbbbbbbbbbbbb" << endl;




    for(int i=0; i<temporal_WO_indices.size(); i++, iter++)
    {
      nav_msgs::Odometry rtk_odom_msg;
      rtk_odom_msg=gps[temporal_WO_indices[i]];
      if(rtk_odom_msg.header.seq<=0) continue;
      Eigen::Isometry3d pose;
      CovarianceMatrixd cov;
      tss_utils::poseWithCovarianceMsg2EigenPoseCov(rtk_odom_msg.pose,pose,cov);
      double step=params_.GPS_step;

      if(GraphCreator::getPosesDiff(pose, last_pose).translation().norm()>=step)
      {
        VertexSE3* v = verteces[iter];
        g2o::EdgeSE3Prior* e_gps_prior=new g2o::EdgeSE3Prior();
        e_gps_prior->setVertex(0,v);
        e_gps_prior->setMeasurement(pose);
/*        
        cov(0,0)=0.5625;
        cov(1,1)=0.5625;
        cov(2,2)=2.25;
        */

        if(cov(0,0)==0&&cov(1,1)==0&&cov(2,2)==0)
          cov.setIdentity(6,6);
        
//         /// planar assumption
//         if(cov(2,2)>10)
//         {
//           pose.translation()(2)=0;
//           e_gps_prior->setMeasurement(pose);
//         }
        
        if(cov(2,2)>20)
        {
          cov.topLeftCorner(2,2)/=params_.GPS_scale_txy/100;
          cov(2,2)/=params_.GPS_scale_tz/100;
        }
        else
        {
          cov.topLeftCorner(2,2)/=params_.GPS_scale_txy;
          cov(2,2)/=params_.GPS_scale_tz;
          
        }
        cov=cov.inverse();
        cov.bottomRightCorner(3,3)*=0;
        e_gps_prior->setInformation(cov);
        prior_edges.push_back(e_gps_prior);
        last_pose=pose;
        
        vo_p_list[iter]=pose.translation();
        
        if(params_.enable_elevation)
        {
          g2o::EdgeSE3Prior* e_elev_prior=new g2o::EdgeSE3Prior();
          e_elev_prior->setVertex(0,v);
          Eigen::Isometry3d pos; pos.setIdentity();
          pos.translation()(2)=computeGmapElevation(pose.translation(), elevation_grid);
          e_elev_prior->setMeasurement(pos);
          CovarianceMatrixd elev_info; 
          elev_info.setZero(6,6);
          elev_info(2,2)=params_.elevation_scale;
          e_elev_prior->setInformation(elev_info);
          prior_edges.push_back(e_elev_prior);
        }
      }
    }
  }
  
  if(params_.enable[3])
  {
    iter=0;
    last_pose.setIdentity();
    last_pose.translation()=Eigen::Vector3d(-100000, -10000, -10000);
    int prev_SVO_iter=-1;
    
    std::unordered_map<int, Eigen::Vector3d> vo_p_list;
    
//     bool isCalib=false;
//     Eigen::Isometry3d calib_T;
    for(int i=0; i<temporal_WO_indices.size(); i++, iter++)
    {
      nav_msgs::Odometry vo_odom_msg;
      vo_odom_msg=vo[temporal_WO_indices[i]];
      if(vo_odom_msg.header.seq<=0) continue;
      Eigen::Isometry3d pose;
      CovarianceMatrixd cov;
      tss_utils::poseWithCovarianceMsg2EigenPoseCov(vo_odom_msg.pose,pose,cov);

      if(GraphCreator::getPosesDiff(pose, last_pose).translation().norm()>params_.SVO_step)
      {
        if(prev_SVO_iter>=0)
        {
          VertexSE3* prev_WO_v = verteces[prev_SVO_iter];
          VertexSE3* curr_WO_v = verteces[iter];
          Eigen::Isometry3d prev_WO_T=prev_WO_v->estimate();
          Eigen::Isometry3d curr_WO_T=curr_WO_v->estimate();
          Eigen::Isometry3d diff_WO_T=prev_WO_T.inverse() * curr_WO_T;
          
          
          Eigen::Isometry3d diff_SVO_T=((last_pose).inverse() * pose) ;

          g2o::EdgeSE3* edge=new g2o::EdgeSE3();
          edge->setVertex(0, prev_WO_v);
          edge->setVertex(1,curr_WO_v);
          edge->setMeasurement(diff_SVO_T);
          cov.setIdentity(6,6);
          cov.topLeftCorner(2,2)/=params_.SVO_scale_t;
          cov(2,2)/=params_.SVO_scale_tz;
          cov.bottomRightCorner(3,3)/=params_.SVO_scale_r;
          edge->setInformation(cov.inverse());     
          svo_edges.push_back(edge);

          vo_p_list[iter]=pose.translation();
          
          if(params_.enable_ackerman)
          {
            Eigen::Quaterniond q(diff_SVO_T.linear()); q.x()=0; q.y()=0;
            if(q.w()<0)
              q.z()*=-1;
            
            q.w()=sqrt(1-(q.z()*q.z()));
            double yaw=q.toRotationMatrix().eulerAngles(0,1,2)(2);
            double disp=diff_SVO_T.translation().head(2).norm();
            if(diff_SVO_T.translation()(0)<0)
              disp*=-1;
            double x=disp*cos(yaw);
            double y=disp*sin(yaw);
            
            std::cout<<"yaw: "<<yaw*180/M_PI<<endl;
            
            Eigen::Quaterniond q_yaw;
            q_yaw = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                  * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
                  
            Eigen::Quaterniond q_rollpitch(diff_SVO_T.linear()); q_rollpitch.z()=0;
            q_rollpitch.w()=sqrt(1-(q_rollpitch.x()*q_rollpitch.x() + q_rollpitch.y()*q_rollpitch.y()));
            Eigen::Isometry3d T_plane; T_plane.setIdentity();
            T_plane.linear()=q_rollpitch.toRotationMatrix();
            
            std::cout<<"pitch: "<<T_plane.linear().eulerAngles(0,1,2)(1)*180/M_PI<<endl;
            std::cout<<"roll: "<<T_plane.linear().eulerAngles(0,1,2)(0)*180/M_PI<<endl;
                  
            Eigen::Isometry3d T_ackerman;
            T_ackerman.translation()=Eigen::Vector3d(x, y, 0);
            T_ackerman.linear()=q_yaw.toRotationMatrix();
//             T_ackerman=T_plane*T_ackerman;
            CovarianceMatrixd info_ackerman;
            info_ackerman.setIdentity(6,6);
            
            info_ackerman.topLeftCorner(2,2)*=params_.ACKERMAN_scale_t/fabs(disp);
            info_ackerman(2,2)=0;
            info_ackerman(3,3)=0;
            info_ackerman(4,4)=0;
            info_ackerman(5,5)*=params_.ACKERMAN_scale_r/fabs(disp);
            
//             info_ackerman.topLeftCorner(3,3)*=params_.ACKERMAN_scale_t/fabs(disp);
//             info_ackerman.bottomRightCorner(3,3)*=params_.ACKERMAN_scale_r/fabs(disp);

            
            g2o::EdgeSE3* edge_ackerman=new g2o::EdgeSE3();
            edge_ackerman->setVertex(0, prev_WO_v);
            edge_ackerman->setVertex(1,curr_WO_v);
            edge_ackerman->setMeasurement(T_ackerman);
            edge_ackerman->setInformation(info_ackerman);     
            svo_edges.push_back(edge_ackerman);
          }
        }

        last_pose=pose;
        prev_SVO_iter=iter;
      }
    }
    
    if(params_.enable_MA)
    {
      for(unordered_map<int, Eigen::Vector3d>::iterator it=vo_p_list.begin(); it!=vo_p_list.end();)
      {
        Eigen::Vector3d ref_p=it->second;
        int ref_i=it->first;
        VertexSE3* ref_v = verteces[ref_i];
        it++;
        
        for(unordered_map<int, Eigen::Vector3d>::iterator it2=it; it2!=vo_p_list.end(); it2++)
        {
          Eigen::Vector3d curr_p=it2->second;
          int curr_i=it2->first;
          double norm=(curr_p.head(2)-ref_p.head(2)).norm();
          if(norm<params_.MA_radius)
          {
            VertexSE3* curr_v = verteces[curr_i];
            Eigen::Vector3d diff=(curr_p-ref_p);
            double z_diff=fabs(diff(2));
    //         if(z_diff<.25) continue;
            double k=params_.MA_scale/norm;
            
            g2o::EdgeSE3* edge=new g2o::EdgeSE3();
            edge->setVertex(0, ref_v);
            edge->setVertex(1,curr_v);
            Eigen::Isometry3d I; I.setIdentity();
//             I.translation().head(2)=-(curr_p.head(2)-ref_p.head(2));
            edge->setMeasurement(I);
            CovarianceMatrixd info; info.setZero(6,6);
//             info(0,0)=k/100;
//             info(1,1)=k/100;
            info(2,2)=k;
            edge->setInformation(info);     
            svo_edges.push_back(edge);
          }
        }
      }
    }
  }
  writeGraph(params_.graph_filename, verteces, vo_verteces, svo_verteces, edges, vo_edges, svo_edges, prior_edges);
}
