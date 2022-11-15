#include "g2o_optimizer.h"
#include "graph_creator.h"
#include <fstream>

using namespace std;
using namespace g2o;

G2OOptimizer::G2OOptimizer(): QObject()
{
  BlockSolverX::LinearSolverType * linearSolver = new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();
  BlockSolverX* blockSolver = new BlockSolverX(linearSolver);
  OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);
  
  optimizer_.setVerbose(true);
  optimizer_.setAlgorithm(optimizationAlgorithm);
}

void G2OOptimizer::optimize(const std::vector<nav_msgs::Odometry>& poses, std::vector<nav_msgs::Odometry>& opt_poses, bool manifold_assumption)
{
  if(!manifold_assumption)
    performOptimization();
  else
  {
    addManifoldAssumptionEdges(params_.out_filename);
    
    SparseOptimizer optimizer;
    BlockSolverX::LinearSolverType * linearSolver = new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();
    BlockSolverX* blockSolver = new BlockSolverX(linearSolver);
    OptimizationAlgorithmLevenberg* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);
    
    optimizer.setVerbose(true);
    optimizer.setAlgorithm(optimizationAlgorithm);
      
    //   optimizer.clear();
    ifstream ifs(params_.out_filename);
  //     ifs.open(params_.out_filename);
    optimizer.load(ifs);

    
    AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator("Huber");
    for (SparseOptimizer::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
        OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
          e->setRobustKernel(creator->construct());
          e->robustKernel()->setDelta(1);
    }
    
          
    optimizer.initializeOptimization();
    optimizer.optimize(params_.N_iterations/4);

    if (params_.out_filename.size() > 0) {
      if (params_.out_filename == "-") {
        cerr << "saving to stdout";
        optimizer.save(cout);
      } else {
        cerr << "saving " << params_.out_filename << " ... ";
        optimizer.save(params_.out_filename.c_str());
      }
      cerr << "done." << endl;
    }
  }
  
  getOptimizedPoses(poses, opt_poses);
}

void G2OOptimizer::addManifoldAssumptionEdges(const string& filename)
{
  
  ifstream ifs(params_.out_filename);
  std::unordered_map<int, Eigen::Vector3d> p_list;
  while(!ifs.eof())
  {
    string tag;
    ifs>>tag;
    if(strcmp(tag.c_str(),"VERTEX_SE3:QUAT")==0)
    {     
      int i;
      ifs>>i;
      
      double tx,ty,tz, qw,qx,qy,qz;
      ifs>>tx;ifs>>ty;ifs>>tz;
      ifs>>qw;ifs>>qx;ifs>>qy;ifs>>qz;
      
      p_list[i]=Eigen::Vector3d(tx,ty,tz);     
    }
  }
  ifs.close();
  
  ofstream ofs(params_.out_filename, std::ofstream::app);
  for(unordered_map<int, Eigen::Vector3d>::iterator it=p_list.begin(); it!=p_list.end();)
  {
    Eigen::Vector3d ref_p=it->second;
    int ref_i=it->first;
    it++;
    for(unordered_map<int, Eigen::Vector3d>::iterator it2=it; it2!=p_list.end(); it2++)
    {
      Eigen::Vector3d curr_p=it2->second;
      int curr_i=it2->first;
      double norm=(curr_p.head(2)-ref_p.head(2)).norm();
      if(norm<graph_params_.MA_radius)
      {
        Eigen::Vector3d diff=(curr_p-ref_p);
        double z_diff=fabs(diff(2));
//         if(z_diff<.25) continue;
        double k=graph_params_.MA_scale/norm;
        stringstream ss;
//         ss<<"EDGE_SE3:QUAT "<< ref_i <<" "<<curr_i<<" "<<diff(0)<<" "<<diff(1)<<" 0 0 0 0 1 1 0 0 0 0 0 1 0 0 0 0 "<<k<<" 0 0 0 0 0 0 0 0 0"<<endl;
        ss<<"EDGE_SE3:QUAT "<< ref_i <<" "<<curr_i<<" 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 "<<k<<" 0 0 0 0 0 0 0 0 0"<<endl;
        ofs<<ss.str();
      }
    }
  }
  ofs.close();
}

void G2OOptimizer::getOptimizedPoses(const std::vector<nav_msgs::Odometry>& poses, vector< nav_msgs::Odometry >& opt_poses)
{
  opt_poses.clear();
  opt_poses.resize(poses.size());
  ifstream ifs(params_.out_filename);
  while(!ifs.eof())
  {
    string tag;
    ifs>>tag;
    if(strcmp(tag.c_str(),"VERTEX_SE3:QUAT")==0)
    {     
      Eigen::Isometry3d pose;
      int i;
      ifs>>i;
      
      if(i>=poses.size()||i<0)
      {
        nav_msgs::Odometry dummy; dummy.header.seq=0;
        opt_poses[i]=(dummy); 
        continue;
      }
      
      opt_poses[i]=poses[i];
      
      double tx,ty,tz, qw,qx,qy,qz;
      ifs>>tx;ifs>>ty;ifs>>tz;
      ifs>>qx;ifs>>qy;ifs>>qz;ifs>>qw;
      
      opt_poses[i].pose.pose.position.x=tx;
      opt_poses[i].pose.pose.position.y=ty;
      opt_poses[i].pose.pose.position.z=tz;
      
      opt_poses[i].pose.pose.orientation.x=qx;
      opt_poses[i].pose.pose.orientation.y=qy;
      opt_poses[i].pose.pose.orientation.z=qz;
      opt_poses[i].pose.pose.orientation.w=qw;
    }
  }
}

void G2OOptimizer::computeUtilezedOdometries(const vector< nav_msgs::Odometry >& wo, const vector< nav_msgs::Odometry >& gps, 
                                             const std::vector<nav_msgs::Odometry>& vo,
                                             const std::vector<nav_msgs::Odometry>& gt,
                                             vector< nav_msgs::Odometry >& used_wo, vector< nav_msgs::Odometry >& used_gps, 
                                             vector< nav_msgs::Odometry >& used_vo,
                                             vector< nav_msgs::Odometry >& used_gt
                                             )
{
  vector<int> temporal_WO_indices;
  double iter=0;
  Eigen::Isometry3d last_pose; last_pose.setIdentity();
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
//     cout<<params_.WO_step<<" "<<GraphCreator::getPosesDiff(pose, last_pose).translation().norm()<<endl;
    if(GraphCreator::getPosesDiff(pose, last_pose).translation().norm()>graph_params_.WO_step&&
      (gps[i].header.seq>0)&&
      (gt[i].header.seq>0)&&
      (vo[i].header.seq>0))
    {      
      temporal_WO_indices.push_back(i);
      used_wo.push_back(wo[i]);
      used_gps.push_back(gps[i]);
      used_vo.push_back(vo[i]);
      used_gt.push_back(gt[i]);
      iter++;
      last_pose=pose;
      
    }
  }

}


// void G2OOptimizer::SWoptimize(const std::vector<nav_msgs::Odometry>& _wo,
//                               const std::vector<nav_msgs::Odometry>& _gps,
//                               const std::vector<nav_msgs::Odometry>& _vo,
//                               const std::vector<nav_msgs::Odometry>& _gt,
//                               const std::vector<Eigen::Vector3d>& _elevation_grid
//                               )
// {
//   
//   cout<<"SLOT SWoptimized"<<endl;
//   
//   std::vector<nav_msgs::Odometry> sw, prev_sw;
//   int start_idx, end_idx;
//   
//   std::vector<nav_msgs::Odometry> wo, gps, vo, gt, final_sw_opt, final_opt;
//   computeUtilezedOdometries(_wo, _gps, _vo, _gt, wo,gps,vo,gt);
//   
//   
//   int N=graph_params_.sw_size;
//   for(int i=0; i<N; i++)
//     sw.push_back(wo[i]);
//   
//   start_idx=0; end_idx=N-1;
// 
//   GraphCreator* gc=new GraphCreator();
//   gc->setParams(graph_params_);
//   
//   double RMSE=0;
//   while(start_idx<wo.size()-N)
//   {
//     gc->create(sw, wo, gps, vo, gt, _elevation_grid, start_idx);
// //     ifstream ifs(params_.in_filename);
// //     optimizer.clear();
// //     optimizer.load(ifs);
// //     
// //     AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator("Cauchy");
// //     for (SparseOptimizer::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
// //         OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
// //           e->setRobustKernel(creator->construct());
// //           e->robustKernel()->setDelta(1);
// //     }
// //     optimizer.initializeOptimization();
// //     optimizer.optimize(params_.N_iterations);
// // 
// //     if (params_.out_filename.size() > 0) {
// //       if (params_.out_filename == "-") {
// //         cerr << "saving to stdout";
// //         optimizer.save(cout);
// //       } else {
// //         cerr << "saving " << params_.out_filename << " ... ";
// //         optimizer.save(params_.out_filename.c_str());
// //       }
// //       cerr << "done." << endl;
// //     }
// //     ifs.close();
//     performOptimization();
//     prev_sw=sw;
//     getOptimizedPoses(prev_sw, sw);
//     
//     Eigen::Vector3d err;
//     err=computeError(sw[0], gt[start_idx]);
//     RMSE+=err.squaredNorm();
//     cout<<"Err Norm: "<<err.norm()<<endl;
//     
//     emit showPosesSignal(sw);
//     
//     start_idx++;
//     prev_sw=sw;
//     sw.clear();
//     sw.resize(N);
//     for(int i=0; i<N-1; i++)
//       sw[i]=prev_sw[i+1];
//     
//     final_sw_opt.push_back(prev_sw[0]);
//     
//     nav_msgs::Odometry prev_wo_msg, wo_msg;
//     prev_wo_msg=wo[start_idx+N-2]; wo_msg=wo[start_idx+N-1];
//     Eigen::Isometry3d prev_wo_pose, wo_pose, prev_sw_pose;
//     CovarianceMatrixd cov;
//     tss_utils::poseWithCovarianceMsg2EigenPoseCov(prev_wo_msg.pose, prev_wo_pose,cov);
//     tss_utils::poseWithCovarianceMsg2EigenPoseCov(wo_msg.pose, wo_pose,cov);
//     tss_utils::poseWithCovarianceMsg2EigenPoseCov(sw[N-2].pose, prev_sw_pose,cov);
//     Eigen::Vector3d T_wo=wo_pose.translation()-prev_wo_pose.translation();
//     Eigen::Vector3d pos2=prev_sw_pose.translation()+T_wo;
//     nav_msgs::Odometry p=wo[start_idx+N-1];
//     p.pose.pose.position.x=pos2(0);
//     p.pose.pose.position.y=pos2(1);
//     p.pose.pose.position.z=pos2(2);
//     
//     sw[N-1]=p;
//     
//     std::cout<<"sw start_idx: "<<start_idx<<endl;
//     
//   }//end while
//   cout<<"Final sw RMSE: "<<sqrt(RMSE/start_idx)<<endl;
//   
//   for(int i=1; i<prev_sw.size(); i++)
//     final_sw_opt.push_back(prev_sw[i]);
//   emit showPosesSignal(final_sw_opt);
//   
//   start_idx=0;
//   gc->create(final_sw_opt, wo, gps, vo, gt, _elevation_grid, start_idx);
//   
// //   ifstream ifs(params_.in_filename);
// //   optimizer.clear();
// //   optimizer.load(ifs);
// //   
// //   AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator("Cauchy");
// //   for (SparseOptimizer::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
// //       OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
// //         e->setRobustKernel(creator->construct());
// //         e->robustKernel()->setDelta(1);
// //   }
// //   optimizer.initializeOptimization();
// //   optimizer.optimize(params_.N_iterations);
// // 
// //   if (params_.out_filename.size() > 0) {
// //     if (params_.out_filename == "-") {
// //       cerr << "saving to stdout";
// //       optimizer.save(cout);
// //     } else {
// //       cerr << "saving " << params_.out_filename << " ... ";
// //       optimizer.save(params_.out_filename.c_str());
// //     }
// //     cerr << "done." << endl;
// //   }
// //   ifs.close();
//   performOptimization();
//   getOptimizedPoses(final_sw_opt, final_opt);
//   cout<<"final_opt size: "<<final_opt.size()<<endl;
//   emit showPosesSignal(final_opt);
// }

void G2OOptimizer::SWoptimize(const std::vector<nav_msgs::Odometry>& _wo,
                              const std::vector<nav_msgs::Odometry>& _gps,
                              const std::vector<nav_msgs::Odometry>& _vo,
                              const std::vector<nav_msgs::Odometry>& _gt,
                              const std::vector<Eigen::Vector3d>& _elevation_grid
                              )
{
  
  cout<<"SLOT SWoptimized"<<endl;
  
  std::vector<nav_msgs::Odometry> sw, prev_sw;
  int start_idx, end_idx;
  
  std::vector<nav_msgs::Odometry> wo, gps, vo, gt, final_sw_opt, final_opt;
  computeUtilezedOdometries(_wo, _gps, _vo, _gt, wo,gps,vo,gt);
  
  
  int N=graph_params_.sw_size;
  for(int i=0; i<N; i++)
    sw.push_back(wo[i]);
  
  start_idx=0; end_idx=N-1;

  GraphCreator* gc=new GraphCreator();
  gc->setParams(graph_params_);
  
  double RMSE=0;
  int current_idx=N;
  final_sw_opt.resize(wo.size());
  while(current_idx<wo.size())
  {
    gc->create(sw, wo, gps, vo, gt, _elevation_grid, start_idx);
//     ifstream ifs(params_.in_filename);
//     optimizer.clear();
//     optimizer.load(ifs);
//     
//     AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator("Cauchy");
//     for (SparseOptimizer::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
//         OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
//           e->setRobustKernel(creator->construct());
//           e->robustKernel()->setDelta(1);
//     }
//     optimizer.initializeOptimization();
//     optimizer.optimize(params_.N_iterations);
// 
//     if (params_.out_filename.size() > 0) {
//       if (params_.out_filename == "-") {
//         cerr << "saving to stdout";
//         optimizer.save(cout);
//       } else {
//         cerr << "saving " << params_.out_filename << " ... ";
//         optimizer.save(params_.out_filename.c_str());
//       }
//       cerr << "done." << endl;
//     }
//     ifs.close();
    performOptimization();
    prev_sw=sw;
    getOptimizedPoses(prev_sw, sw);
    
    Eigen::Vector3d err;
    err=computeError(sw[0], gt[start_idx]);
    RMSE+=err.squaredNorm();
    cout<<"Err Norm: "<<err.norm()<<endl;
    
    emit showPosesSignal(sw);
    
    for(int i=0; i<sw.size(); i++)
    {
      final_sw_opt[start_idx+i]=sw[i]; 
    }
    
    current_idx++;
//     cout<<"debug0"<<endl;
    start_idx=computeSWstart_idx(gps, current_idx);
//     cout<<"debug0 "<<start_idx<<" "<<current_idx<<endl;
    ((current_idx-start_idx) < N || start_idx<0)? start_idx=current_idx-N: start_idx;
    
//     start_idx++;
    prev_sw=sw;
    sw.clear();
    sw.resize(current_idx-start_idx+1);
    for(int i=current_idx-1; i>=start_idx; i--)
    {
//       std::cout<<"debug1: "<<i<<" "<<sw.size()<<" "<<i-start_idx<<endl;
      sw[i-start_idx]=final_sw_opt[i];
    }
    
    //final_sw_opt.push_back(prev_sw[0]);
    
    
    nav_msgs::Odometry prev_wo_msg, wo_msg;
    prev_wo_msg=wo[current_idx-1]; wo_msg=wo[current_idx];
    Eigen::Isometry3d prev_wo_pose, wo_pose, prev_sw_pose;
    CovarianceMatrixd cov;
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(prev_wo_msg.pose, prev_wo_pose,cov);
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(wo_msg.pose, wo_pose,cov);
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(sw[sw.size()-2].pose, prev_sw_pose,cov);
    Eigen::Vector3d T_wo=wo_pose.translation()-prev_wo_pose.translation();
    Eigen::Vector3d pos2=prev_sw_pose.translation()+T_wo;
    nav_msgs::Odometry p=wo[current_idx];
    p.pose.pose.position.x=pos2(0);
    p.pose.pose.position.y=pos2(1);
    p.pose.pose.position.z=pos2(2);
    
    sw[sw.size()-1]=p;
    
    std::cout<<"sw start_idx: "<<start_idx<<endl;
    
  }//end while
//   cout<<"Final sw RMSE: "<<sqrt(RMSE/start_idx)<<endl;
  
//   for(int i=1; i<prev_sw.size(); i++)
//     final_sw_opt.push_back(prev_sw[i]);
  emit showPosesSignal(final_sw_opt);
  
  start_idx=0;
//   gc->create(final_sw_opt, wo, gps, vo, gt, _elevation_grid, start_idx);
  
//   ifstream ifs(params_.in_filename);
//   optimizer.clear();
//   optimizer.load(ifs);
//   
//   AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator("Cauchy");
//   for (SparseOptimizer::EdgeSet::const_iterator it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it) {
//       OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
//         e->setRobustKernel(creator->construct());
//         e->robustKernel()->setDelta(1);
//   }
//   optimizer.initializeOptimization();
//   optimizer.optimize(params_.N_iterations);
// 
//   if (params_.out_filename.size() > 0) {
//     if (params_.out_filename == "-") {
//       cerr << "saving to stdout";
//       optimizer.save(cout);
//     } else {
//       cerr << "saving " << params_.out_filename << " ... ";
//       optimizer.save(params_.out_filename.c_str());
//     }
//     cerr << "done." << endl;
//   }
//   ifs.close();
  
//    performOptimization();
//   getOptimizedPoses(final_sw_opt, final_opt);
//   cout<<"final_opt size: "<<final_opt.size()<<endl;
//   emit showPosesSignal(final_opt);
}

int G2OOptimizer::computeSWstart_idx(const vector< nav_msgs::Odometry >& gps, const int current_idx)
{
  
  int limit_idx=current_idx-120;
  (limit_idx<0)? limit_idx=0:limit_idx;
  Eigen::Isometry3d pose;
  CovarianceMatrixd cov;
  tss_utils::poseWithCovarianceMsg2EigenPoseCov(gps[current_idx].pose,pose,cov);
  float min_idx=-1;
  for(int i=current_idx-1; i>=limit_idx; i--)
  { 
    Eigen::Isometry3d vo_pose, prev_vo_pose;
    
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(gps[i].pose,prev_vo_pose,cov); 
    
    if((prev_vo_pose.translation().head(2)-pose.translation().head(2)).norm()<1.2)
    {
      min_idx=i; 
    }
  }
  return min_idx;
}


void G2OOptimizer::performOptimization()
{
  ifstream ifs(params_.in_filename);
  optimizer_.clear();
  optimizer_.load(ifs);
  
  AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator("Huber");
  for (SparseOptimizer::EdgeSet::const_iterator it = optimizer_.edges().begin(); it != optimizer_.edges().end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
        e->setRobustKernel(creator->construct());
        e->robustKernel()->setDelta(1);
  }
  OptimizableGraph::Vertex* v_aux;
  int id_min=10000000;
  for (SparseOptimizer::VertexIDMap::const_iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it) {
      
    OptimizableGraph::Vertex* v= static_cast<OptimizableGraph::Vertex*>(it->second);
    if (v->id()==0)
      continue;
    if(id_min>v->id())
    {
         
         id_min=v->id();
         v_aux=v;
    }
       
         

    }
     v_aux->setFixed(true);
//     cout<<"FIXEDDDDDDDDDDDDDDDD"<<endl;
    
  optimizer_.initializeOptimization();
  optimizer_.optimize(params_.N_iterations);

  if (params_.out_filename.size() > 0) {
    if (params_.out_filename == "-") {
      cerr << "saving to stdout";
      optimizer_.save(cout);
    } else {
      cerr << "saving " << params_.out_filename << " ... ";
      optimizer_.save(params_.out_filename.c_str());
    }
    cerr << "done." << endl;
  }
  ifs.close();
}

Eigen::Vector3d G2OOptimizer::computeError(const nav_msgs::Odometry& odom, const nav_msgs::Odometry& gt)
{
  Eigen::Vector3d odom_eigen, gt_eigen;
  odom_eigen(0)=odom.pose.pose.position.x;
  odom_eigen(1)=odom.pose.pose.position.y;
  odom_eigen(2)=odom.pose.pose.position.z;
  gt_eigen(0)=gt.pose.pose.position.x;
  gt_eigen(1)=gt.pose.pose.position.y;
  gt_eigen(2)=gt.pose.pose.position.z;
  
  return odom_eigen-gt_eigen;
}





