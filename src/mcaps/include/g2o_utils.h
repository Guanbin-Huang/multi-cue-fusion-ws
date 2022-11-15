#pragma once

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/core/factory.h"
//#include "g2o/types/slam3d/types_slam3d.h"
//#include "g2o/types/slam2d/types_slam2d.h"

#include "g2o/stuff/command_args.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/estimate_propagator.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"

#include "paramsUtils.h"

void g2oGraph2EigenPoses(const std::string graph_filename, std::vector<Eigen::Isometry3d>& poses)
{
  poses.clear();
  std::ifstream ifs(graph_filename);
  string tag; 
  while(!ifs.eof())
  {    
    ifs>>tag;
    if(strcmp(tag.c_str(),"VERTEX_SE3:QUAT")==0)
    {     
      Eigen::Isometry3d pose;
      int i;
      ifs>>i;
      double tx,ty,tz, qw,qx,qy,qz;
      ifs>>tx;ifs>>ty;ifs>>tz;
      pose.translation()=Eigen::Vector3d(tx,ty,tz);
      ifs>>qw;ifs>>qx;ifs>>qy;ifs>>qz;
      Eigen::Quaterniond q; q.w()=qw; q.x()=qx; q.y()=qy; q.z()=qz;
      pose.linear()=q.toRotationMatrix();    
      poses.push_back(pose);      
    }
  }
}