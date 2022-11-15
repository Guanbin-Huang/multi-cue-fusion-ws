#ifndef G2O_OPTIMIZER_H
#define G2O_OPTIMIZER_H

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

#include "interpolation_utils.h"
#include "paramsUtils.h"
#include "pose_graph/temporalgraph.h"
#include "pose_graph/tss_utils.h"

#include <QtGui>

class G2OOptimizer : public QObject {
  Q_OBJECT
public:
  
  G2OOptimizer();
    
  void optimize(const std::vector<nav_msgs::Odometry>& poses, std::vector<nav_msgs::Odometry>& opt_poses, bool manifold_assumption);
//   void SWoptimize(const std::vector<nav_msgs::Odometry>& wo,
//                   const std::vector<nav_msgs::Odometry>& gps,
//                   const std::vector<nav_msgs::Odometry>& vo,
//                   const std::vector<nav_msgs::Odometry>& gt,
//                   const std::vector<Eigen::Vector3d>& elevation_grid
//                   );
  void setParams(const Configuration::GraphParams& g_params,const Configuration::OptimizerParams& params)
  {
    graph_params_=g_params;
    params_=params;
  };
  
Q_SIGNALS:

  void showPosesSignal(std::vector<nav_msgs::Odometry> opt_poses);
  
public Q_SLOTS:
  
  void SWoptimize(const std::vector<nav_msgs::Odometry>& wo,
                  const std::vector<nav_msgs::Odometry>& gps,
                  const std::vector<nav_msgs::Odometry>& vo,
                  const std::vector<nav_msgs::Odometry>& gt,
                  const std::vector<Eigen::Vector3d>& elevation_grid
                  );
  
private:
  
  void performOptimization();
  
  void getOptimizedPoses(const std::vector<nav_msgs::Odometry>& poses, std::vector<nav_msgs::Odometry>& opt_poses);
  void addManifoldAssumptionEdges(const std::string& filename);
  
  void computeUtilezedOdometries( const vector< nav_msgs::Odometry >& wo, const vector< nav_msgs::Odometry >& gps, 
                                  const std::vector<nav_msgs::Odometry>& vo,
                                  const std::vector<nav_msgs::Odometry>& gt,
                                  vector< nav_msgs::Odometry >& used_wo, vector< nav_msgs::Odometry >& used_gps, 
                                  vector< nav_msgs::Odometry >& used_vo,
                                  vector< nav_msgs::Odometry >& used_gt
                                );
  Eigen::Vector3d computeError(const nav_msgs::Odometry& odom, const nav_msgs::Odometry& gt);
  
  int computeSWstart_idx(const vector< nav_msgs::Odometry >& gps, const int current_idx);
  
  Configuration::OptimizerParams params_;
  Configuration::GraphParams graph_params_;
  
  g2o::SparseOptimizer optimizer_;
  
};

#endif