#ifndef GRAPH_CREATOR_H
#define GRAPH_CREATOR_H

#include "paramsUtils.h"
#include "interpolation_utils.h"
#include "pose_graph/temporalgraph.h"
#include "pose_graph/tss_utils.h"

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

#include <geodetic_utils/geodetic_conv.hpp>
#include "elevation_utils.h"

class GraphCreator
{
public:
    GraphCreator(){}
    ~GraphCreator(){}

    void setParams(const Configuration::GraphParams& params){params_=params;}
    void create(const std::vector<nav_msgs::Odometry>& wo,
                const std::vector<nav_msgs::Odometry>& gps,
                const std::vector<nav_msgs::Odometry>& vo,
                const std::vector<nav_msgs::Odometry>& gt,
                const std::vector<Eigen::Vector3d>& elevation_grid,
                const std::vector<nav_msgs::Odometry>& imu,
                const std::vector<sensor_msgs::PointCloud2>& clouds,
                std::vector<int>& temporal_WO_indices
                );
    void create(const vector< nav_msgs::Odometry >& sw,
                const std::vector<nav_msgs::Odometry>& wo,
                const std::vector<nav_msgs::Odometry>& gps,
                const std::vector<nav_msgs::Odometry>& vo,
                const std::vector<nav_msgs::Odometry>& gt,
                const std::vector<Eigen::Vector3d>& elevation_grid,
                const int& start_idx
                );
    void createFromInterpolation();
    
    static void writeGraph(const std::string name, 
                           const std::vector< g2o::VertexSE3* >& verteces, 
                           const std::vector< g2o::VertexSE3* >& vo_verteces, 
                           const std::vector< g2o::VertexSE3* >& svo_verteces,
                           const std::vector< g2o::EdgeSE3* >& edges, 
                           const std::vector< g2o::EdgeSE3* >& vo_edges, 
                           const std::vector< g2o::EdgeSE3* >& svo_edges,
                           const std::vector< g2o::EdgeSE3Prior* >& prior_edges);
    
    static Eigen::Isometry3d getPosesDiff(const Eigen::Isometry3d& pos1,
                                          const Eigen::Isometry3d& pos2);
private:
  
    
    
    Configuration::GraphParams params_;

};

#endif // GRAPH_CREATOR_H

