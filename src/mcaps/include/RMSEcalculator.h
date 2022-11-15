#ifndef RMSE_CALCULATOR_H
#define RMSE_CALCULATOR_H

#include "interpolation_utils.h"
#include "paramsUtils.h"
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

class RMSEcalculator
{
public:
    RMSEcalculator(const std::string& out_folder);
    ~RMSEcalculator(){}

//     void getGT(std::vector<Eigen::Isometry3d>& gt){gt=gt_traj_;};
//     void getPoses(std::vector<Eigen::Isometry3d>& poses){poses=opt_pose_traj_;};
    
    double compute(const std::vector<nav_msgs::Odometry>& poses,
                 const std::vector<nav_msgs::Odometry>& gt_poses,
                 std::vector<nav_msgs::Odometry>& used_poses,
                std::vector<nav_msgs::Odometry>& used_gt_poses
                );
    void computeFromInterpolation();
private:

    std::string out_folder_;

};

#endif // RMSE_CALCULATOR_H
