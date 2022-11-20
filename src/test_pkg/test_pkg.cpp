#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

typedef g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType> SlamLinearSolver;

int main(int argc, char **argv)
{
 

    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(0);
    // v->setEstimate(g2o::Isometry3::Identity());
    v->setFixed(false);
   

    return 0;
}