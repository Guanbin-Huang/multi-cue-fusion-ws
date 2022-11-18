#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/types/slam3d/vertex_se3.h>

using namespace std;
using namespace g2o;

int main(int argc, char *argv[])
{

    Eigen::Isometry3d pose;
    VertexSE3* v = (VertexSE3 *)malloc(sizeof(VertexSE3));
    // VertexSE3 *v = new VertexSE3;
    v->setEstimate(pose);
    std::cout << "awega" << std::endl;

    return 0;
}



