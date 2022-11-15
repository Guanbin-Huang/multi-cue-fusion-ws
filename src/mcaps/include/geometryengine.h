#ifndef GEOMETRYENGINE_H
#define GEOMETRYENGINE_H

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>

/*#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PolygonMesh.h"
#include "pcl/pcl_macros.h"*/
//#include "pcl/ros/conversions.h"
#include "pcl/io/pcd_io.h"

#include <eigen3/Eigen/Dense>

class GeometryEngine : protected QOpenGLFunctions
{
public:
    GeometryEngine();
    virtual ~GeometryEngine();

    void drawPointCloudGeometry(QOpenGLShaderProgram *program);
    void drawTrajectoryGeometry(QOpenGLShaderProgram *program);
    void drawGTTrajectoryGeometry(QOpenGLShaderProgram *program);
    void drawGPSTrajectoryGeometry(QOpenGLShaderProgram *program);
    void drawVOTrajectoryGeometry(QOpenGLShaderProgram *program);
    void drawNodeGeometry(QOpenGLShaderProgram *program);

    void initPointCloudGeometry(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void initPointCloudNormalGeometry(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);
    void initTrajectoryGeometry(std::vector<Eigen::Isometry3d>& poses, const Eigen::Vector3d& color);
    void initGTTrajectoryGeometry(std::vector<Eigen::Isometry3d>& poses, const Eigen::Vector3d& color);
    void initGPSTrajectoryGeometry(std::vector<Eigen::Isometry3d>& poses, const Eigen::Vector3d& color);
    void initVOTrajectoryGeometry(std::vector<Eigen::Isometry3d>& poses, const Eigen::Vector3d& color);
    void initNodeGeometry(const Eigen::Vector3d& center, const double& r, const Eigen::Vector3d& color);
    
    int cloud_size_, traj_size_, gt_traj_size_, gps_traj_size_, vo_traj_size_, node_size_;
private:

    QOpenGLBuffer arrayBuf;
    QOpenGLBuffer indexBuf;
    
    QOpenGLBuffer traj_arrayBuf;
    QOpenGLBuffer traj_indexBuf;
    
    QOpenGLBuffer gt_traj_arrayBuf;
    QOpenGLBuffer gt_traj_indexBuf;
    
    QOpenGLBuffer gps_traj_arrayBuf;
    QOpenGLBuffer gps_traj_indexBuf;
    
    QOpenGLBuffer vo_traj_arrayBuf;
    QOpenGLBuffer vo_traj_indexBuf;

    QOpenGLBuffer node_arrayBuf;
    QOpenGLBuffer node_indexBuf;

};

#endif // GEOMETRYENGINE_H

