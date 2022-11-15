#ifndef OGLWIDGET_H
#define OGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

#include "geometryengine.h"

/*#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/PolygonMesh.h"
#include "pcl/pcl_macros.h"*/
//#include "pcl/ros/conversions.h"
#include "pcl/io/pcd_io.h"

// #include <opencv2/opencv.hpp>


#define VSHADER \
    "\
    uniform mat4 mvp_matrix;\
    attribute vec4 a_position;\
    attribute vec2 a_texcoord;\
    varying vec2 v_texcoord;\
    attribute vec3 vertex_color; \
    varying vec3 fragment_color; \
    void main()\
    {\
        gl_Position = mvp_matrix * a_position;\
        v_texcoord = a_texcoord;\
        fragment_color = vertex_color; \
    }\
    "

#define FSHADER "\
    uniform sampler2D texture;\
    varying vec2 v_texcoord;\
    varying vec3 fragment_color; \
    void main()\
    {\
        gl_FragColor = vec4(fragment_color, 1);\
        /*gl_FragColor = texture2D(texture, v_texcoord);*/\
    }"


class GeometryEngine;

class OGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    OGLWidget(QWidget *parent = 0);
    ~OGLWidget();

    void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        geometries->initPointCloudGeometry(cloud);
        update();
    }
    void setPointCloudNormal(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud)
    {
        geometries->initPointCloudNormalGeometry(cloud);
        update();
    }
    
    void setTrajectoryPoints(std::vector<Eigen::Isometry3d>& poses, const Eigen::Vector3d& color)
    {
        geometries->initTrajectoryGeometry(poses, color);
//         update();
    }
    
    void setGTTrajectoryPoints(std::vector<Eigen::Isometry3d>& poses, const Eigen::Vector3d& color)
    {
        geometries->initGTTrajectoryGeometry(poses, color);
    }
    
    void setGPSTrajectoryPoints(std::vector<Eigen::Isometry3d>& poses, const Eigen::Vector3d& color)
    {
        geometries->initGPSTrajectoryGeometry(poses, color);
    }
    
    void setVOTrajectoryPoints(std::vector<Eigen::Isometry3d>& poses, const Eigen::Vector3d& color)
    {
        geometries->initVOTrajectoryGeometry(poses, color);
    }
    
    void setNodePoints(const Eigen::Vector3d& center, const double& r, const Eigen::Vector3d& color)
    {
        geometries->initNodeGeometry(center, r, color);
    }

protected:
    void mousePressEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    void timerEvent(QTimerEvent *e) override;

    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void initShaders();
    void initTextures();

private:
    QBasicTimer timer;
    QOpenGLShaderProgram program;
    GeometryEngine *geometries;

    QOpenGLTexture *texture;

    QMatrix4x4 projection;

    QVector2D mousePressPosition, mouseRightPressPosition;
    QVector3D rotationAxis;
    qreal angularSpeed;
    QQuaternion rotation;

    int iter_;
    bool mouse_down_, mouse_right_down_;
    qreal fov_change_, x_pov_, y_pov_;
};

#endif // OGLWIDGET_H

