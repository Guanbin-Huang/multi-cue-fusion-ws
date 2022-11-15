#include "oglwidget.h"

#include <QMouseEvent>

#include <math.h>
#include <iostream>

using namespace std;

OGLWidget::OGLWidget(QWidget *parent) :
    QOpenGLWidget(parent),
    geometries(0),
    texture(0),
    angularSpeed(0),
    iter_(0),
    mouse_down_(false),
    mouse_right_down_(false),
    fov_change_(0),
    x_pov_(0),
    y_pov_(0)
{
}

OGLWidget::~OGLWidget()
{
    // Make sure the context is current when deleting the texture
    // and the buffers.
    makeCurrent();
    delete texture;
    delete geometries;
    doneCurrent();
}

void OGLWidget::mousePressEvent(QMouseEvent *e)
{
    if(e->buttons()&Qt::LeftButton)
    {
        mousePressPosition = QVector2D(e->localPos());
        mouse_down_=true;
    }
    if(e->buttons()&Qt::RightButton)
    {
        mouseRightPressPosition = QVector2D(e->localPos());
        mouse_right_down_=true;
    }
}

void OGLWidget::mouseMoveEvent(QMouseEvent *e)
{
    if(!(mouse_down_||mouse_right_down_))
        return;
    if(mouse_down_)
    {
        QVector2D currPos(e->localPos());
        QVector2D diff = currPos - mousePressPosition;

        // Rotation axis is perpendicular to the mouse position difference
        // vector
        QVector3D n = QVector3D(diff.y(), diff.x(), 0.0).normalized();

        // Accelerate angular speed relative to the length of the mouse sweep
        qreal acc = diff.length() / 20.0;

        // Calculate new rotation axis as weighted sum
        rotationAxis = (rotationAxis * angularSpeed + n * acc).normalized();

        // Increase angular speed
        angularSpeed += acc;

        mousePressPosition=currPos;
    }
    if(mouse_right_down_)
    {
        QVector2D currPos(e->localPos());
        QVector2D diff = currPos - mouseRightPressPosition;

        y_pov_ += diff.y() / 100;
        x_pov_ -= diff.x() / 100;

        mouseRightPressPosition=currPos;
    }
    update();
}

void OGLWidget::mouseReleaseEvent(QMouseEvent *)
{
    mouse_down_=false;
    mouse_right_down_=false;
}

void OGLWidget::wheelEvent(QWheelEvent *e)
{

    QVector2D currPos(e->angleDelta() / 8);
    QVector2D diff = currPos;
    fov_change_ += diff.y() / 100;
    //cout<<fov_change_<<endl;
    update();
}

void OGLWidget::timerEvent(QTimerEvent *)
{
    // Decrease angular speed (friction)
    angularSpeed *= 0.8;

    // Stop rotation when speed goes below threshold
    if (angularSpeed < 0.01) {
        angularSpeed = 0.0;
    } else {
        // Update rotation
        rotation = QQuaternion::fromAxisAndAngle(rotationAxis, angularSpeed) * rotation;

        // Request an update
        update();
    }
}

void OGLWidget::initializeGL()
{
    initializeOpenGLFunctions();

//     glClearColor(255, 255, 255, 1);
    glClearColor(0, 0, 0, 1);

    initShaders();
    initTextures();

    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    // Enable back face culling
    glEnable(GL_CULL_FACE);

    geometries = new GeometryEngine;

    // Use QBasicTimer because its faster than QTimer
    timer.start(12, this);
}

void OGLWidget::initShaders()
{
    if(!program.addShaderFromSourceCode(QOpenGLShader::Vertex, VSHADER))
        close();

    if(!program.addShaderFromSourceCode(QOpenGLShader::Fragment, FSHADER))
        close();

    // Link shader pipeline
    if (!program.link())
        close();

    // Bind shader pipeline for use
    if (!program.bind())
        close();
}

void OGLWidget::initTextures()
{
    // Load cube.png image
    texture = new QOpenGLTexture(QImage("/home/marco/catkin_ws/src/flourish_mapping/box.jpg").mirrored());

    // Set nearest filtering mode for texture minification
    texture->setMinificationFilter(QOpenGLTexture::Nearest);

    // Set bilinear filtering mode for texture magnification
    texture->setMagnificationFilter(QOpenGLTexture::Linear);

    // Wrap texture coordinates by repeating
    // f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
    texture->setWrapMode(QOpenGLTexture::Repeat);
}

void OGLWidget::resizeGL(int w, int h)
{
    // Calculate aspect ratio
    qreal aspect = qreal(w) / qreal(h ? h : 1);

    // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
    qreal zNear = .10, zFar = 500.0, fov = 55.0 ;

    // Reset projection
    projection.setToIdentity();

    // Set perspective projection
    projection.perspective(fov, aspect, zNear, zFar);
}

void OGLWidget::paintGL()
{
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    texture->bind();

    // Calculate model view transformation
    QMatrix4x4 matrix;
    matrix.translate(x_pov_, y_pov_, -5.0 + fov_change_);
    matrix.translate(-x_pov_, -y_pov_, 0);
    matrix.rotate(rotation);
    matrix.translate(x_pov_, y_pov_, 0);

    // Set modelview-projection matrix
    program.setUniformValue("mvp_matrix", projection * matrix);

    // Use texture unit 0 which contains cube.png
    program.setUniformValue("texture", 0);

    // Draw cube geometry
    if(geometries->cloud_size_>0)
      geometries->drawPointCloudGeometry(&program);
    if(geometries->traj_size_>0)
      geometries->drawTrajectoryGeometry(&program);
    if(geometries->gt_traj_size_>0)
      geometries->drawGTTrajectoryGeometry(&program);
    if(geometries->gps_traj_size_>0)
      geometries->drawGPSTrajectoryGeometry(&program);
    if(geometries->vo_traj_size_>0)
      geometries->drawVOTrajectoryGeometry(&program);
    if(geometries->node_size_>0)
      geometries->drawNodeGeometry(&program);
    

//     char pixels[4*100*100];
//         std::cout<<"ooooooooo1"<<endl;
// 
//     glReadPixels(0, 0, 100, 100, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
//         std::cout<<"ooooooooo2"<<endl;
// 
//     cv::Mat im(100,100,CV_8UC4, pixels);
//     std::cout<<"ooooooooo3"<<endl;
//     cv::imshow("ciao", im);
//     cv::waitKey(20);
//         std::cout<<"ooooooooo4"<<endl;

}
