#include "geometryengine.h"

#include <QVector2D>
#include <QVector3D>

#include <iostream>

using namespace std;

struct VertexData
{
    QVector3D position;
    QVector3D color;
    QVector2D texCoord;
};

GeometryEngine::GeometryEngine()
    : indexBuf(QOpenGLBuffer::IndexBuffer),
      traj_indexBuf(QOpenGLBuffer::IndexBuffer),
      gt_traj_indexBuf(QOpenGLBuffer::IndexBuffer),
      gps_traj_indexBuf(QOpenGLBuffer::IndexBuffer),
      vo_traj_indexBuf(QOpenGLBuffer::IndexBuffer),
      node_indexBuf(QOpenGLBuffer::IndexBuffer)
{
    initializeOpenGLFunctions();

    // Generate 2 VBOs
    arrayBuf.create();
    indexBuf.create();
    
    traj_arrayBuf.create();
    traj_indexBuf.create();
    
    gt_traj_arrayBuf.create();
    gt_traj_indexBuf.create();
    
    gps_traj_arrayBuf.create();
    gps_traj_indexBuf.create();
    
    vo_traj_arrayBuf.create();
    vo_traj_indexBuf.create();
    
    node_arrayBuf.create();
    node_indexBuf.create();

    cloud_size_=0;
    gt_traj_size_=0;
    traj_size_=0;
    gps_traj_size_=0;
    vo_traj_size_=0;
    node_size_=0;
}

GeometryEngine::~GeometryEngine()
{
    arrayBuf.destroy();
    indexBuf.destroy();
    traj_arrayBuf.destroy();
    traj_indexBuf.destroy();
    
    gt_traj_arrayBuf.destroy();
    gt_traj_indexBuf.destroy();
    
    gps_traj_arrayBuf.destroy();
    gps_traj_indexBuf.destroy();
    
    vo_traj_arrayBuf.destroy();
    vo_traj_indexBuf.destroy();
    
    node_arrayBuf.destroy();
    node_indexBuf.destroy();
}

void GeometryEngine::initPointCloudGeometry(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    cloud_size_=cloud->points.size();
    
    if(cloud_size_==0)
      return;
    
    QVector3D dummy_color(0,1,0);
    QVector2D dummy_tex(0,0);
//     VertexData vertices[cloud_size_];
    VertexData* vertices;
//             std::cout<<"allocating "<<cloud_size_*sizeof(VertexData)<<" bytes"<<endl;

    vertices = (VertexData*)calloc(cloud_size_, sizeof(VertexData));
//             std::cout<<"allocated "<<cloud_size_*sizeof(VertexData)<<" bytes"<<endl;

    GLuint* indices;
    indices = (GLuint*)calloc(cloud_size_, sizeof(GLuint));
    for(int i=0; i<cloud_size_; i++)
    {
      double z=cloud->points[i].z;
//       if(z<-2.5)
//       {
//         continue;
//       }
      double alpha=0, beta=0, gamma=1;
        QVector3D p(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        if(z<2.5&&z>0)
        {
          alpha=z/2.5;
          gamma=1-alpha;
        }
        else if(z>=2.5&&z<10)
        {
          gamma=0;
          beta=z/10;
          alpha=1-beta;
        }
        else if(z>=10)
        {
          gamma=z/10 - 1;
          (gamma>1)? gamma=1:gamma;
          beta=1;
          alpha=0;
        }
        dummy_color.setX(beta/1.1);
        dummy_color.setY(gamma/1.3);
        dummy_color.setZ(alpha/1.3);
        vertices[i]={p,dummy_color,dummy_tex};
        indices[i]=(GLuint)i;
    }

    
    // Transfer vertex data to VBO 0
    arrayBuf.bind();
    arrayBuf.allocate(vertices, cloud_size_ * sizeof(VertexData));
    
//     std::cout<<"allocated "<<endl;

    // Transfer index data to VBO 1
    indexBuf.bind();
    indexBuf.allocate(indices, cloud_size_ * sizeof(GLuint));
//         std::cout<<"allocated index "<<endl;

}

void GeometryEngine::initPointCloudNormalGeometry(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud)
{
    cloud_size_=cloud->points.size();
    
    if(cloud_size_==0)
      return;
    
    QVector3D dummy_color(0,1,0);
    QVector2D dummy_tex(0,0);
//     VertexData vertices[cloud_size_];
    VertexData* vertices;
//             std::cout<<"allocating "<<cloud_size_*sizeof(VertexData)<<" bytes"<<endl;

    vertices = (VertexData*)calloc(cloud_size_, sizeof(VertexData));
//             std::cout<<"allocated "<<cloud_size_*sizeof(VertexData)<<" bytes"<<endl;

    GLuint* indices;
    indices = (GLuint*)calloc(cloud_size_, sizeof(GLuint));
    for(int i=0; i<cloud_size_; i++)
    {
      double z=cloud->points[i].z;
      double alpha=0, beta=0, gamma=1;
      QVector3D p(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
       
        dummy_color.setX((double)fabs(cloud->points[i].normal_x));
        dummy_color.setY((double)fabs(cloud->points[i].normal_z));
        dummy_color.setZ((double)fabs(cloud->points[i].normal_y));
        vertices[i]={p,dummy_color,dummy_tex};
        indices[i]=(GLuint)i;
    }

    
    // Transfer vertex data to VBO 0
    arrayBuf.bind();
    arrayBuf.allocate(vertices, cloud_size_ * sizeof(VertexData));
    
//     std::cout<<"allocated "<<endl;

    // Transfer index data to VBO 1
    indexBuf.bind();
    indexBuf.allocate(indices, cloud_size_ * sizeof(GLuint));
//         std::cout<<"allocated index "<<endl;

}

void GeometryEngine::initTrajectoryGeometry(vector< Eigen::Isometry3d >& poses, const Eigen::Vector3d& color)
{
  traj_size_=poses.size();
  
  if(traj_size_==0)
    return;
  
  QVector3D dummy_color(color(0), color(1), color(2));
  QVector2D dummy_tex(0,0);
  VertexData vertices[traj_size_];
  GLushort indices[traj_size_];
  for(int i=0; i<traj_size_; i++)
  {
      QVector3D p(poses[i].translation()(0), poses[i].translation()(1), poses[i].translation()(2));
      vertices[i]={p,dummy_color,dummy_tex};
      indices[i]=(GLushort)i;
  }

  // Transfer vertex data to VBO 0
  traj_arrayBuf.bind();
  traj_arrayBuf.allocate(vertices, traj_size_ * sizeof(VertexData));

  // Transfer index data to VBO 1
  traj_indexBuf.bind();
  traj_indexBuf.allocate(indices, traj_size_ * sizeof(GLushort));
}

void GeometryEngine::initGTTrajectoryGeometry(vector< Eigen::Isometry3d >& poses, const Eigen::Vector3d& color)
{
  gt_traj_size_=poses.size();
  
  if(gt_traj_size_==0)
    return;
  
  QVector3D dummy_color((float)color(0), (float)color(1), (float)color(2));
  QVector2D dummy_tex(0,0);
  VertexData vertices[gt_traj_size_];
  GLushort indices[gt_traj_size_];
  for(int i=0; i<gt_traj_size_; i++)
  {
      QVector3D p(poses[i].translation()(0), poses[i].translation()(1), poses[i].translation()(2));
      vertices[i]={p,dummy_color,dummy_tex};
      indices[i]=(GLushort)i;
  }

  // Transfer vertex data to VBO 0
  gt_traj_arrayBuf.bind();
  gt_traj_arrayBuf.allocate(vertices, gt_traj_size_ * sizeof(VertexData));

  // Transfer index data to VBO 1
  gt_traj_indexBuf.bind();
  gt_traj_indexBuf.allocate(indices, gt_traj_size_ * sizeof(GLushort));
}

void GeometryEngine::initGPSTrajectoryGeometry(vector< Eigen::Isometry3d >& poses, const Eigen::Vector3d& color)
{
  gps_traj_size_=poses.size();
  
  if(gps_traj_size_==0)
    return;
  
  QVector3D dummy_color((float)color(0), (float)color(1), (float)color(2));
  QVector2D dummy_tex(0,0);
  VertexData vertices[gps_traj_size_];
  GLushort indices[gps_traj_size_];
  for(int i=0; i<gps_traj_size_; i++)
  {
      QVector3D p(poses[i].translation()(0), poses[i].translation()(1), poses[i].translation()(2));
      vertices[i]={p,dummy_color,dummy_tex};
      indices[i]=(GLushort)i;
  }

  // Transfer vertex data to VBO 0
  gps_traj_arrayBuf.bind();
  gps_traj_arrayBuf.allocate(vertices, gps_traj_size_ * sizeof(VertexData));

  // Transfer index data to VBO 1
  gps_traj_indexBuf.bind();
  gps_traj_indexBuf.allocate(indices, gps_traj_size_ * sizeof(GLushort));
}

void GeometryEngine::initVOTrajectoryGeometry(vector< Eigen::Isometry3d >& poses, const Eigen::Vector3d& color)
{
  vo_traj_size_=poses.size();
  
  if(vo_traj_size_==0)
    return;
  
  QVector3D dummy_color((float)color(0), (float)color(1), (float)color(2));
  QVector2D dummy_tex(0,0);
  VertexData vertices[vo_traj_size_];
  GLushort indices[vo_traj_size_];
  for(int i=0; i<vo_traj_size_; i++)
  {
      QVector3D p(poses[i].translation()(0), poses[i].translation()(1), poses[i].translation()(2));
      vertices[i]={p,dummy_color,dummy_tex};
      indices[i]=(GLushort)i;
  }

  // Transfer vertex data to VBO 0
  vo_traj_arrayBuf.bind();
  vo_traj_arrayBuf.allocate(vertices, vo_traj_size_ * sizeof(VertexData));

  // Transfer index data to VBO 1
  vo_traj_indexBuf.bind();
  vo_traj_indexBuf.allocate(indices, vo_traj_size_ * sizeof(GLushort));
}

void GeometryEngine::initNodeGeometry(const Eigen::Vector3d& center, const double& r, const Eigen::Vector3d& color)
{
  node_size_=24;
  QVector3D dummy_color((float)color(0), (float)color(1), (float)color(2));
  QVector2D dummy_tex(0,0);
  VertexData vertices[node_size_];
  GLushort indices[node_size_];
  QVector3D p;
  
  for(int i=0; i<node_size_; i++)
    indices[i]=(GLushort)i;
  
  p=QVector3D(center(0)+r/2, center(1)+r/2, center(2)+r/2); vertices[0]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)-r/2, center(1)+r/2, center(2)+r/2); vertices[1]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)-r/2, center(1)-r/2, center(2)+r/2); vertices[2]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)+r/2, center(1)-r/2, center(2)+r/2); vertices[3]={p, dummy_color, dummy_tex}; 
  
  p=QVector3D(center(0)+r/2, center(1)+r/2, center(2)-r/2); vertices[4]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)-r/2, center(1)+r/2, center(2)-r/2); vertices[5]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)-r/2, center(1)-r/2, center(2)-r/2); vertices[6]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)+r/2, center(1)-r/2, center(2)-r/2); vertices[7]={p, dummy_color, dummy_tex}; 
  
  
  p=QVector3D(center(0)+r/2, center(1)+r/2, center(2)+r/2); vertices[8]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)+r/2, center(1)-r/2, center(2)+r/2); vertices[9]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)+r/2, center(1)-r/2, center(2)-r/2); vertices[10]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)+r/2, center(1)+r/2, center(2)-r/2); vertices[11]={p, dummy_color, dummy_tex}; 
  
  p=QVector3D(center(0)-r/2, center(1)+r/2, center(2)+r/2); vertices[12]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)-r/2, center(1)-r/2, center(2)+r/2); vertices[13]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)-r/2, center(1)-r/2, center(2)-r/2); vertices[14]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)-r/2, center(1)+r/2, center(2)-r/2); vertices[15]={p, dummy_color, dummy_tex};
  
  
  p=QVector3D(center(0)+r/2, center(1)+r/2, center(2)+r/2); vertices[16]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)-r/2, center(1)+r/2, center(2)+r/2); vertices[17]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)-r/2, center(1)+r/2, center(2)-r/2); vertices[18]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)+r/2, center(1)+r/2, center(2)-r/2); vertices[19]={p, dummy_color, dummy_tex}; 
  
  p=QVector3D(center(0)+r/2, center(1)-r/2, center(2)+r/2); vertices[20]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)-r/2, center(1)-r/2, center(2)+r/2); vertices[21]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)-r/2, center(1)-r/2, center(2)-r/2); vertices[22]={p, dummy_color, dummy_tex};
  p=QVector3D(center(0)+r/2, center(1)-r/2, center(2)-r/2); vertices[23]={p, dummy_color, dummy_tex};
  
  node_arrayBuf.bind();
  node_arrayBuf.allocate(vertices, node_size_ * sizeof(VertexData));

  node_indexBuf.bind();
  node_indexBuf.allocate(indices, node_size_ * sizeof(GLushort));
}

void GeometryEngine::drawPointCloudGeometry(QOpenGLShaderProgram *program)
{
    // Tell OpenGL which VBOs to use
    arrayBuf.bind();
    indexBuf.bind();

    // Offset for position
    quintptr offset = 0;

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    int colorLocation = program->attributeLocation("vertex_color");
    program->enableAttributeArray(colorLocation);
    program->setAttributeBuffer(colorLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    offset+=sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int texcoordLocation = program->attributeLocation("a_texcoord");
    program->enableAttributeArray(texcoordLocation);
    program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

    // Draw cube geometry using indices from VBO 1
    glDrawElements(GL_POINTS, cloud_size_, GL_UNSIGNED_INT, 0);
    
}

void GeometryEngine::drawTrajectoryGeometry(QOpenGLShaderProgram* program)
{
    traj_arrayBuf.bind();
    traj_indexBuf.bind();

    // Offset for position
    quintptr offset = 0;

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    int colorLocation = program->attributeLocation("vertex_color");
    program->enableAttributeArray(colorLocation);
    program->setAttributeBuffer(colorLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    offset+=sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int texcoordLocation = program->attributeLocation("a_texcoord");
    program->enableAttributeArray(texcoordLocation);
    program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

    glDrawElements(GL_LINE_STRIP, traj_size_, GL_UNSIGNED_SHORT, 0);
}

void GeometryEngine::drawGTTrajectoryGeometry(QOpenGLShaderProgram* program)
{
  gt_traj_arrayBuf.bind();
  gt_traj_indexBuf.bind();

  // Offset for position
  quintptr offset = 0;

  // Tell OpenGL programmable pipeline how to locate vertex position data
  int vertexLocation = program->attributeLocation("a_position");
  program->enableAttributeArray(vertexLocation);
  program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

  // Offset for texture coordinate
  offset += sizeof(QVector3D);

  int colorLocation = program->attributeLocation("vertex_color");
  program->enableAttributeArray(colorLocation);
  program->setAttributeBuffer(colorLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

  offset+=sizeof(QVector3D);

  // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
  int texcoordLocation = program->attributeLocation("a_texcoord");
  program->enableAttributeArray(texcoordLocation);
  program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

  glDrawElements(GL_LINE_STRIP, gt_traj_size_, GL_UNSIGNED_SHORT, 0);
}

void GeometryEngine::drawGPSTrajectoryGeometry(QOpenGLShaderProgram* program)
{
  gps_traj_arrayBuf.bind();
  gps_traj_indexBuf.bind();

  // Offset for position
  quintptr offset = 0;

  // Tell OpenGL programmable pipeline how to locate vertex position data
  int vertexLocation = program->attributeLocation("a_position");
  program->enableAttributeArray(vertexLocation);
  program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

  // Offset for texture coordinate
  offset += sizeof(QVector3D);

  int colorLocation = program->attributeLocation("vertex_color");
  program->enableAttributeArray(colorLocation);
  program->setAttributeBuffer(colorLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

  offset+=sizeof(QVector3D);

  // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
  int texcoordLocation = program->attributeLocation("a_texcoord");
  program->enableAttributeArray(texcoordLocation);
  program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

  glDrawElements(GL_LINE_STRIP, gps_traj_size_, GL_UNSIGNED_SHORT, 0);
}

void GeometryEngine::drawVOTrajectoryGeometry(QOpenGLShaderProgram* program)
{
  vo_traj_arrayBuf.bind();
  vo_traj_indexBuf.bind();

  // Offset for position
  quintptr offset = 0;

  // Tell OpenGL programmable pipeline how to locate vertex position data
  int vertexLocation = program->attributeLocation("a_position");
  program->enableAttributeArray(vertexLocation);
  program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

  // Offset for texture coordinate
  offset += sizeof(QVector3D);

  int colorLocation = program->attributeLocation("vertex_color");
  program->enableAttributeArray(colorLocation);
  program->setAttributeBuffer(colorLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

  offset+=sizeof(QVector3D);

  // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
  int texcoordLocation = program->attributeLocation("a_texcoord");
  program->enableAttributeArray(texcoordLocation);
  program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

  glDrawElements(GL_LINE_STRIP, vo_traj_size_, GL_UNSIGNED_SHORT, 0);
}

void GeometryEngine::drawNodeGeometry(QOpenGLShaderProgram* program)
{
  node_arrayBuf.bind();
  node_indexBuf.bind();

  // Offset for position
  quintptr offset = 0;

  // Tell OpenGL programmable pipeline how to locate vertex position data
  int vertexLocation = program->attributeLocation("a_position");
  program->enableAttributeArray(vertexLocation);
  program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

  // Offset for texture coordinate
  offset += sizeof(QVector3D);

  int colorLocation = program->attributeLocation("vertex_color");
  program->enableAttributeArray(colorLocation);
  program->setAttributeBuffer(colorLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

  offset+=sizeof(QVector3D);

  // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
  int texcoordLocation = program->attributeLocation("a_texcoord");
  program->enableAttributeArray(texcoordLocation);
  program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

  glDrawElements(GL_QUADS, node_size_, GL_UNSIGNED_SHORT, 0);
}