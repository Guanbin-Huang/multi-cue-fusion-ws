#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <QtGui>
#include <QMessageBox>
#include <QFileDialog>
#include <QTime>

//#include "pcl/ros/conversions.h"
#include <pcl/io/pcd_io.h>

#include "graph_creator.h"
#include "g2o_optimizer.h"
#include "RMSEcalculator.h"
#include "data_extractor.h"
#include "g2o_utils.h"
#include "stitching_utils.h"
// #include "nicp_utils.h"

#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  sw_optimizer_=new G2OOptimizer;

  ui->setupUi(this);
      

  QObject::connect(ui->CreateGraphButton, SIGNAL(clicked (bool)), this, SLOT(createGraph()));
  QObject::connect(ui->OptimizeButton, SIGNAL(clicked (bool)), this, SLOT(optimizeGraph()));
  QObject::connect(ui->SWOptimizeButton, SIGNAL(clicked (bool)), this, SLOT(sw_optimizeGraph()));
  QObject::connect(ui->OptimizeManifoldButton, SIGNAL(clicked (bool)), this, SLOT(optimizeGraphManifold()));
  QObject::connect(ui->saveGraphParamsButton, SIGNAL(clicked (bool)), this, SLOT(saveGraphParams()));
  QObject::connect(ui->saveOptimizationParamsButton, SIGNAL(clicked (bool)), this, SLOT(saveOptimizationParams()));
  QObject::connect(ui->loadGraphParamsButton, SIGNAL(clicked (bool)), this, SLOT(loadGraphParams()));
  QObject::connect(ui->loadOptimizationParamsButton, SIGNAL(clicked (bool)), this, SLOT(loadOptimizationParams()));
//   QObject::connect(ui->stats_button, SIGNAL(clicked (bool)), this, SLOT(computeStatistics()));
  
  QObject::connect(ui->topicGTlineEdit, SIGNAL(textChanged(QString)), this, SLOT(setUpdateRequest()));
  QObject::connect(ui->topicWOlineEdit, SIGNAL(textChanged(QString)), this, SLOT(setUpdateRequest()));
  QObject::connect(ui->topicGPSlineEdit, SIGNAL(textChanged(QString)), this, SLOT(setUpdateRequest()));
  QObject::connect(ui->topicSVOlineEdit, SIGNAL(textChanged(QString)), this, SLOT(setUpdateRequest()));
  QObject::connect(ui->bag_filenamelineEdit, SIGNAL(textChanged(QString)), this, SLOT(setUpdateRequest()));
  QObject::connect(ui->TSS_filenamelineEdit, SIGNAL(textChanged(QString)), this, SLOT(setUpdateRequest()));
  QObject::connect(ui->TSSmain_dirlineEdit, SIGNAL(textChanged(QString)), this, SLOT(setUpdateRequest()));
  
//     QObject::connect(ui->WOcheckBox, SIGNAL(stateChanged(int)), this, SLOT(setUpdateRequest()));
//     QObject::connect(ui->GPScheckBox, SIGNAL(stateChanged(int)), this, SLOT(setUpdateRequest()));
//     QObject::connect(ui->SVOcheckBox, SIGNAL(stateChanged(int)), this, SLOT(setUpdateRequest()));
//     QObject::connect(ui->InterpolationcheckBox, SIGNAL(stateChanged(int)), this, SLOT(setUpdateRequest()));
  

  QObject::connect(ui->visPointCloudcheckBox, SIGNAL(clicked (bool)), this, SLOT(visPointCloud()));
  QObject::connect(ui->visWOcheckBox, SIGNAL(clicked (bool)), this, SLOT(visWO_trajectory()));
  QObject::connect(ui->original_radioButton, SIGNAL(clicked (bool)), this, SLOT(visWO_trajectory()));
  QObject::connect(ui->optimized_radioButton, SIGNAL(clicked (bool)), this, SLOT(visWO_trajectory()));
  QObject::connect(ui->visGTcheckBox, SIGNAL(clicked (bool)), this, SLOT(visGT_trajectory()));
  QObject::connect(ui->visGPScheckBox, SIGNAL(clicked (bool)), this, SLOT(visGPS_trajectory()));
  QObject::connect(ui->visVOcheckBox, SIGNAL(clicked (bool)), this, SLOT(visVO_trajectory()));
  
  QObject::connect(ui->nodesSlider, SIGNAL(valueChanged (int)), this, SLOT(nodeVisualization()));
  QObject::connect(ui->nodesspinBox, SIGNAL(valueChanged (int)), this, SLOT(nodesspinBox_slot()));
  
  update_requested_=true;
  
  QThread::currentThread()->setObjectName("main");
  
  thread_.setObjectName("thread");
  sw_optimizer_->moveToThread(&thread_);
  thread_.start();
  
  QObject::connect(this->sw_optimizer_, SIGNAL(showPosesSignal(std::vector<nav_msgs::Odometry> )),
                    this, SLOT(showPosesSlot(std::vector<nav_msgs::Odometry> )));
  
  QObject::connect(this, SIGNAL(sw_optimizeSignal(const std::vector<nav_msgs::Odometry> ,
                                                  const std::vector<nav_msgs::Odometry> ,
                                                  const std::vector<nav_msgs::Odometry> ,
                                                  const std::vector<nav_msgs::Odometry> ,
                                                  const std::vector<Eigen::Vector3d>)),
                    this->sw_optimizer_, SLOT(SWoptimize(const std::vector<nav_msgs::Odometry> ,
                                                        const std::vector<nav_msgs::Odometry> ,
                                                        const std::vector<nav_msgs::Odometry> ,
                                                        const std::vector<nav_msgs::Odometry> ,
                                                        const std::vector<Eigen::Vector3d> )));
  
}

MainWindow::~MainWindow()
{
  thread_.wait();
    delete ui;
}

void MainWindow::setParams()
{
  graph_params_.tss_maindir=ui->TSSmain_dirlineEdit->text().toStdString();
  graph_params_.tss_filename=ui->TSS_filenamelineEdit->text().toStdString();
  graph_params_.bag_filename=ui->bag_filenamelineEdit->text().toStdString();
  graph_params_.WO_topic=ui->topicWOlineEdit->text().toStdString();
  graph_params_.GPS_topic=ui->topicGPSlineEdit->text().toStdString();
//     graph_params_.VO_topic=ui->topicVOlineEdit->text().toStdString();
  graph_params_.SVO_topic=ui->topicSVOlineEdit->text().toStdString();
  graph_params_.GT_topic=ui->topicGTlineEdit->text().toStdString();
  graph_params_.JAI_RGB_topic=ui->topicJAI_RGBlineEdit->text().toStdString();
  graph_params_.IMU_topic=ui->topicIMUlineEdit->text().toStdString();
  graph_params_.CLOUD_topic=ui->topicCLOUDlineEdit->text().toStdString();
  
//     graph_params_.GPS_calib=ui->GPS_caliblineEdit->text().toStdString();
//     graph_params_.VO_calib=ui->VO_caliblineEdit->text().toStdString();
//     graph_params_.SVO_calib=ui->SVO_caliblineEdit->text().toStdString();
//     graph_params_.GT_calib=ui->GT_caliblineEdit->text().toStdString();

  graph_params_.enable[0]=ui->WOcheckBox->isChecked();
  graph_params_.enable[1]=ui->GPScheckBox->isChecked();
  graph_params_.enable[2]=ui->MonoVocheckBox->isChecked();
  graph_params_.enable[3]=ui->SVOcheckBox->isChecked();
  
  graph_params_.enable_MA=ui->MAcheckBox->isChecked();
  graph_params_.enable_elevation=ui->elevationcheckBox->isChecked();
  graph_params_.enable_ackerman=ui->ackermancheckBox->isChecked();
  graph_params_.enable_imu=ui->imucheckBox->isChecked();
  graph_params_.enable_velodyne=ui->velodynecheckBox->isChecked();

  graph_params_.WO_step=ui->WOstepSpinBox->value();
  graph_params_.GPS_step=ui->GPSstepSpinBox->value();
//     graph_params_.VO_step=ui->MonoVOstepSpinBox->value();
  graph_params_.SVO_step=ui->StereoVOstepSpinBox->value();
  graph_params_.CLOUD_step=ui->CLOUDstepSpinBox->value();

  graph_params_.WO_scale_t=ui->WOscaletSpinBox->value();
  graph_params_.WO_scale_r=ui->WOscalerSpinBox->value();
  graph_params_.GPS_scale_txy=ui->GPSscaletxySpinBox->value();
  graph_params_.GPS_scale_tz=ui->GPSscaletzSpinBox->value();
//     graph_params_.VO_scale_t=ui->VOscaletSpinBox->value();
//     graph_params_.VO_scale_r=ui->VOscalerSpinBox->value();
  graph_params_.SVO_scale_t=ui->SVOscaletSpinBox->value();
  graph_params_.SVO_scale_tz=ui->SVOscaletzSpinBox->value();
  graph_params_.SVO_scale_r=ui->SVOscalerSpinBox->value();
  
  graph_params_.ACKERMAN_scale_t=ui->ackscaletSpinBox->value();
  graph_params_.ACKERMAN_scale_r=ui->ackscalerSpinBox->value();
  
  graph_params_.IMU_scale=ui->IMUscaleSpinBox->value();
  graph_params_.CLOUD_scale=ui->CLOUDscaleSpinBox->value();
  
  graph_params_.MA_radius=ui->MAradiusSpinBox->value();
  graph_params_.MA_scale=ui->MAscaleSpinBox->value();
  
  graph_params_.sw_size=(int)ui->swsizeSpinBox->value();
  graph_params_.sw_scale=ui->swscaleSpinBox->value();
  
  graph_params_.elevation_scale=ui->elevationscaleSpinBox->value();

  graph_params_.graph_filename=ui->graphFileLineEdit->text().toStdString();
  
  graph_params_.elevation_map_file=ui->elevationmapFileLineEdit->text().toStdString();
  
  graph_params_.out_folder=ui->outFolder_LineEdit->text().toStdString();

  optimizer_params_.N_iterations=ui->IterationsSpinBox->value();
  optimizer_params_.in_filename=ui->graphFileLineEdit->text().toStdString();
  optimizer_params_.out_filename=ui->outFileLineEdit->text().toStdString();
}

void MainWindow::updateGUIGraphParams()
{
  ui->TSSmain_dirlineEdit->setText(QString(graph_params_.tss_maindir.c_str()));
  ui->TSS_filenamelineEdit->setText(QString(graph_params_.tss_filename.c_str()));
  ui->bag_filenamelineEdit->setText(QString(graph_params_.bag_filename.c_str()));
  ui->topicWOlineEdit->setText(QString(graph_params_.WO_topic.c_str()));
  ui->topicGPSlineEdit->setText(QString(graph_params_.GPS_topic.c_str()));
//     ui->topicVOlineEdit->setText(QString(graph_params_.VO_topic.c_str()));
  ui->topicSVOlineEdit->setText(QString(graph_params_.SVO_topic.c_str()));
  ui->topicGTlineEdit->setText(QString(graph_params_.GT_topic.c_str()));
  ui->topicJAI_RGBlineEdit->setText(QString(graph_params_.JAI_RGB_topic.c_str()));
  ui->topicIMUlineEdit->setText(QString(graph_params_.IMU_topic.c_str()));
  ui->topicCLOUDlineEdit->setText(QString(graph_params_.CLOUD_topic.c_str()));
  
//     ui->GPS_caliblineEdit->setText(QString(graph_params_.GPS_calib.c_str()));
//     ui->VO_caliblineEdit->setText(QString(graph_params_.VO_calib.c_str()));
//     ui->SVO_caliblineEdit->setText(QString(graph_params_.SVO_calib.c_str()));
//     ui->GT_caliblineEdit->setText(QString(graph_params_.GT_calib.c_str()));

  ui->WOcheckBox->setChecked(graph_params_.enable[0]);
  ui->GPScheckBox->setChecked(graph_params_.enable[1]);
  ui->MonoVocheckBox->setChecked(graph_params_.enable[2]);
  ui->SVOcheckBox->setChecked(graph_params_.enable[3]);
  
  ui->MAcheckBox->setChecked(graph_params_.enable_MA);
  ui->elevationcheckBox->setChecked(graph_params_.enable_elevation);
  ui->ackermancheckBox->setChecked(graph_params_.enable_ackerman);
  ui->imucheckBox->setChecked(graph_params_.enable_imu);
  ui->velodynecheckBox->setChecked(graph_params_.enable_velodyne);

  ui->WOstepSpinBox->setValue(graph_params_.WO_step);
  ui->GPSstepSpinBox->setValue(graph_params_.GPS_step);
//     ui->MonoVOstepSpinBox->setValue(graph_params_.VO_step);
  ui->StereoVOstepSpinBox->setValue(graph_params_.SVO_step);
  ui->CLOUDstepSpinBox->setValue(graph_params_.CLOUD_step);

  ui->WOscaletSpinBox->setValue(graph_params_.WO_scale_t);
  ui->WOscalerSpinBox->setValue(graph_params_.WO_scale_r);
  ui->GPSscaletxySpinBox->setValue(graph_params_.GPS_scale_txy);
  ui->GPSscaletzSpinBox->setValue(graph_params_.GPS_scale_tz);
//     ui->VOscaletSpinBox->setValue(graph_params_.VO_scale_t);
//     ui->VOscalerSpinBox->setValue(graph_params_.VO_scale_r);
  ui->SVOscaletSpinBox->setValue(graph_params_.SVO_scale_t);
  ui->SVOscaletzSpinBox->setValue(graph_params_.SVO_scale_tz);
  ui->SVOscalerSpinBox->setValue(graph_params_.SVO_scale_r);
  
  ui->ackscaletSpinBox->setValue(graph_params_.ACKERMAN_scale_t);
  ui->ackscalerSpinBox->setValue(graph_params_.ACKERMAN_scale_r);
  
  ui->IMUscaleSpinBox->setValue(graph_params_.IMU_scale);
  ui->CLOUDscaleSpinBox->setValue(graph_params_.CLOUD_scale);
  
  ui->MAradiusSpinBox->setValue(graph_params_.MA_radius);
  ui->MAscaleSpinBox->setValue(graph_params_.MA_scale);
  
  ui->swsizeSpinBox->setValue(graph_params_.sw_size);
  ui->swscaleSpinBox->setValue(graph_params_.sw_scale);
  
  ui->elevationscaleSpinBox->setValue(graph_params_.elevation_scale);

  ui->graphFileLineEdit->setText(QString(graph_params_.graph_filename.c_str()));
  
  ui->elevationmapFileLineEdit->setText(QString(graph_params_.elevation_map_file.c_str()));
  
  ui->outFolder_LineEdit->setText(QString(graph_params_.out_folder.c_str()));

}

void MainWindow::updateGUIOptimizerParams()
{
  ui->IterationsSpinBox->setValue(optimizer_params_.N_iterations);

  ui->graphFileLineEdit->setText(QString(optimizer_params_.in_filename.c_str()));
  ui->outFileLineEdit->setText(QString(optimizer_params_.out_filename.c_str()));    
}

///// SLOTS //////

void MainWindow::createGraph()
{
  ui->nodes_widget->setEnabled(false);

  setParams();
  if(update_requested_)
  {
    DataExtractor* de=new DataExtractor;
    de->setParams(graph_params_);
    de->extract(pose_traj_, gps_traj_, vo_traj_, gt_traj_, stats_.jai_imgs_, stats_.good_jai_idxs_, elevation_grid_, imu_traj_, clouds_);
    update_requested_=false;
  }
  cout<<"creating graph..."<<endl;
  GraphCreator* gc=new GraphCreator();
  gc->setParams(graph_params_);
  gc->create(pose_traj_, gps_traj_, vo_traj_, gt_traj_, elevation_grid_, imu_traj_, clouds_, temporal_WO_indices_);
  stats_.poses_indices_=temporal_WO_indices_;
  
  cout<<"DONE!"<<endl;
  RMSEcalculator res(graph_params_.out_folder);
  vector<nav_msgs::Odometry> v;
  rmse_=res.compute(pose_traj_, gt_traj_, v, used_gt_traj_);
  
  visGT_trajectory();
  visWO_trajectory();
}

void MainWindow::optimizeGraph()
{
  setParams();
  cout<<"optimizing graph..."<<endl;
  G2OOptimizer* optimizer=new G2OOptimizer;
  optimizer->setParams(graph_params_, optimizer_params_);
  optimizer->optimize(pose_traj_, opt_pose_traj_, false);
  cout<<"DONE!"<<endl;
  RMSEcalculator res(graph_params_.out_folder);
  rmse_=res.compute(opt_pose_traj_, gt_traj_, used_opt_poses_, used_gt_traj_);

  stats_.poses_=opt_pose_traj_;
  
  writeFilesForPlots(graph_params_.out_folder);
  
  ui->nodesSlider->setMaximum(temporal_WO_indices_.size()-1);
  ui->nodesspinBox->setMaximum(temporal_WO_indices_.size()-1);
  ui->nodes_widget->setEnabled(true);
  
  visGT_trajectory();
  visWO_trajectory();
}

void MainWindow::sw_optimizeGraph()
{
  setParams();
  
  sw_optimizer_->setParams(graph_params_, optimizer_params_);
  
  emit sw_optimizeSignal(pose_traj_, gps_traj_, vo_traj_, gt_traj_,elevation_grid_);
}

void MainWindow::optimizeGraphManifold()
{
  setParams();
  cout<<"optimizing graph manifold assumption..."<<endl;
  G2OOptimizer* optimizer=new G2OOptimizer;
  optimizer->setParams(graph_params_, optimizer_params_);
  optimizer->optimize(pose_traj_, opt_pose_traj_, true);
  cout<<"DONE!"<<endl;
  RMSEcalculator res(graph_params_.out_folder);
  rmse_=res.compute(opt_pose_traj_, gt_traj_, used_opt_poses_, used_gt_traj_);

  writeFilesForPlots(graph_params_.out_folder);
  
  visGT_trajectory();
  visWO_trajectory();
}

void MainWindow::saveGraphParams()
{
  setParams();
  QString save_filename = QFileDialog::getSaveFileName(this, "Save GraphParams file", QString(), QString("*.yaml"));
  if(save_filename.length()>0)
  {
    if(!save_filename.contains(QString(".yaml")))
      save_filename.append(".yaml");
    graph_params_.writeParams(save_filename.toStdString());
  }
}

void MainWindow::saveOptimizationParams()
{
  setParams();
  QString save_filename = QFileDialog::getSaveFileName(this, "Save OptimizerParams file", QString(), QString("*.yaml"));
  if(save_filename.length()>0)
  {
    if(!save_filename.contains(QString(".yaml")))
      save_filename.append(".yaml");
    optimizer_params_.writeParams(save_filename.toStdString());
  }
}

void MainWindow::loadGraphParams()
{
  QString filename = QFileDialog::getOpenFileName(this, "load graph params", QString(), "*.yaml" );
  if (!QFile::exists(filename)) {
    QMessageBox::critical(this, "Error!", "File does not exist.");
    return;
  }
  graph_params_.readParams(filename.toStdString());

  updateGUIGraphParams();
}

void MainWindow::loadOptimizationParams()
{
  QString filename = QFileDialog::getOpenFileName(this, "load optimizer params", QString(), "*.yaml" );
  if (!QFile::exists(filename)) {
    QMessageBox::critical(this, "Error!", "File does not exist.");
    return;
  }
  optimizer_params_.readParams(filename.toStdString());

  updateGUIOptimizerParams();
}

void MainWindow::readGraphParamsFromFile(const string yaml_filename)
{
  graph_params_.readParams(yaml_filename);

  updateGUIGraphParams();
}

void MainWindow::readOptimizerParamsFromFile(const string yaml_filename)
{
  optimizer_params_.readParams(yaml_filename);

  updateGUIOptimizerParams();
}

void MainWindow::visPointCloud()
{
  setParams();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
  Eigen::Isometry3d pose, calib;
  CovarianceMatrixd cov;
  calib.translation()=Eigen::Vector3d(0.689, -0.612, 0.977);
  Eigen::Quaterniond q_vel; q_vel.x()=0.168365; q_vel.y()=-.183284; q_vel.z()=.968419; q_vel.w()=.0150184;
//     q_vel.normalize();
  calib.linear()=q_vel.toRotationMatrix();
  
  if(ui->visPointCloudcheckBox->isChecked())
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<temporal_WO_indices_.size(); i+=4)
    {
      int idx=temporal_WO_indices_[i];
      tss_utils::poseWithCovarianceMsg2EigenPoseCov(opt_pose_traj_[idx].pose, pose, cov);
      
      if(clouds_[idx].header.seq>0)
      {
        
        tss_utils::cloudMsg2PCL(clouds_[idx], *curr_cloud);
        
//         Eigen::Quaterniond q(pose.linear());q.x()=0; q.y()=0;
//         if(q.w()<0)
//           q.z()*=-1;
//         q.w()=sqrt(1 - (q.z()*q.z()));
//         q.normalize();
//         pose.linear()=q.toRotationMatrix();
        pcl::transformPointCloud(*curr_cloud, *curr_cloud, (pose*calib).matrix());
//         pcl::VoxelGrid<pcl::PointXYZ > sor;
//         sor.setInputCloud (curr_cloud);
//         sor.setLeafSize (0.25f, 0.25f, 0.25f);
//         sor.filter (*curr_cloud);
        
        pcl::CropBox<pcl::PointXYZ > cropFilter(true);
        cropFilter.setInputCloud(curr_cloud);

        cropFilter.setMin(Eigen::Vector4f(-2,-2,-.5, 0)); 
        cropFilter.setMax(Eigen::Vector4f(2,2,1.5, 0)); 
        cropFilter.setTranslation(Eigen::Vector3f(pose.translation()(0),pose.translation()(1),pose.translation()(2)));
        std::vector<int> ind; 
        cropFilter.filter(ind);
//         pcl::IndicesPtr indices(ind);
        
        pcl::ExtractIndices<pcl::PointXYZ> extract ; 
        extract.setInputCloud (curr_cloud); 
        boost::shared_ptr<vector<int> > indicesptr (new vector<int> (ind)); 
        extract.setIndices (indicesptr); 
        //extract.setNegative (false); //Removes part_of_cloud but retain the original full_cloud 
        extract.setNegative (true); // Removes part_of_cloud from full cloud  and keep the rest 
        extract.filter (*curr_cloud); 
        
//         pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointNormal>);
// 
//         pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
//         pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//         norm_est.setSearchMethod (tree);
//         norm_est.setKSearch (30);
//         
//         norm_est.setInputCloud (curr_cloud);
//         norm_est.compute (*points_with_normals_src);
//         pcl::copyPointCloud (*curr_cloud, *points_with_normals_src);
        
        *cloud+=*curr_cloud;
        std::cout<<i<<" "<<idx<<" "<<curr_cloud->size()<<" "<<cloud->size()<<endl;
      }

    }
  }
//   pcl::VoxelGrid<pcl::PointXYZ > sor;
//   sor.setInputCloud (cloud);
//   sor.setLeafSize (0.35f, 0.35f, 0.15f);
//   sor.filter (*cloud);

//   pcl::io::savePCDFileASCII ("cloud.pcd", *cloud);
  if(cloud->size()>0)
    pcl::io::savePCDFileBinaryCompressed("cloud.pcd", *cloud);
//   pcl::io::loadPCDFile("cloud.pcd", *cloud);
  std::cout<<" cloud size: "<<cloud->size()<<endl;
  ui->openGLWidget->setPointCloud(cloud);
  std::cout<<" DONE: "<<cloud->size()<<endl;

}

void MainWindow::visWO_trajectory()
{ 
//   setParams();
  std::vector<Eigen::Isometry3d> traj;

  if(ui->visWOcheckBox->isChecked())
  {
    string graph_name;
    if(ui->original_radioButton->isChecked())
      tss_utils::odometry2EigenPoseList(pose_traj_, traj);
    else
      tss_utils::odometry2EigenPoseList(opt_pose_traj_, traj);
  }
  ui->openGLWidget->setTrajectoryPoints(traj, Eigen::Vector3d(1,1,1));
  ui->openGLWidget->update();
}

void MainWindow::visGT_trajectory()
{
  std::vector<Eigen::Isometry3d> traj; 

  if(ui->visGTcheckBox->isChecked())
    tss_utils::odometry2EigenPoseList(gt_traj_, traj);

  ui->openGLWidget->setGTTrajectoryPoints(traj, Eigen::Vector3d(1,.2,.1));
  ui->openGLWidget->update();
}

void MainWindow::visGPS_trajectory()
{
  std::vector<Eigen::Isometry3d> traj; 

  if(ui->visGPScheckBox->isChecked())
    tss_utils::odometry2EigenPoseList(gps_traj_, traj);

  ui->openGLWidget->setGPSTrajectoryPoints(traj, Eigen::Vector3d(.1,.5,1));
  ui->openGLWidget->update();
}

void MainWindow::visVO_trajectory()
{
  std::vector<Eigen::Isometry3d> traj; 

  if(ui->visVOcheckBox->isChecked())
    tss_utils::odometry2EigenPoseList(vo_traj_, traj);

  ui->openGLWidget->setVOTrajectoryPoints(traj, Eigen::Vector3d(.1,1,.1));
  ui->openGLWidget->update();
}

void MainWindow::visSelectedNode(int n)
{
  std::vector<Eigen::Isometry3d> traj;
  tss_utils::odometry2EigenPoseList(opt_pose_traj_, traj);
  if(!ui->original_radioButton->isChecked() && opt_pose_traj_.size()>n)
  {
    ui->openGLWidget->setNodePoints(traj[n].translation(), .5, Eigen::Vector3d(.1,1,1));
    ui->openGLWidget->update();
  }
  visCLOUD(n);
}

void MainWindow::visJAI(int n)
{
  int N=3;
  int iter=0;
  std::vector<cv::Mat> images;
  int i=n;
  if(!ui->original_radioButton->isChecked() && temporal_WO_indices_.size()>n)
    i=temporal_WO_indices_[n];
  
  while(iter<N)
  {
    int idx=(i-2)+iter;
    if(idx<0||idx>=stats_.jai_imgs_.size())
    {
      iter++;
      continue;
    }
    cv::Mat im;
    if(!tss_utils::imageMsg2CvMat(stats_.jai_imgs_[idx], im))
    {
      iter++;
      N++;
      continue;
    }
    images.push_back(im);
    iter++; 
  }
  
//   stats_.compute(n);
  
  //compute stitcing
//   cv::Mat stitched_img;
//   image_stitching(images, stitched_img);
//   
//   QImage imgIn= QImage((uchar*) stitched_img.data, stitched_img.cols, stitched_img.rows, stitched_img.step, QImage::Format_RGB888);
//   ui->imageLabel->setPixmap(QPixmap::fromImage(imgIn));
}

void MainWindow::visCLOUD(int n)
{
  if(n<1) return;
  int i=temporal_WO_indices_[n];
  Eigen::Isometry3d pose, calib;
  CovarianceMatrixd cov;
  calib.translation()=Eigen::Vector3d(0.689, -0.612, 0.977);
  Eigen::Quaterniond q_vel; q_vel.x()=0.168365; q_vel.y()=-.183284; q_vel.z()=.968419; q_vel.w()=.0150184;
//     q_vel.normalize();
  calib.linear()=q_vel.toRotationMatrix();
  pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud(new  pcl::PointCloud<pcl::PointXYZ>());

  
    Eigen::Isometry3d last_pose;

    tss_utils::poseWithCovarianceMsg2EigenPoseCov(pose_traj_[temporal_WO_indices_[n-1]].pose, last_pose, cov);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZ>());
    for(int h=temporal_WO_indices_[n-1]; h<=temporal_WO_indices_[n]; h++)
    {
      
      tss_utils::poseWithCovarianceMsg2EigenPoseCov(pose_traj_[h].pose, pose, cov);
      if(clouds_[h].header.seq>0)
      {
        tss_utils::cloudMsg2PCL(clouds_[h], *cloud);
        pcl::transformPointCloud(*cloud, *cloud, (last_pose.inverse()*pose*calib).matrix());
        *curr_cloud+=*cloud;
      }  
    }
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(integrated_clouds_[n]));
//     tss_utils::cloudMsg2PCL(clouds_[i],*cloud);
    
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(opt_pose_traj_[i].pose, pose, cov);
    
    pcl::transformPointCloud(*curr_cloud, *curr_cloud, (pose).matrix());
    ui->openGLWidget->setPointCloud(curr_cloud); 
}

void MainWindow::nodeVisualization()
{
  this->ui->nodesspinBox->blockSignals(false);
  int v=this->ui->nodesSlider->value();
  this->ui->nodesspinBox->setValue(v);
  this->ui->nodesspinBox->blockSignals(false);
  
  visSelectedNode(v);
  visJAI(v);
}

void MainWindow::nodesspinBox_slot()
{
  int v=this->ui->nodesspinBox->value();
  this->ui->nodesSlider->setValue(v);
}

void MainWindow::computeStatistics()
{
  stats_.compute();
}

void MainWindow::writeFilesForPlots(const string& out_folder)
{
  Eigen::Isometry3d pose;
  CovarianceMatrixd cov;
  
  ofstream pose_ofs(out_folder+"wo.txt");
  for(int i=0; i<pose_traj_.size(); i++)
  {
    if(pose_traj_[i].header.seq<=0)
      continue;
    ros::Time stamp=pose_traj_[i].header.stamp;
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(pose_traj_[i].pose, pose, cov);
    Eigen::Quaterniond q(pose.linear());
    pose_ofs<<fixed<<stamp.toSec()<<" "<<pose.translation()(0)<<" "<<pose.translation()(1)<<" "<<pose.translation()(2)
            <<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
  }
  pose_ofs.close();
  
  ofstream gps_ofs(out_folder+"gps.txt");
  for(int i=0; i<gps_traj_.size(); i++)
  {
    if(gps_traj_[i].header.seq<=0)
      continue;
    ros::Time stamp=gps_traj_[i].header.stamp;
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(gps_traj_[i].pose, pose, cov);
    Eigen::Quaterniond q(pose.linear());
    gps_ofs<<fixed<<stamp.toSec()<<" "<<pose.translation()(0)<<" "<<pose.translation()(1)<<" "<<pose.translation()(2)
            <<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
  }
  gps_ofs.close();
  
  ofstream vo_ofs(out_folder+"vo.txt");
  for(int i=0; i<vo_traj_.size(); i++)
  {
    if(vo_traj_[i].header.seq<=0)
      continue;
    ros::Time stamp=vo_traj_[i].header.stamp;
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(vo_traj_[i].pose, pose, cov);
    Eigen::Quaterniond q(pose.linear());
    vo_ofs<<fixed<<stamp.toSec()<<" "<<pose.translation()(0)<<" "<<pose.translation()(1)<<" "<<pose.translation()(2)
            <<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
  }
  vo_ofs.close();
  
  ofstream gt_ofs(out_folder+"gt.txt");
  for(int i=0; i<gt_traj_.size(); i++)
  {
    if(gt_traj_[i].header.seq<=0)
      continue;
    ros::Time stamp=gt_traj_[i].header.stamp;
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(gt_traj_[i].pose, pose, cov);
    Eigen::Quaterniond q(pose.linear());
    gt_ofs<<fixed<<stamp.toSec()<<" "<<pose.translation()(0)<<" "<<pose.translation()(1)<<" "<<pose.translation()(2)
            <<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
  }
  gt_ofs.close();
  
  ofstream opt_ofs(out_folder+"optimized_poses.txt");
  for(int i=0; i<opt_pose_traj_.size(); i++)
  {
    if(opt_pose_traj_[i].header.seq<=0)
      continue;
    ros::Time stamp=opt_pose_traj_[i].header.stamp;
    tss_utils::poseWithCovarianceMsg2EigenPoseCov(opt_pose_traj_[i].pose, pose, cov);
    Eigen::Quaterniond q(pose.linear());
    opt_ofs<<fixed<<stamp.toSec()<<" "<<pose.translation()(0)<<" "<<pose.translation()(1)<<" "<<pose.translation()(2)
            <<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
  }
  opt_ofs.close();
  
  ofstream rmse_ofs(out_folder+"rmse.txt");
  rmse_ofs<<rmse_<<endl;
  rmse_ofs.close();
  
  graph_params_.writeParams(out_folder+"gui_params.yaml");
}

void MainWindow::showPosesSlot(vector< nav_msgs::Odometry > poses)
{
  opt_pose_traj_=poses;
  visWO_trajectory();
}

