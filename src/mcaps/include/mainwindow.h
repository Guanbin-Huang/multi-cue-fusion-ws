#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <eigen3/Eigen/Dense>

#include "paramsUtils.h"
#include "interpolation_utils.h"
#include "pose_graph/temporalgraph.h"
#include "pose_graph/tss_utils.h"
#include "g2o_optimizer.h"
#include "data_statistics.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setParams();
    void writeGraphParamsOnFile(const std::string yaml_filename);
    void writeOptimizerParamsOnFile(const std::string yaml_filename);
    void readGraphParamsFromFile(const std::string yaml_filename);
    void readOptimizerParamsFromFile(const std::string yaml_filename);

    void updateGUIGraphParams();
    void updateGUIOptimizerParams();
    
    void writeFilesForPlots(const std::string& out_folder);

public Q_SLOTS:

    void createGraph();
    void optimizeGraph();
    void sw_optimizeGraph();
    void optimizeGraphManifold();
    void saveGraphParams();
    void saveOptimizationParams();
    void loadGraphParams();
    void loadOptimizationParams();
    
    void setUpdateRequest(){update_requested_=true;}

    void visPointCloud();
    void visWO_trajectory();
    void visGT_trajectory();
    void visGPS_trajectory();
    void visVO_trajectory();
    
    void visJAI(int);
    void visSelectedNode(int);
    void nodeVisualization();
    void nodesspinBox_slot();
    
    void visCLOUD(int n);
    
    void showPosesSlot(std::vector<nav_msgs::Odometry> poses);
    
    void computeStatistics();
  

Q_SIGNALS:
  
  void sw_optimizeSignal(const std::vector<nav_msgs::Odometry> wo,
                        const std::vector<nav_msgs::Odometry> gps,
                        const std::vector<nav_msgs::Odometry> vo,
                        const std::vector<nav_msgs::Odometry> gt,
                        const std::vector<Eigen::Vector3d> elev);

private:
    Ui::MainWindow *ui;
    Configuration::GraphParams graph_params_;
    Configuration::OptimizerParams optimizer_params_;
    
    bool update_requested_;
    
    std::vector<nav_msgs::Odometry> gt_traj_, opt_pose_traj_, pose_traj_, vo_traj_, gps_traj_, imu_traj_,
                                    used_gt_traj_, used_opt_poses_;
                                    
    std::vector<sensor_msgs::PointCloud2> clouds_;
                                    
    std::vector<Eigen::Vector3d> elevation_grid_;

    std::vector<int> temporal_WO_indices_;
    
    double rmse_;
    
    G2OOptimizer* sw_optimizer_;
    DataStatistics stats_;
    
    QThread thread_;

};

#endif // MAINWINDOW_H
