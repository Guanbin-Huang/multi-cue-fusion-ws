#ifndef DATA_EXTRACTOR_H
#define DATA_EXTRACTOR_H

#include "paramsUtils.h"
#include "interpolation_utils.h"
#include "pose_graph/temporalgraph.h"
#include "pose_graph/tss_utils.h"
#include <geodetic_utils/geodetic_conv.hpp>

class DataExtractor
{
public:
    DataExtractor(){}
    ~DataExtractor(){}

    void setParams(const Configuration::GraphParams& params){params_=params;}
    void extract(std::vector<nav_msgs::Odometry>& wo,
                 std::vector<nav_msgs::Odometry>& gps,
                 std::vector<nav_msgs::Odometry>& vo,
                 std::vector<nav_msgs::Odometry>& gt,
                 std::vector<sensor_msgs::Image>& jai,
                 std::vector<int>& good_jai_idxs,
                 std::vector<Eigen::Vector3d>& elevation_grid,
                 std::vector<nav_msgs::Odometry>& imu,
                 std::vector<sensor_msgs::PointCloud2>& cloud
                 );
private:
     
    void computeElevetionGrid(std::vector<Eigen::Vector3d>& grid, double z_offset);
  
    Configuration::GraphParams params_;
};

#endif // DATA_EXTRACTOR_H

