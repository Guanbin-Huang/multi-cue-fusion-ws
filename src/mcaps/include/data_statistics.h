#ifndef DATA_STATISTICS_H
#define DATA_STATISTICS_H

#include "paramsUtils.h"
#include "interpolation_utils.h"
#include "pose_graph/temporalgraph.h"
#include "pose_graph/tss_utils.h"
#include "elevation_utils.h"
#include <opencv2/opencv.hpp>

struct Statistics
{
  Statistics()
  {
    green_density=0;
    crop_density=0;
    weed_density=0;
  }
  
  double green_density;
  double crop_density, weed_density;
  
  void print()
  {
    std::cout<<endl;
    std::cout<< "---"<<endl;
    std::cout<< "green_density: "<<green_density<<endl;
    std::cout<< "crop_density: "<<crop_density<<endl;
    std::cout<< "weed_density: "<<weed_density<<endl;
    std::cout<< "---"<<endl;
    std::cout<<endl;
  }
};

class DataStatistics
{
public:
  DataStatistics(){}
  ~DataStatistics(){}
  
  void compute();
  Statistics compute(int i);
  
  std::vector<Statistics> poses_stats_;
  std::vector<nav_msgs::Odometry> poses_;
  std::vector<sensor_msgs::Image> jai_imgs_;
  std::vector<int> good_jai_idxs_, poses_indices_;
    
private:
  
  double computeGreenDensity(const cv::Mat& im);
  double computeGreenDensity(const std::vector<cv::Mat>& im);
};

#endif // DATA_STATISTICS_H