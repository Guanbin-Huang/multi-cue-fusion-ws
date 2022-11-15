#pragma once

#include "paramsUtils.h"
#include "pose_graph/tss_utils.h"

static double computeGmapElevation(const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& elevation_grid)
{
  std::map<double, Eigen::Vector3d> distanceongrid;
  for(unsigned int iter = 0; iter < elevation_grid.size(); iter++)
  {
    float d = ( p.head(2) - elevation_grid[iter].head(2) ).norm();
    distanceongrid[d] = elevation_grid[iter];
  }
  
//   std::cout << p.transpose() << std::endl;
  
  std::map<double, Eigen::Vector3d>::iterator map_iter = distanceongrid.begin();
  
  std::pair<double, Eigen::Vector3d> first = std::pair<double, Eigen::Vector3d>( (map_iter)->first, (map_iter)->second);
  map_iter++;
  std::pair<double, Eigen::Vector3d> second = std::pair<double, Eigen::Vector3d>( (map_iter)->first, (map_iter)->second);
  map_iter++;
  std::pair<double, Eigen::Vector3d> third = std::pair<double, Eigen::Vector3d>( (map_iter)->first, (map_iter)->second);
  map_iter++;
  std::pair<double, Eigen::Vector3d> fourth = std::pair<double, Eigen::Vector3d>( (map_iter)->first, (map_iter)->second);
    
//   std::cout << first.first << std::endl;
  
  double sum = first.first + second.first + third.first + fourth.first;
  double v1 = first.first/sum;
  double v2 = second.first/sum;
  double v3 = third.first/sum;
  double v4 = fourth.first/sum;
  
  double z = first.second(2) * v1 + second.second(2) * v2 + third.second(2) * v3 + fourth.second(2) * v4;
//   z-=51;
  std::cout << "quota da Gmaps: " << z << " " << p(2) << std::endl;
  
  return z;
}