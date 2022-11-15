#pragma once

#include "srrg_image_utils/depth_utils.h"
#include "srrg_nicp/projective_aligner.h"
#include "srrg_nicp/spherical_projector.h"
#include <srrg_system_utils/system_utils.h>
#include "srrg_types/spherical_camera_info.h"

namespace nicp_utils
{
  static void makeSphericalImage(srrg_core::FloatImage& depth, 
                                const std::vector<Eigen::Vector3f>& points,
                                float horizontal_fov=M_PI,
                                float vertical_fov=M_PI/2,
                                float horizontal_res=360/M_PI,
                                float vertical_res=360/M_PI,
                                float range_max=30.0f)
  {
    // compute the size of the image
    int cols=ceil(2*horizontal_fov*horizontal_res);
    int rows=ceil(2*vertical_fov*vertical_res);
    depth.create(rows,cols);
    depth=range_max;
    for (size_t i=0; i<points.size(); i++)
    {
      const Eigen::Vector3f p=points[i];
      float range=p.norm();
      if (range>range_max){
        continue;
      }
      // compute azimuth and elevation, in polar coordinates
      float azimuth=atan2(p.y(),p.x());
      if (fabs(azimuth)>horizontal_fov){
        continue;
      }
      float xy_norm=p.head<2>().norm();
      float elevation=atan2(p.z(),xy_norm);
        
      if (fabs(elevation)>vertical_fov){
        continue;
      }
      // compute the binning
      float r=rows/2+round(vertical_res*elevation);
      float c=cols/2+round(horizontal_res*azimuth);
      if (r<0||r>=rows) {
        continue;
      }
      if (c<0||c>=cols)
        continue;
      float &d=depth.at<float>(r,c);
      if (range<d){
        d=range;
      }
    }
    cv::Mat dbg(depth);
    cv::normalize(dbg,dbg, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imshow("test", dbg);
    cv::waitKey(0);
  }

  static void computeCloudFromSphericalImage(srrg_nicp::SphericalProjector& projector, const srrg_core::FloatImage& depth, const Eigen::Vector4f& K, srrg_core::Cloud3D& dest_model)
  {
    srrg_core::UnsignedShortImage spherical_short_image;
    float depth_scale=1e2; //1cm/px
    srrg_core::convert_32FC1_to_16UC1(spherical_short_image,depth, depth_scale);
    
    projector.setImageSize(depth.rows, depth.cols);
    projector.unproject(dest_model,spherical_short_image); 
  }

  static void computeCloudFromVector3fVector(srrg_nicp::SphericalProjector& projector, const std::vector<Eigen::Vector3f>& points, const Eigen::Vector4f& K, srrg_core::Cloud3D& dest_model)
  { 
    srrg_core::FloatImage spherical_depth;
    float depth_scale=1e2; //1cm/px
    makeSphericalImage(spherical_depth, points, K(0), K(1), K(2), K(3), 30.0f);
    computeCloudFromSphericalImage(projector, spherical_depth, K, dest_model);
        std::cout<<"unprojected spheriacl image..."<<std::endl;

  }

}
