#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>

static void image_stitching(const std::vector<cv::Mat>& imgs, cv::Mat& stitched_image)
{ 
  std::setlocale(LC_ALL, "en_US.UTF-8");
  
  if(imgs.size()<=0)
    return;
  
  cv::Ptr<cv::Feature2D> orb = cv::ORB::create(); 
  cv::FlannBasedMatcher matcher;  
  cv::BFMatcher bfmatcher;

  std::vector<cv::KeyPoint> kpts; 
  cv::Mat dscptrs;

  cv::Mat total_H = cv::Mat::eye(3,3,CV_64FC1);
  cv::Mat big_stitched_image=cv::Mat(imgs[0].size().width*2, imgs[0].size().height*2, imgs[0].type(), cv::Scalar(0,0,0));
  
  for(int i=0; i<imgs.size(); i++)
  {
    std::vector<cv::KeyPoint> curr_kpts; 
    cv::Mat curr_dscptrs;

    orb->detectAndCompute(imgs[i], cv::noArray(), curr_kpts, curr_dscptrs);
        
    if(i==0)
    {
      kpts = curr_kpts;
      dscptrs = curr_dscptrs.clone();
      continue;
    }
      
    std::vector< cv::DMatch > matches;
    bfmatcher.match(dscptrs, curr_dscptrs, matches);
    std::vector< cv::DMatch > ref_matches;
      
    float max = 0;
    
    for( unsigned int iter = 0; iter < matches.size(); iter++)
      if( matches.at(iter).distance > max )
        max = matches.at(iter).distance;
      
    for( unsigned int iter = 0; iter < matches.size(); iter++)
      if(matches.at(iter).distance < max/2.5)
        ref_matches.push_back(matches.at(iter));

    std::vector<cv::Point2f> obj, scene;
    for(unsigned int iter = 0; iter < ref_matches.size(); iter++)
    {
      obj.push_back( kpts[ref_matches.at(iter).queryIdx].pt );
      scene.push_back( curr_kpts[ref_matches.at(iter).trainIdx].pt );
    }
      
    cv::Mat out_img; 
    cv::Mat H;
    if(ref_matches.size()<10)
      H=cv::Mat::eye(3,3,CV_64FC1);
    else
      H = cv::findHomography(scene, obj, cv::RANSAC);
      
    total_H=total_H*H;   
    double scale=2;
    
    cv::Mat P=total_H.clone();
    P.at<double>(2,2)=scale;
    P.at<double>(0,2)+=big_stitched_image.cols/2;
    P.at<double>(1,2)+=big_stitched_image.rows/2;
    cv::warpPerspective(imgs[i], out_img, P, big_stitched_image.size());
    
    out_img.copyTo(big_stitched_image, out_img);   
    
    kpts = curr_kpts,
    dscptrs = curr_dscptrs.clone();
  }
  
  cv::Mat points, stitched_bin, stitched_image_grey;
  cv::cvtColor(big_stitched_image,stitched_image_grey,cv::COLOR_BGR2GRAY);
  cv::threshold(stitched_image_grey,stitched_bin,1,255,cv::THRESH_BINARY);
  cv::findNonZero(stitched_bin,points);
  cv::Rect Min_Rect=boundingRect(points);
  stitched_image=cv::Mat(Min_Rect.size(), imgs[0].type());
  stitched_image=big_stitched_image(Min_Rect);
  
  cv::resize(stitched_image, stitched_image, cv::Size(), 0.4, 0.4);
}
