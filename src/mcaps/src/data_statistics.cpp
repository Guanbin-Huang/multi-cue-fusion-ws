#include "data_statistics.h"

double DataStatistics::computeGreenDensity(const vector< cv::Mat >& im)
{

}

double DataStatistics::computeGreenDensity(const cv::Mat& im)
{
  cv::Mat grey;
  cv::cvtColor(im, grey, CV_RGB2GRAY);
  cv::Mat g, r, points;
  cv::extractChannel(im, g,1);
  cv::extractChannel(im, r,0);
  g=g-r;
  cv::threshold(g,g,10,255,cv::THRESH_BINARY);
  cv::findNonZero(g,points);
  
//   cv::imshow("ciao", g);
//   cv::waitKey(20);
  
  return (double)points.rows/(double)(g.rows*g.cols);
}

Statistics DataStatistics::compute(int i)
{
  Statistics st;
  if(i>=poses_indices_.size())
    return st;
  
  cv::Mat im;
  if(!tss_utils::imageMsg2CvMat(jai_imgs_[poses_indices_[i]], im))
    return st;
    
  st.green_density=computeGreenDensity(im);
  
  return st;
}

void DataStatistics::compute()
{
  poses_stats_.clear();
  poses_stats_.resize(poses_.size());
  for(int i=0; i<poses_.size(); i++)
  {
    poses_stats_[i]=compute(i);
    poses_stats_[i].print();
  }
}

