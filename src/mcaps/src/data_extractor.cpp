#include "data_extractor.h"

void DataExtractor::computeElevetionGrid(vector< Eigen::Vector3d >& grid, double z_offset)
{
  std::ifstream inputfile;
  inputfile.open(params_.tss_maindir + "/" + params_.elevation_map_file);
  if (!inputfile.is_open()) {
    std::cerr << "***[error]: file path not valid. Error in opening ***" << std::endl;
    std::exit(-1);
  }
  
  vector< Eigen::Vector3d > elevation_grid;
  std::string line, prefix;
  while (!inputfile.eof()) {
    getline(inputfile, line);
    if (line.length() != 0 && line[0] != '#') //discard line beginning with '#'
      {
  char *pch;
  char *cstr_line = new char[line.length() + 1];
  strcpy(cstr_line, line.c_str());
  pch = std::strtok(cstr_line, " ");
  std::vector<double> temp;
    while (pch != NULL) {
      std::istringstream ss(pch);
      double f;
      if (ss >> f)
        temp.push_back(f);
      else
        std::cerr << "***[error]: invalid conversion from string to float ***" << std::endl;
      pch = std::strtok(NULL, " ");
    }
    elevation_grid.push_back( Eigen::Vector3d( temp.at(0), temp.at(1), temp.at(2) ) );

      }

  }// End While
  inputfile.close();
  
  double start_lat, start_lon, start_elev;
  Eigen::Quaterniond gps_quat_calib;
  
  if(params_.bag_filename.find("setB")==std::string::npos) //so it is datasetA
  {
    start_lat = 47.4501173333; 
    start_lon = 8.68260966667;
    start_elev = 554.3696289;
    cout<<"DATASET A"<<endl;
    gps_quat_calib=Eigen::Quaterniond(Eigen::AngleAxisd( -2.18, Eigen::Vector3d::UnitZ() ));
  }
  else
  {
    start_lat = 47.4501056667; 
    start_lon = 8.68261683333;
//     start_elev = 605.1;
//     start_lat = 47.4501173333; 
//     start_lon = 8.68260966667;
    start_elev = 554.3696289;
    cout<<"DATASET B"<<endl;    
    gps_quat_calib=Eigen::Quaterniond(Eigen::AngleAxisd( -2.67, Eigen::Vector3d::UnitZ() ));
  }
    
  geodetic_converter::GeodeticConverter geodetic_to_NED;
  geodetic_to_NED.initialiseReference(start_lat, start_lon, start_elev);
  
  grid.clear();
  double offset;
  for(unsigned int iter=0; iter < elevation_grid.size(); iter++)
  {
    double x,y,z;
    
    geodetic_to_NED.geodetic2Ned(elevation_grid[iter](0), elevation_grid[iter](1)+0.0003, elevation_grid[iter](2), &x,&y,&z );
    grid.push_back( gps_quat_calib.toRotationMatrix().inverse()*Eigen::Vector3d(x,y,z)); 
    if(iter==0)
      offset=grid[0](2);
    grid[iter](2)-=offset;
//     std::cout << grid.at(grid.size()-1).transpose() << std::endl;
  }
}


void DataExtractor::extract(std::vector<nav_msgs::Odometry>& wo,
                            std::vector<nav_msgs::Odometry>& gps,
                            std::vector<nav_msgs::Odometry>& vo,
                            std::vector<nav_msgs::Odometry>& gt,
                            std::vector<sensor_msgs::Image>& jai,
                            std::vector<int>& good_jai_idxs,
                            std::vector<Eigen::Vector3d>& elevation_grid,
                            std::vector<nav_msgs::Odometry>& imu,
                            std::vector<sensor_msgs::PointCloud2>& cloud
                            )
{ 
  TSS* temporalgraph = new TSS(params_.tss_maindir);
  temporalgraph->load(params_.tss_filename);
  temporalgraph->setBag(params_.bag_filename);
    
  gt.clear(); wo.clear(); gps.clear(); vo.clear(); jai.clear(), good_jai_idxs.clear(), imu.clear(), cloud.clear();
  gt.resize(temporalgraph->size(), nav_msgs::Odometry());
  wo.resize(temporalgraph->size(), nav_msgs::Odometry());
  gps.resize(temporalgraph->size(), nav_msgs::Odometry());
  vo.resize(temporalgraph->size(), nav_msgs::Odometry());
  jai.resize(temporalgraph->size(), sensor_msgs::Image());
  imu.resize(temporalgraph->size(), nav_msgs::Odometry());
  cloud.resize(temporalgraph->size(), sensor_msgs::PointCloud2());
  
  elevation_grid.clear();
  
  for(int i=0; i<temporalgraph->size(); i++)
  {
    gt[i].header.seq=0;
    wo[i].header.seq=0;
    gps[i].header.seq=0;
    vo[i].header.seq=0;
    jai[i].header.seq=0;
    imu[i].header.seq=0;
    cloud[i].header.seq=0;
  }
  
  cout<<"reading bag and extracting data..."<<endl;
  bool first_leica, first_gps, first_vo;
  first_leica=first_gps=first_vo=false;
  nav_msgs::Odometry first_leica_data, first_gps_data, first_vo_data;
  for(int i=0; i<temporalgraph->size(); i++)
  {  
    if(temporalgraph->nodeHasMeasurement(i, params_.JAI_RGB_topic))
    {
      jai[i]=(*temporalgraph->getNodeData<sensor_msgs::Image>(i, params_.JAI_RGB_topic));
      good_jai_idxs.push_back(i);
    }
    if(!temporalgraph->nodeHasMeasurement(i, params_.GPS_topic))
      continue;
    
    nav_msgs::OdometryConstPtr gps_msg=temporalgraph->getNodeData<nav_msgs::Odometry>(i, params_.GPS_topic);
    if(gps_msg)
    {
      gps[i]=(*gps_msg);
      if(!first_gps)
      {
        first_gps=true;
        first_gps_data=gps[i];
      }
      gps[i].pose.pose.position.x-=first_leica_data.pose.pose.position.x;
      gps[i].pose.pose.position.y-=first_leica_data.pose.pose.position.y;
      gps[i].pose.pose.position.z-=first_leica_data.pose.pose.position.z;
    }
    
    if(temporalgraph->nodeHasMeasurement(i, params_.GT_topic))
    {
      gt[i]=(*temporalgraph->getNodeData<nav_msgs::Odometry>(i, params_.GT_topic));
      if(!first_leica)
      {
        Eigen::Vector3d t(gt[i].pose.pose.position.x, gt[i].pose.pose.position.y, gt[i].pose.pose.position.z);
        if(t.norm()<1000)
        {
          first_leica=true;
          first_leica_data=gt[i];
        }
      }
      gt[i].pose.pose.position.x-=first_leica_data.pose.pose.position.x;
      gt[i].pose.pose.position.y-=first_leica_data.pose.pose.position.y;
      gt[i].pose.pose.position.z-=first_leica_data.pose.pose.position.z;
    }
    if(temporalgraph->nodeHasMeasurement(i, params_.WO_topic))
      wo[i]=(*temporalgraph->getNodeData<nav_msgs::Odometry>(i, params_.WO_topic));
    if(temporalgraph->nodeHasMeasurement(i, params_.SVO_topic))
    {
      vo[i]=(*temporalgraph->getNodeData<nav_msgs::Odometry>(i, params_.SVO_topic));
      if(!first_vo)
      {
        first_vo=true;
        first_vo_data=vo[i];
      }
      vo[i].pose.pose.position.x-=first_leica_data.pose.pose.position.x;
      vo[i].pose.pose.position.y-=first_leica_data.pose.pose.position.y;
      vo[i].pose.pose.position.z-=first_leica_data.pose.pose.position.z;
    }
    if(temporalgraph->nodeHasMeasurement(i, params_.IMU_topic))
      imu[i]=(*temporalgraph->getNodeData<nav_msgs::Odometry>(i, params_.IMU_topic));
    if(temporalgraph->nodeHasMeasurement(i, params_.CLOUD_topic))
      cloud[i]=(*temporalgraph->getNodeData<sensor_msgs::PointCloud2>(i, params_.CLOUD_topic));
  }
  temporalgraph->closeBag();
  
  computeElevetionGrid(elevation_grid, first_leica_data.pose.pose.position.z);
  
  cout<<"all data extracted!"<<endl;
}