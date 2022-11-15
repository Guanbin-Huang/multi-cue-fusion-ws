## Mapper_Linker

![N|Solid](http://www.allaboutlean.com/wp-content/uploads/2016/12/Fotolia-Grass-and-scissors.jpg)

## Dependencies

- Ros Indigo/Kinetic
- Eigen 2.x/3
- Yaml-cpp
- Opencv 2.4.x/3
- **Tested on Ubuntu 14.04/16.04**

## Design Goals
The mapper linker aims to be an easy interface to build maps from sensor data, or to add/modify an already existing graph with new info. 

## Use cases
- **Pose Graph from data** taken from UAV and/or UGV. The data can be retrieved from .bag file or directly from ros topics
- **Data Exchange** between UAV and UGV, by means of the pose graph itself, or with the "compressed" .bag files
- **Data post-processing** after the gather phase. The tool offers the possibility to fastly retrieve the entire pose graph data from .bag file, or just a single node 

## Pose-Graph structure

```yaml
  id: node_id
  position:
    - px
    - py
    - pz
  orientation:
    - qw
    - qx
    - qy
    - qz
  pose_time: ros::time when pose has been acquired
  has_covariance: true
  covariance:
    - 0
    - ...
  sensor_measurements:
    -
      - camera_topic/image_raw
      - sensor_msgs::Image
      - ros::time when sensor message has been acquired
    -
      - point_cloud_topic
      - sensor_msgs::Pointcloud2
      - ros::time when sensor message has been acquired

```

## Tutorial
- **Building and saving the Pose-Graph**

```cpp
#include <pose_graph/temporalgraph.h>
#include "ROS stuff..."

rosbag::Bag bag;
  bag.open ( input_bag, rosbag::bagmode::Read );

  /* Topics required in your pose_graph */
  std::vector<std::string> topics;
  topics.push_back ( pose_topic );
  topics.push_back ( camera_topic );

  rosbag::View view ( bag, rosbag::TopicQuery ( topics ) );

  TSS pose_graph;
  
  for ( rosbag::View::iterator it = view.begin(); it!=view.end(); it++ )
  {
    rosbag::MessageInstance message_ ( *it );
    std::cout << message_.getTopic() << std::endl;

    if ( message_.getTopic() == pose_topic )
    {
      nav_msgs::Odometry::ConstPtr pose_ptr_ = message_.instantiate<nav_msgs::Odometry>();

      /* Store pose in Eigen format */
      Eigen::Vector3d pose ( pose_ptr_->pose.pose.position.x,
                             pose_ptr_->pose.pose.position.y,
                             pose_ptr_->pose.pose.position.z );
      Eigen::Quaterniond quat ( pose_ptr_->pose.pose.orientation.w,
                                pose_ptr_->pose.pose.orientation.x,
                                pose_ptr_->pose.pose.orientation.y,
                                pose_ptr_->pose.pose.orientation.z );
                          
      /* Save pose covariance is you need */                            
      CovarianceMatrixd cov;
      cov.setZero ( 6,6 );

      /* Create a new node instance */
      TemporalNode node_ ( pose, quat, pose_ptr_->header.stamp );
      temp_.setCovarianceMatrix ( cov );
      
      /* Save image data in SensorReadingID format and then attach it to the current pose_graph node */
      SensorReadingID temp_sens_RD_ ( "sensor_msgs::Image", temp_image_.header.stamp );
      node_.addMeasurement ( "/camera_topic/image_raw", temp_sens_RD_ );
      
      /* Append current node to pose_graph */
      pose_graph.append ( node_ );

    }

    if ( message_.getTopic() == camera_topic )
    {
        /* ... save image data as you need ... */
    }

  }
  
  /* Save the pose_graph in the "pose_graph_package_dir/graphs" directory
  pose_graph.save ( "pose_graph.yaml" );
```

- **Loading an Existing Pose-Graph and storing the "compressed" Bag**

```cpp
#include <pose_graph/temporalgraph.h>
#include "ROS stuff..."

pose_graph.load("pose_graph.yaml");
pose_graph.setBag("input_bag.bag");

pose_graph.extractBag("output_bag.bag");
pose_graph.closeBag();
```

- **Retrieving sensor information from Pose-Graph**

```cpp
#include <pose_graph/temporalgraph.h>
#include "ROS stuff..."

pose_graph.load("pose_graph.yaml");
pose_graph.setBag("input_bag.bag");

// Output directory where to store data from pose_graph
string dir="/home/....../pose_graph_data/";

for(size_t iter=0; i<pose_graph.size(); iter++)
{
    // Extracting image data from the bag
    sensor_msgs::ImageConstPtr im_msg=graph_reduced.getNodeData<sensor_msgs::Image>(iter, "/camera_topic/image_raw");
    if(!im_msg) continue;

    // Extracting image pose data
    Eigen::Isometry3d pose;
    graph_reduced.getNodePose(iter, pose);
    
    // Manage .bag data as you need
    // ...
}
```
- **Add new data or Modify an existing Pose-Graph**


