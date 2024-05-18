/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/dlo.h"

#include "rclcpp/rclcpp.hpp"
#include "direct_lidar_odometry/srv/save_pcd.hpp"

// #include <boost/circular_buffer.hpp>
// #include <boost/algorithm/string.hpp>

// #include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/surface/concave_hull.h>
// #include <pcl/surface/convex_hull.h>
// #include <pcl_ros/impl/transforms.hpp>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/transforms.h>
// #include <tf2_ros/transform_broadcaster.h>

// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/Imu.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

// #include <direct_lidar_odometry/save_pcd.h>
// #include <direct_lidar_odometry/save_traj.h>
// #include <nano_gicp/nano_gicp.hpp>

typedef pcl::PointXYZI PointType;

class dlo::MapNode: public rclcpp::Node{

public:

  MapNode();
  ~MapNode();

  static void abort() {
    abort_ = true;
  }

  void start();
  void stop();

private:

  void keyframeCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe);

  bool savePcd(std::shared_ptr<direct_lidar_odometry::srv::SavePCD::Request> req,
               std::shared_ptr<direct_lidar_odometry::srv::SavePCD::Response> res);

  void getParams();


  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;

  rclcpp::Service<direct_lidar_odometry::srv::SavePCD>::SharedPtr save_pcd_srv;

  pcl::PointCloud<PointType>::Ptr dlo_map;
  pcl::VoxelGrid<PointType> voxelgrid;

  rclcpp::Time map_stamp;
  std::string odom_frame;

  bool publish_full_map_;
  double publish_freq_;
  double leaf_size_;

  static std::atomic<bool> abort_;

};
