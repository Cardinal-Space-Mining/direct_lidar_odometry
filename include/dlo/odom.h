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

#include <deque>
#include <utility>
#include <chrono>

#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nano_gicp/nano_gicp.hpp>


typedef pcl::PointXYZI PointType;

class dlo::OdomNode: public rclcpp::Node{

public:

  OdomNode();
  ~OdomNode();

  static void abort() {
    abort_ = true;
  }

  void start();
  void stop();

private:

  // void abortTimerCB(const ros::TimerEvent& e);
  void icpCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc);
  void imuCB(const sensor_msgs::msg::Imu::SharedPtr imu);
  // bool saveTrajectory(direct_lidar_odometry::save_traj::Request& req,
  //                     direct_lidar_odometry::save_traj::Response& res);

  void getParams();

  void publishToROS();
  void publishPose();
  void publishTransform();
  void publishKeyframe();
  void publishFilteredScan();

  void preprocessPoints();
  void initializeInputTarget();
  void setInputSources();

  void initializeDLO();
  void gravityAlign();

  void getNextPose();
  void integrateIMU();

  void propagateS2S(Eigen::Matrix4f T);
  void propagateS2M();

  void setAdaptiveParams();

  void computeMetrics();
  void computeSpaciousness();

  void transformCurrentScan();
  void updateKeyframes();
  void computeConvexHull();
  void computeConcaveHull();
  void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);
  void getSubmapKeyframes();

  void debug();

  double first_imu_time;

  tf2_ros::Buffer tfbuffer;
  tf2_ros::TransformListener tflistener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr icp_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kf_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub;

  Eigen::Vector3f origin;
  std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud<PointType>::Ptr>> keyframes;
  std::vector<std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>> keyframe_normals;

  std::atomic<bool> dlo_initialized;
  std::atomic<bool> imu_calibrated;

  std::string odom_frame;
  std::string child_frame;
  bool transform_to_child_;

  pcl::PointCloud<PointType>::Ptr export_scan;
  pcl::PointCloud<PointType>::Ptr current_scan;
  pcl::PointCloud<PointType>::Ptr current_scan_t;
  bool export_filtered;

  pcl::PointCloud<PointType>::Ptr keyframes_cloud;
  pcl::PointCloud<PointType>::Ptr keyframe_cloud;
  int num_keyframes;

  pcl::ConvexHull<PointType> convex_hull;
  pcl::ConcaveHull<PointType> concave_hull;
  std::vector<int> keyframe_convex;
  std::vector<int> keyframe_concave;

  pcl::PointCloud<PointType>::Ptr submap_cloud;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> submap_normals;

  std::vector<int> submap_kf_idx_curr;
  std::vector<int> submap_kf_idx_prev;
  std::atomic<bool> submap_hasChanged;

  pcl::PointCloud<PointType>::Ptr source_cloud;
  pcl::PointCloud<PointType>::Ptr target_cloud;

  rclcpp::Time scan_stamp;

  double curr_frame_stamp;
  double prev_frame_stamp;
  std::vector<double> comp_times;

  nano_gicp::NanoGICP<PointType, PointType> gicp_s2s;
  nano_gicp::NanoGICP<PointType, PointType> gicp;

  pcl::CropBox<PointType> crop;
  pcl::VoxelGrid<PointType> vf_scan;
  pcl::VoxelGrid<PointType> vf_submap;

  nav_msgs::msg::Odometry odom;
  nav_msgs::msg::Odometry kf;

  geometry_msgs::msg::PoseStamped pose_ros;

  std::deque<std::pair<std::chrono::system_clock::time_point, Eigen::Vector<float, 6>>> pose_variance_samples;
  std::chrono::duration<double> variance_sample_history;
  bool calculate_odom_variance;

  Eigen::Matrix4f T;
  Eigen::Matrix4f T_s2s, T_s2s_prev;
  Eigen::Quaternionf q_final;

  Eigen::Vector3f pose_s2s;
  Eigen::Matrix3f rotSO3_s2s;
  Eigen::Quaternionf rotq_s2s;

  Eigen::Vector3f pose;
  Eigen::Matrix3f rotSO3;
  Eigen::Quaternionf rotq;

  Eigen::Matrix4f imu_SE3;

  struct XYZd {
    double x;
    double y;
    double z;
  };

  struct ImuBias {
    XYZd gyro;
    XYZd accel;
  };

  ImuBias imu_bias;

  struct ImuMeas {
    double stamp;
    XYZd ang_vel;
    XYZd lin_accel;
  };

  ImuMeas imu_meas;

  boost::circular_buffer<ImuMeas> imu_buffer;

  static bool comparatorImu(ImuMeas m1, ImuMeas m2) {
    return (m1.stamp < m2.stamp);
  };

  struct Metrics {
    std::vector<float> spaciousness;
  };

  Metrics metrics;

  static std::atomic<bool> abort_;
  std::atomic<bool> stop_publish_thread;
  std::atomic<bool> stop_publish_keyframe_thread;
  std::atomic<bool> stop_metrics_thread;
  std::atomic<bool> stop_debug_thread;

  std::thread publish_thread;
  std::thread publish_keyframe_thread;
  std::thread metrics_thread;
  std::thread debug_thread;

  std::mutex mtx_imu;

  std::string cpu_type;
  std::vector<double> cpu_percents;
  clock_t lastCPU, lastSysCPU, lastUserCPU;
  int numProcessors;

  // Parameters
  std::string version_;

  bool gravity_align_;

  double keyframe_thresh_dist_;
  double keyframe_thresh_rot_;

  int submap_knn_;
  int submap_kcv_;
  int submap_kcc_;
  double submap_concave_alpha_;

  bool initial_pose_use_;
  Eigen::Vector3f initial_position_;
  Eigen::Quaternionf initial_orientation_;

  bool crop_use_;
  // double crop_size_;
  Eigen::Vector4f crop_min_, crop_max_;

  bool vf_scan_use_;
  double vf_scan_res_;

  bool vf_submap_use_;
  double vf_submap_res_;

  bool adaptive_params_use_;

  bool imu_use_;
  int imu_calib_time_;
  int imu_buffer_size_;

  int gicp_min_num_points_;

  int gicps2s_k_correspondences_;
  double gicps2s_max_corr_dist_;
  int gicps2s_max_iter_;
  double gicps2s_transformation_ep_;
  double gicps2s_euclidean_fitness_ep_;
  int gicps2s_ransac_iter_;
  double gicps2s_ransac_inlier_thresh_;

  int gicps2m_k_correspondences_;
  double gicps2m_max_corr_dist_;
  int gicps2m_max_iter_;
  double gicps2m_transformation_ep_;
  double gicps2m_euclidean_fitness_ep_;
  int gicps2m_ransac_iter_;
  double gicps2m_ransac_inlier_thresh_;

};
