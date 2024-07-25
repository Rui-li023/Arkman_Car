
#include <ros/ros.h>

// #include <chrono>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mutex>


#include "pcl/filters/voxel_grid.h"
#include "pcl/impl/point_types.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <stdexcept>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>
// #include <thread>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

namespace icp {

using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudXYZIN = pcl::PointCloud<pcl::PointXYZINormal>;

class IcpNode {
public:
  IcpNode(ros::NodeHandle &nh);

  // ~IcpNode() {
  //   if (tf_publisher_thread_->joinable()) {
  //     tf_publisher_thread_->join();
  //   }
  // }

  // void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  //   pcl::fromROSMsg(*msg, *cloud_in_);
  //   if (first_scan_) {
  //     auto pose_msg =
  //         boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();
  //     pose_msg->header = msg->header;
  //     pose_msg->pose.pose = initial_pose_;
  //     initialPoseCallback(pose_msg);
  //     first_scan_ = false;
  //   }
  // }

  // void initialPoseCallback(
  //     const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  //   Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y,
  //                       msg->pose.pose.position.z);
  //   Eigen::Quaterniond q(
  //       msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
  //       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  //   Eigen::Matrix4d initial_guess;
  //   initial_guess.block<3, 3>(0, 0) = q.toRotationMatrix();
  //   initial_guess.block<3, 1>(0, 3) = pos;
  //   initial_guess(3, 3) = 1;

  //   ROS_INFO("Aligning the pointcloud");
  //   Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_in_, initial_guess);
  //   if (!success_) {
  //     map_to_laser = initial_guess;
  //     ROS_ERROR("ICP failed");
  //   }

  //   Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
  //   try {
  //     geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
  //         laser_frame_id_, odom_frame_id_, ros::Time(0), ros::Duration(10));
  //     Eigen::Vector3d t(transform.transform.translation.x,
  //                       transform.transform.translation.y,
  //                       transform.transform.translation.z);
  //     Eigen::Quaterniond q(
  //         transform.transform.rotation.w, transform.transform.rotation.x,
  //         transform.transform.rotation.y, transform.transform.rotation.z);
  //     laser_to_odom.block<3, 3>(0, 0) = q.toRotationMatrix();
  //     laser_to_odom.block<3, 1>(0, 3) = t;
  //   } catch (tf2::TransformException &ex) {
  //     std::lock_guard<std::mutex> lock(mutex_);
  //     ROS_ERROR("%s", ex.what());
  //     is_ready_ = false;
  //     return;
  //   }

  //   Eigen::Matrix4d result =
  //       map_to_laser * laser_to_odom.matrix().cast<double>();
  //   {
  //     std::lock_guard<std::mutex> lock(mutex_);
  //     map_to_odom_.transform.translation.x = result(0, 3);
  //     map_to_odom_.transform.translation.y = result(1, 3);
  //     map_to_odom_.transform.translation.z = result(2, 3);

  //     Eigen::Matrix3d rotation = result.block<3, 3>(0, 0);
  //     Eigen::Quaterniond q(rotation);

  //     map_to_odom_.transform.rotation.w = q.w();
  //     map_to_odom_.transform.rotation.x = q.x();
  //     map_to_odom_.transform.rotation.y = q.y();
  //     map_to_odom_.transform.rotation.z = q.z();
  //     is_ready_ = true;
  //   }
  // }

private:
  // void publishTf() {
  //   ros::Rate rate(100);
  //   while (ros::ok()) {
  //     {
  //       std::lock_guard<std::mutex> lock(mutex_);
  //       if (is_ready_) {
  //         ROS_INFO_STREAM("Publishing tf"
  //                         << map_to_odom_.transform.translation.x << " "
  //                         << map_to_odom_.transform.translation.y << " "
  //                         << map_to_odom_.transform.translation.z);
  //         map_to_odom_.header.stamp = ros::Time::now();
  //         map_to_odom_.header.frame_id = map_frame_id_;
  //         map_to_odom_.child_frame_id = odom_frame_id_;
  //         tf_broadcaster_->sendTransform(map_to_odom_);
  //       }
  //     }
  //     rate.sleep();
  //   }
  // }

  // Eigen::Matrix4d multiAlignSync(pcl::PointCloud<pcl::PointXYZI>::Ptr
  // cloud_in,
  //                                Eigen::Matrix4d &initial_guess) {
  //   Eigen::Matrix4d trans_matrix = initial_guess;
  //   for (int i = 0; i < 10; i++) {
  //     std::lock_guard<std::mutex> lock(mutex_);
  //     ROS_INFO("align cloud %d/%d", i + 1, 10);
  //     Eigen::Matrix4d matrix = alignSync(cloud_in, trans_matrix);
  //     if (!success_) {
  //       ROS_ERROR("alignSync failed");
  //       return trans_matrix;
  //     }
  //     trans_matrix = matrix;
  //   }
  //   return trans_matrix;
  // }

  // Eigen::Matrix4d alignSync(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
  //                           Eigen::Matrix4d &trans_matrix) {
  //   std::lock_guard<std::mutex> lock(mutex_);
  //   icp_rough_.setInputSource(cloud_in);
  //   pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(
  //       new pcl::PointCloud<pcl::PointXYZI>);
  //   icp_rough_.align(*aligned, trans_matrix.cast<float>());
  //   if (!icp_rough_.hasConverged()) {
  //     success_ = false;
  //     return trans_matrix;
  //   }
  //   success_ = true;
  //   icp_refine_.setInputSource(aligned);
  //   icp_refine_.align(*aligned, trans_matrix.cast<float>());
  //   if (!icp_refine_.hasConverged()) {
  //     success_ = false;
  //     return trans_matrix;
  //   }
  //   success_ = true;
  //   return trans_matrix;
  // }

  static PointCloudXYZIN::Ptr
  addNorm(PointCloudXYZI::Ptr cloud);

  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber initial_pose_sub_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_;
  PointCloudXYZIN::Ptr rough_map_;
  PointCloudXYZIN::Ptr refine_map_;
  // std::mutex mutex_;
  bool is_ready_;
  bool first_scan_;
  int rough_iter_, refine_iter_; // 迭代次数

  // geometry_msgs::Pose initial_pose_;

  pcl::VoxelGrid<pcl::PointXYZI> voxel_rough_filter_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_refine_filter_;

  double thresh_, xy_offset_, yaw_offset_, yaw_resolution_;
  // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // tf2_ros::Buffer tf_buffer_;
  std::string map_frame_id_, odom_frame_id_, laser_frame_id_, pcd_path_;
  // tf2_msgs::TFMessage map_to_odom_;
  // std::unique_ptr<std::thread> tf_publisher_thread_;
  // bool success_;
  pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_rough_,
      icp_refine_;
};
} // namespace icp
