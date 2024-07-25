#include "icp_relocalization/icp_relocalization.h"
#include <filesystem>
#include <boost/shared_ptr.hpp>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>

namespace icp {
IcpNode::IcpNode(ros::NodeHandle &nh)
    : nh_(nh), rough_iter_(10), refine_iter_(5), first_scan_(true) {
  is_ready_ = false;
  cloud_in_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  double rough_leaf_size, refine_leaf_size;

  nh_.param("rough_leaf_size", rough_leaf_size, 0.4);
  nh_.param("refine_leaf_size", refine_leaf_size, 0.1);
  nh_.param("pcd_path", pcd_path_, std::string(""));
  nh_.param("map_frame_id", map_frame_id_, std::string("map"));
  nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  nh_.param("laser_frame_id", laser_frame_id_, std::string("laser"));
  nh_.param("thresh", thresh_, 0.15);
  nh_.param("xy_offset", xy_offset_, 0.2);
  nh_.param("yaw_offset", yaw_offset_, 30.0);

  voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size,
                                  rough_leaf_size);
  voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size,
                                   refine_leaf_size);

  if (!std::filesystem::exists(pcd_path_)) {
    ROS_ERROR("Invalid pcd path: %s", pcd_path_.c_str());
    throw std::runtime_error("Invalid pcd path");
  }

  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  reader.read(pcd_path_, *cloud);
  voxel_refine_filter_.setInputCloud(cloud);
  voxel_refine_filter_.filter(*cloud);

  refine_map_ = addNorm(cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);

  pcl::copyPointCloud(*refine_map_, *point_rough);
  voxel_rough_filter_.setInputCloud(point_rough);
  voxel_rough_filter_.filter(*filterd_point_rough);
  rough_map_ = addNorm(filterd_point_rough);

  icp_rough_.setMaximumIterations(rough_iter_);
  icp_rough_.setInputTarget(rough_map_);

  icp_refine_.setMaximumIterations(refine_iter_);
  icp_refine_.setInputTarget(refine_map_);

  ROS_INFO("pcd point size: %ld, %ld", refine_map_->size(), rough_map_->size());

  // yaw_offset_ *= M_PI / 180.0;
  // nh_.param("yaw_resolution", yaw_resolution_, 10.0);
  // yaw_resolution_ *= M_PI / 180.0;

  // std::vector<double> initial_pose_vec;
  // nh_.param("initial_pose", initial_pose_vec,
  //           std::vector<double>{0, 0, 0, 0, 0, 0});
  // try {
  //   initial_pose_.position.x = initial_pose_vec.at(0);
  //   initial_pose_.position.y = initial_pose_vec.at(1);
  //   initial_pose_.position.z = initial_pose_vec.at(2);
  //   tf2::Quaternion q;
  //   q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4),
  //            initial_pose_vec.at(5));
  //   initial_pose_.orientation.x = q.x();
  //   initial_pose_.orientation.y = q.y();
  //   initial_pose_.orientation.z = q.z();
  //   initial_pose_.orientation.w = q.w();
  // } catch (const std::out_of_range &ex) {
  //   ROS_ERROR("initial_pose is not a vector with 6 elements, what():%s",
  //             ex.what());
  // }

  // std::string pointcloud_topic;
  // nh_.param("pointcloud_topic", pointcloud_topic,
  //           std::string("/livox/lidar/pointcloud"));
  // ROS_INFO("pointcloud_topic: %s", pointcloud_topic.c_str());
  // pointcloud_sub_ =
  //     nh_.subscribe(pointcloud_topic, 10, &IcpNode::pointcloudCallback,
  //     this);
  // initial_pose_sub_ =
  //     nh_.subscribe("/initialpose", 10, &IcpNode::initialPoseCallback, this);

  // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  // tf_publisher_thread_ =
  //     std::make_unique<std::thread>(&IcpNode::publishTf, this);

  // ROS_INFO("icp_registration initialized");
};


PointCloudXYZIN::Ptr
IcpNode::addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(15);
  normalEstimator.compute(*normals);
  PointCloudXYZIN::Ptr out(new PointCloudXYZIN);
  pcl::concatenateFields(*cloud, *normals, *out);
  return out;
}

} // namespace icp

int main(int argc, char **argv) {
  ros::init(argc, argv, "icp_registration");
  ros::NodeHandle nh;
  icp::IcpNode icp_node(nh);
  ros::spin();
  return 0;
}