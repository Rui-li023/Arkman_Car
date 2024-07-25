#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
class DynamicTransformPublisher {
public:
  DynamicTransformPublisher() {
    ros::NodeHandle nh;
    pose_subscriber_ = nh.subscribe(
        "/initialpose", 1, &DynamicTransformPublisher::poseCallback, this);
    received_first_pose_ = false;
    current_transform_.header.frame_id = "map";
    current_transform_.child_frame_id = "camera_init";
  }
  void
  poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    current_transform_.header.stamp = ros::Time::now();
    current_transform_.transform.translation.x = msg->pose.pose.position.x;
    current_transform_.transform.translation.y = msg->pose.pose.position.y;
    current_transform_.transform.translation.z = msg->pose.pose.position.z;
    current_transform_.transform.rotation = msg->pose.pose.orientation;
    received_first_pose_ = true;
  }
  void publishTransform() {
    ros::Rate rate(10.0); // 10 Hz
    while (ros::ok()) {
      if (received_first_pose_) {
        current_transform_.header.stamp = ros::Time::now();
        broadcaster_.sendTransform(current_transform_);
      }
      rate.sleep();
      ros::spinOnce();
    }
  }
private:
  ros::Subscriber pose_subscriber_;
  tf2_ros::TransformBroadcaster broadcaster_;
  geometry_msgs::TransformStamped current_transform_;
  bool received_first_pose_;
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_transform_publisher");
  DynamicTransformPublisher publisher;
  publisher.publishTransform();
  return 0;
}