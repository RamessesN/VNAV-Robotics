#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>

class FramesPublisherNode {
 private:
  ros::NodeHandle nh;
  ros::Time startup_time;

  ros::Timer heartbeat;
  
  tf2_ros::TransformBroadcaster tf_broadcaster;

 public:
  FramesPublisherNode() {
    // NOTE: This method is run once, when the node is launched.
    startup_time = ros::Time::now();
    heartbeat = nh.createTimer(ros::Duration(0.02), &FramesPublisherNode::onPublish, this);
    heartbeat.start();
  }

  void onPublish(const ros::TimerEvent&) {
    // Compute time elapsed in seconds since the node has been started
    double time = (ros::Time::now() - startup_time).toSec();

    // Here we declare two geometry_msgs::TransformStamped objects, which need to be populated
    geometry_msgs::TransformStamped AV1World;
    geometry_msgs::TransformStamped AV2World;
    // NOTE: fields in a ros message default to zero, so we set an identity transform by setting just the w component of the rotation
    AV1World.transform.rotation.w = 1.0;
    AV2World.transform.rotation.w = 1.0;

    // 2. Populate the transforms

    // --- AV1 ---
    AV1World.header.stamp = ros::Time::now();
    AV1World.header.frame_id = "world";
    AV1World.child_frame_id = "av1";
    // path: [cos(t), sin(t), 0]
    AV1World.transform.translation.x = cos(time);
    AV1World.transform.translation.y = sin(time);
    AV1World.transform.translation.z = 0.0;
    // angle: [0, 0, t]
    tf2::Quaternion q1;
    q1.setRPY(0, 0, time);
    AV1World.transform.rotation.x = q1.x();
    AV1World.transform.rotation.y = q1.y();
    AV1World.transform.rotation.z = q1.z();
    AV1World.transform.rotation.w = q1.w();

    // --- AV2 ---
    AV2World.header.stamp = ros::Time::now();
    AV2World.header.frame_id = "world";
    AV2World.child_frame_id = "av2";
    // path: [sin(t), 0, cos(2t)]
    AV2World.transform.translation.x = sin(time);
    AV2World.transform.translation.y = 0.0;
    AV2World.transform.translation.z = cos(2 * time);

    // Publish the transforms using a tf2_ros::TransformBroadcaster
    tf_broadcaster.sendTransform(AV1World);
    tf_broadcaster.sendTransform(AV2World);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "frames_publisher_node");
  FramesPublisherNode node;
  ros::spin();
  return 0;
}