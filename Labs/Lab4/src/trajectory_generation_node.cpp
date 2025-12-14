//
// Created by stewart on 9/20/20.
//

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>

class WaypointFollower {
  [[maybe_unused]] ros::Subscriber currentStateSub;
  [[maybe_unused]] ros::Subscriber poseArraySub;
  ros::Publisher desiredStatePub;

  // Current state
  Eigen::Vector3d x;  // current position of the UAV's c.o.m. in the world frame

  ros::Timer desiredStateTimer;

  ros::Time trajectoryStartTime;
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory yaw_trajectory;

  void onCurrentState(nav_msgs::Odometry const& cur_state) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.1 |  16.485 - Fall 2021  - Lab 4 coding assignment (5 pts)
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    //  Populate the variable x, which encodes the current world position of the UAV
    // ~~~~ begin solution
    
    // [修改部分 1.1] 将 ROS 消息转换为 Eigen 向量
    tf::pointMsgToEigen(cur_state.pose.pose.position, x);

    // ~~~~ end solution
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void generateOptimizedTrajectory(geometry_msgs::PoseArray const& poseArray) {
    if (poseArray.poses.size() < 1) {
      ROS_ERROR("Must have at least one pose to generate trajectory!");
      trajectory.clear();
      yaw_trajectory.clear();
      return;
    }

    if (!trajectory.empty()) return;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.2 |  16.485 - Fall 2021  - Lab 4 coding assignment (35 pts)
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    const int D = 3;  // dimension of each vertex in the trajectory
    mav_trajectory_generation::Vertex start_position(D), end_position(D);
    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex start_yaw(1), end_yaw(1);
    mav_trajectory_generation::Vertex::Vector yaw_vertices;

    // ============================================
    // Convert the pose array to a list of vertices
    // ============================================

    // Start from the current position and zero orientation
    using namespace mav_trajectory_generation::derivative_order;
    start_position.makeStartOrEnd(x, SNAP);
    vertices.push_back(start_position);
    
    start_yaw.addConstraint(ORIENTATION, 0); // 假设起始偏航角为0，或者可以改为当前偏航角
    yaw_vertices.push_back(start_yaw);

    double last_yaw = 0;
    
    for (auto i = 0; i < poseArray.poses.size(); ++i) {
      // ~~~~ begin solution
      
      // [修改部分 1.2]
      
      // --- 1. 处理位置顶点 ---
      Eigen::Vector3d pos_eigen;
      tf::pointMsgToEigen(poseArray.poses[i].position, pos_eigen);

      mav_trajectory_generation::Vertex pos_vertex(D);
      
      // 如果是最后一个点，必须设为终点 (makeStartOrEnd)，速度加速度强制为0
      if (i == poseArray.poses.size() - 1) {
          pos_vertex.makeStartOrEnd(pos_eigen, SNAP);
      } else {
          // 如果是中间点，只添加位置约束，允许以一定速度通过
          pos_vertex.addConstraint(POSITION, pos_eigen);
      }
      vertices.push_back(pos_vertex);


      // --- 2. 处理偏航角顶点 ---
      double current_yaw = tf::getYaw(poseArray.poses[i].orientation);

      // 关键步骤：角度解缠 (Unwrapping)，防止由 350度->10度 时发生反转
      while (current_yaw - last_yaw > M_PI) current_yaw -= 2 * M_PI;
      while (current_yaw - last_yaw < -M_PI) current_yaw += 2 * M_PI;

      mav_trajectory_generation::Vertex yaw_vertex(1);
      yaw_vertex.addConstraint(ORIENTATION, current_yaw);
      yaw_vertices.push_back(yaw_vertex);

      last_yaw = current_yaw; // 更新 last_yaw

      // ~~~~ end solution
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // ============================================================
    // Estimate the time to complete each segment of the trajectory
    // ============================================================

    // HINT: play with these segment times and see if you can finish
    // the race course faster!
    std::vector<double> segment_times;
    const double v_max = 15.0;
    const double a_max = 10.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);
    for(int i = 0; i < segment_times.size(); i++) {
      segment_times[i] *= 0.6;
    }

    // =====================================================
    // Solve for the optimized trajectory (linear optimizer)
    // =====================================================
    // Position
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimization<N> opt(D);
    opt.setupFromVertices(vertices, segment_times, mav_trajectory_generation::derivative_order::SNAP);
    opt.solveLinear();

    // Yaw
    mav_trajectory_generation::PolynomialOptimization<N> yaw_opt(1);
    yaw_opt.setupFromVertices(yaw_vertices, segment_times, mav_trajectory_generation::derivative_order::SNAP);
    yaw_opt.solveLinear();

    // ============================
    // Get the optimized trajectory
    // ============================
    mav_trajectory_generation::Segment::Vector segments;
    opt.getTrajectory(&trajectory);
    yaw_opt.getTrajectory(&yaw_trajectory);
    trajectoryStartTime = ros::Time::now();

    ROS_INFO("Generated optimizes trajectory from %lu waypoints", vertices.size());
  }

  void publishDesiredState(ros::TimerEvent const& ev) {
    if (trajectory.empty()) {
        // 如果还没有轨迹，发布当前位置作为期望位置（悬停）
        trajectory_msgs::MultiDOFJointTrajectoryPoint hover_point;
        
        hover_point.time_from_start = ros::Duration(0.0);

        // 1. 设置位置为当前位置 x (由 onCurrentState 更新)
        geometry_msgs::Transform transform;
        tf::vectorEigenToMsg(x, transform.translation);
        
        // 保持当前的 Yaw (或者设为0，视情况而定，这里保持0比较安全)
        transform.rotation = tf::createQuaternionMsgFromYaw(0); 
        hover_point.transforms.push_back(transform);

        // 2. 设置速度为 0
        geometry_msgs::Twist velocity;
        velocity.linear.x = 0; velocity.linear.y = 0; velocity.linear.z = 0;
        velocity.angular.x = 0; velocity.angular.y = 0; velocity.angular.z = 0;
        hover_point.velocities.push_back(velocity);

        // 3. 设置加速度为 0
        geometry_msgs::Twist accel;
        accel.linear.x = 0; accel.linear.y = 0; accel.linear.z = 0;
        accel.angular.x = 0; accel.angular.y = 0; accel.angular.z = 0;
        hover_point.accelerations.push_back(accel);

        // 发布悬停指令
        desiredStatePub.publish(hover_point);
        return;
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 1.3 |  16.485 - Fall 2021  - Lab 4 coding assignment (15 pts)
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    // ~~~~ begin solution
    
    // [修改部分 1.3]

    trajectory_msgs::MultiDOFJointTrajectoryPoint next_point;
    
    // 1. 计算从轨迹开始到现在的时间
    ros::Duration time_from_start = ros::Time::now() - trajectoryStartTime;
    next_point.time_from_start = time_from_start; 

    double sampling_time = time_from_start.toSec(); 

    // 2. 防止时间超出轨迹总时长
    if (sampling_time > trajectory.getMaxTime())
      sampling_time = trajectory.getMaxTime();

    // Getting the desired state based on the optimized trajectory we found.
    using namespace mav_trajectory_generation::derivative_order;
    Eigen::Vector3d des_position = trajectory.evaluate(sampling_time, POSITION);
    Eigen::Vector3d des_velocity = trajectory.evaluate(sampling_time, VELOCITY);
    Eigen::Vector3d des_accel = trajectory.evaluate(sampling_time, ACCELERATION);
    Eigen::VectorXd des_orientation = yaw_trajectory.evaluate(sampling_time, ORIENTATION);
    
    // 调试打印 (可选)
    // ROS_INFO_THROTTLE(1.0, "Traversed %f percent of the trajectory.",
    //                   sampling_time / trajectory.getMaxTime() * 100);

    // Populate next_point

    // A. 填充 Transform (位置 + 姿态)
    geometry_msgs::Transform transform;
    tf::vectorEigenToMsg(des_position, transform.translation);
    // des_orientation 是 VectorXd，取第0个元素作为 yaw，并转为四元数
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(des_orientation(0)), transform.rotation);
    next_point.transforms.push_back(transform);

    // B. 填充 Velocity (线速度 + 角速度)
    geometry_msgs::Twist velocity;
    tf::vectorEigenToMsg(des_velocity, velocity.linear);
    velocity.angular.x = 0; 
    velocity.angular.y = 0; 
    velocity.angular.z = 0; // 简化处理，通常不需要前馈yaw rate
    next_point.velocities.push_back(velocity);

    // C. 填充 Acceleration (线加速度)
    geometry_msgs::Twist accel;
    tf::vectorEigenToMsg(des_accel, accel.linear);
    accel.angular.x = 0;
    accel.angular.y = 0;
    accel.angular.z = 0;
    next_point.accelerations.push_back(accel);

    // ~~~~ end solution
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    desiredStatePub.publish(next_point);
  }

public:
  explicit WaypointFollower(ros::NodeHandle& nh) {
    currentStateSub = nh.subscribe(
        "/current_state", 1, &WaypointFollower::onCurrentState, this);
    poseArraySub = nh.subscribe("/desired_traj_vertices",
                                1,
                                &WaypointFollower::generateOptimizedTrajectory,
                                this);
    desiredStatePub =
        nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/desired_state", 1);
    desiredStateTimer = nh.createTimer(
        ros::Rate(100), &WaypointFollower::publishDesiredState, this); // 建议提高频率到 50-100Hz
    desiredStateTimer.start();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generation_node");
  ros::NodeHandle nh;

  WaypointFollower waypointFollower(nh);

  ros::spin();
  return 0;
}