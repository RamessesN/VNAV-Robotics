#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <cmath>

#define PI M_PI

#include <eigen3/Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>

class controllerNode{
  ros::NodeHandle nh;

  // PART 1: Declare ROS callback handlers
  ros::Subscriber des_state_sub, cur_state_sub;
  ros::Publisher propeller_speeds_pub;
  ros::Timer control_timer;

  // Controller parameters
  double kx, kv, kr, komega;

  // Physical constants (we will set them below)
  double m;              // mass of the UAV
  double g;              // gravity acceleration
  double d;              // distance from the center of propellers to the c.o.m.
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient

  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::MatrixXd F2W;   // Wrench-rotor speeds map

  // Controller internals
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val > 0 ? sqrt(val) : -sqrt(-val);
  }

public:
  controllerNode():e3(0,0,1),F2W(4,4),hz(1000.0){
      // PART 2: Initialize ROS callback handlers
      xd = Eigen::Vector3d::Zero();
      vd = Eigen::Vector3d::Zero();
      ad = Eigen::Vector3d::Zero();
      yawd = 0.0;
      kx, kv, kr, komega = 0, 0, 0, 0;

      des_state_sub = nh.subscribe("desired_state", 1, &  controllerNode::onDesiredState, this);
      cur_state_sub = nh.subscribe("current_state", 1, &controllerNode::onCurrentState, this);
      propeller_speeds_pub = nh.advertise<mav_msgs::Actuators>("/rotor_speed_cmds", 1);
      control_timer = nh.createTimer(ros::Duration(1.0/hz), &controllerNode::controlLoop, this);

      // PART 6: Tune your gains!
      nh.getParam("kx", kx);
      nh.getParam("kv", kv);
      nh.getParam("kr", kr);
      nh.getParam("komega", komega);
      ROS_INFO("Gain values:\nkx: %f \nkv: %f \nkr: %f \nkomega: %f\n", kx, kv, kr, komega);

      // Initialize constants
      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;

      // F2W matrix
      double d_by_sqrt2 = d/std::sqrt(2.0);
      F2W <<
          cf,            cf,            cf,            cf,
          cf*d_by_sqrt2, cf*d_by_sqrt2,-cf*d_by_sqrt2,-cf*d_by_sqrt2,
         -cf*d_by_sqrt2, cf*d_by_sqrt2, cf*d_by_sqrt2,-cf*d_by_sqrt2,
          cd,           -cd,            cd,           -cd;
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){
      //  PART 3: Objective - fill in xd, vd, ad, yawd
      xd << des_state.transforms[0].translation.x, 
            des_state.transforms[0].translation.y, 
            des_state.transforms[0].translation.z;
            
      vd << des_state.velocities[0].linear.x, 
            des_state.velocities[0].linear.y, 
            des_state.velocities[0].linear.z;
            
      ad << des_state.accelerations[0].linear.x, 
            des_state.accelerations[0].linear.y, 
            des_state.accelerations[0].linear.z;

      tf2::Quaternion quat;
      tf2::fromMsg(des_state.transforms[0].rotation, quat);
      yawd = tf2::getYaw(quat);
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      // PART 4: Objective - fill in x, v, R and omega
      // Position
      x << cur_state.pose.pose.position.x, 
          cur_state.pose.pose.position.y, 
          cur_state.pose.pose.position.z;

      // Velocity
      v << cur_state.twist.twist.linear.x, 
          cur_state.twist.twist.linear.y, 
          cur_state.twist.twist.linear.z;

      // Orientation
      tf2::Quaternion quat;
      tf2::fromMsg(cur_state.pose.pose.orientation, quat);
      Eigen::Quaterniond eigen_quat(quat.w(), quat.x(), quat.y(), quat.z());
      eigen_quat.normalize();
      R = eigen_quat.toRotationMatrix();

      // Angular velocity
      Eigen::Vector3d omega_world;
      omega_world << cur_state.twist.twist.angular.x, 
                    cur_state.twist.twist.angular.y, 
                    cur_state.twist.twist.angular.z;

      omega = R.transpose() * omega_world;
  }

  void controlLoop(const ros::TimerEvent& t){
    Eigen::Vector3d ex, ev, er, eomega;
    // PART 5: Objective - Implement the controller!
    ex = x - xd; // position error
    ev = v - vd; // velocity error

    // Rd matrix
    Eigen::Vector3d F_des = -kx*ex - kv*ev + m*g*e3 + m*ad;
    Eigen::Vector3d b3d = F_des.normalized();
    Eigen::Vector3d b1d_desired(cos(yawd), sin(yawd), 0);

    Eigen::Vector3d b2d = (b3d.cross(b1d_desired)).normalized();
    Eigen::Vector3d b1d = (b2d.cross(b3d)).normalized();

    Eigen::Matrix3d Rd;
    Rd.col(0) = b1d;
    Rd.col(1) = b2d;
    Rd.col(2) = b3d;
    
    er = 0.5 * Vee(Rd.transpose() * R - R.transpose() * Rd); // Orientation error
    eomega = omega; // Rotation-rate error
    
    // Desired wrench
    double f = (-kx * ex + -kv * ev + m * g * e3 + m * ad).dot(R * e3);
    Eigen::Vector3d M = -kr * er - komega * eomega + omega.cross(J * omega);

    // Recover the rotor speeds from the wrench
    Eigen::Vector4d W;
    W << f, M.x(), M.y(), M.z();
    Eigen::Vector4d omega_sq = F2W.colPivHouseholderQr().solve(W);

    Eigen::Vector4d rotor_speeds;
    for (int i = 0; i < 4; i++) {
        rotor_speeds(i) = signed_sqrt(omega_sq[i]);
    }

    // Populate and publish the control message
    mav_msgs::Actuators control_msg;
    control_msg.angular_velocities.clear();
    for (int i = 0; i < 4; i++) {
        control_msg.angular_velocities.push_back(rotor_speeds(i));
    }
    propeller_speeds_pub.publish(control_msg);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  controllerNode n;
  ros::spin();
}