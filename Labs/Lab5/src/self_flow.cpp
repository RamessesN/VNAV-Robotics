/*
 * @file self_flow.cpp
 * Feel free to edit any part of this file!
 */
#include <memory>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace std;

/** imageCallback This function is called when a new image is published. */
void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  try {
    // 1. 使用 toCvCopy 替代 toCvShare (更安全，防止内存被提前释放)
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // 转换为灰度
    cv::Mat curr_gray;
    cv::cvtColor(image, curr_gray, cv::COLOR_BGR2GRAY);

    // 静态变量保存上一帧
    static cv::Mat prev_gray;

    // 如果是第一帧，保存并返回
    if (prev_gray.empty()) {
      curr_gray.copyTo(prev_gray);
      return;
    }

    // 2. 计算光流
    cv::Mat flow;
    // 使用经典参数
    cv::calcOpticalFlowFarneback(prev_gray, curr_gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

    // 3. 可视化处理 (类型安全版)
    std::vector<cv::Mat> flow_parts;
    cv::split(flow, flow_parts);
    
    cv::Mat magnitude, angle;
    cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);

    // 使用 vector 而不是数组，更安全
    std::vector<cv::Mat> hsv_planes(3);

    // Channel 0: Hue (色调) -> 显式缩放并转为 8位无符号 (CV_8UC1)
    angle.convertTo(hsv_planes[0], CV_8UC1, 0.5);

    // Channel 1: Saturation (饱和度) -> 显式构造 8位矩阵，且填满 255
    // 之前出错可能就是这里自动变成了 int 类型
    hsv_planes[1] = cv::Mat(angle.size(), CV_8UC1, cv::Scalar(255));

    // Channel 2: Value (亮度) -> 归一化后再转为 8位
    cv::Mat v_norm;
    cv::normalize(magnitude, v_norm, 0, 255, cv::NORM_MINMAX);
    v_norm.convertTo(hsv_planes[2], CV_8UC1);

    // 4. 合并与显示
    cv::Mat hsv;
    cv::merge(hsv_planes, hsv); // 现在三个通道绝对都是 CV_8UC1，不会崩溃

    cv::Mat result_bgr;
    cv::cvtColor(hsv, result_bgr, cv::COLOR_HSV2BGR);

    cv::imshow("view", result_bgr);
    cv::waitKey(1);

    // 5. 更新上一帧
    curr_gray.copyTo(prev_gray);

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

/**
 * @function main
 * @brief Main function
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "optical_flow");
  ros::NodeHandle local_nh("~");

  cv::namedWindow("view", cv::WINDOW_NORMAL);
  image_transport::ImageTransport it(local_nh);
  image_transport::Subscriber sub = it.subscribe("/images_topic", 100, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
  return EXIT_SUCCESS;
}
