#include "lk_feature_tracker.h"

#include <numeric>
#include <vector>

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>

using namespace cv;
using namespace cv::xfeatures2d;

/**
   LK feature tracker Constructor.
*/
LKFeatureTracker::LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
}

LKFeatureTracker::~LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::destroyWindow(window_name_);
}

/** This is the main tracking function, given two images, it detects,
 * describes and matches features.
 * We will be modifying this function incrementally to plot different figures
 * and compute different statistics.
 @param[in] frame Current image frame
*/
void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 7 | Feature Tracking: Lucas-Kanade Tracker
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //
  //   1. Using OpenCV’s documentation and the C++ API for the LK tracker, track
  //   features for the video sequences we provided you by using the Harris
  //   corner detector (like here). Show the feature tracks at a given frame
  //   extracted when using the Harris corners (consider using the 'show'
  //   function below)
  //
  //   Hint 1: take a look at cv::goodFeaturesToTrack and cv::calcOpticalFlowPyrLK
  //
  //   2. Add an extra entry in the table you made previously for the Harris +
  //   LK tracker
  //
  //   Note: LKFeatureTracker does not inherit from the base tracker like other
  //   feature trackers, so you need to also implement the statistics gathering
  //   code right here.
  //
  // ~~~~ begin solution
  // 1. 初始化当前帧的关键点容器
  std::vector<cv::Point2f> curr_points;
  std::vector<uchar> status;
  std::vector<float> err;

  // 2. 逻辑分支：初始化 vs 追踪
  if (prev_corners_.empty()) {
    // Case A: 没有上一帧的点（第一帧或跟丢了），检测新特征点
    // 使用 Shi-Tomasi 角点检测 (Good Features to Track)
    // 参数：输入图, 输出点, 最大点数, 质量等级, 最小距离
    cv::goodFeaturesToTrack(frame, prev_corners_, 1000, 0.01, 10);
    
    // 初始化上一帧图像
    frame.copyTo(prev_frame_);
  } else {
    // Case B: 有上一帧的点，使用光流追踪到当前帧
    // 参数：前一帧, 当前帧, 前一帧点, 当前帧点(输出), 状态, 误差
    cv::calcOpticalFlowPyrLK(prev_frame_, frame, prev_corners_, curr_points, 
                             status, err, cv::Size(21, 21), 3);

    // 3. 过滤跟丢的点 (status == 0) 和 越界的点
    std::vector<cv::Point2f> good_prev;
    std::vector<cv::Point2f> good_curr;

    for (size_t i = 0; i < status.size(); i++) {
        // 如果追踪成功 (status=1) 且点在图像内
        if (status[i]) {
            // 可选：RANSAC 几何校验 (Lab 要求里有 inlierMaskComputation)
            // 但如果为了速度，这里通常先不过滤，或者只做简单的边界检查
            good_prev.push_back(prev_corners_[i]);
            good_curr.push_back(curr_points[i]);
        }
    }

    // 4. 计算并打印统计信息 (Deliverable 7 要求添加 entry)
    // 我们可以调用现成的 inlierMaskComputation 来计算 RANSAC Inliers
    std::vector<uchar> inlier_mask;
    inlierMaskComputation(good_prev, good_curr, &inlier_mask);
    
    int inlier_count = 0;
    for(auto val : inlier_mask) if(val) inlier_count++;

    // 打印日志 (为了填表)
    // 注意：LK 是稀疏光流，这里 Matches 数通常指追踪成功的点数
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 10 == 0) { // 防止刷屏，每10帧打一次
        ROS_INFO_STREAM("LK Stats: Matches: " << good_curr.size() 
                        << " Inliers: " << inlier_count 
                        << " Ratio: " << (double)inlier_count / good_curr.size());
    }

    // 5. 可视化
    // 需要把单通道灰度图转回彩色以便画彩色线
    cv::Mat img_viz;
    cv::cvtColor(frame, img_viz, cv::COLOR_GRAY2BGR);
    show(img_viz, good_prev, good_curr);

    // 6. 补充机制 (Replenishment)
    // 如果点太少了（例如 < 800 个），重新检测补充一些，保持追踪稳定性
    if (good_curr.size() < 800) {
        std::vector<cv::Point2f> new_points;
        cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1); 
        // 在没有点的地方画白色，作为 mask
        mask.setTo(cv::Scalar(255));
        for(auto& p : good_curr) cv::circle(mask, p, 10, cv::Scalar(0), -1);

        cv::goodFeaturesToTrack(frame, new_points, 1000 - good_curr.size(), 0.01, 10, mask);
        good_curr.insert(good_curr.end(), new_points.begin(), new_points.end());
    }

    // 7. 更新状态
    prev_corners_ = good_curr;
    frame.copyTo(prev_frame_);
  }
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 7
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}

/** Display image with tracked features from prev to curr on the image
 * corresponding to 'frame'
 * @param[in] frame The current image frame, to draw the feature track on
 * @param[in] prev The previous set of keypoints
 * @param[in] curr The set of keypoints for the current frame
 */
void LKFeatureTracker::show(const cv::Mat& frame, std::vector<cv::Point2f>& prev,
                            std::vector<cv::Point2f>& curr) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ~~~~ begin solution
  cv::Mat viz_img = frame.clone();

  for (size_t i = 0; i < curr.size(); i++) {
      cv::line(viz_img, prev[i], curr[i], cv::Scalar(0, 255, 0), 2);
      cv::circle(viz_img, curr[i], 3, cv::Scalar(0, 0, 255), -1);
  }

  cv::imshow(window_name_, viz_img);
  cv::waitKey(1);
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

/** Compute Inlier Mask out of the given matched keypoints.
 @param[in] pts1 List of keypoints detected on the first image.
 @param[in] pts2 List of keypoints detected on the second image.
 @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
*/
void LKFeatureTracker::inlierMaskComputation(const std::vector<cv::Point2f>& pts1,
                                             const std::vector<cv::Point2f>& pts2,
                                             std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, CV_FM_RANSAC,
                       max_dist_from_epi_line_in_px, confidence_prob,
                       *inlier_mask);
  } catch(...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}
