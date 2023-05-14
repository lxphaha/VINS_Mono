
#pragma once

#include <csignal>
#include <cstdio>
#include <execinfo.h>
#include <iostream>
#include <queue>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
// #include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker {
public:
  FeatureTracker();

  void readImage(const cv::Mat &_img, double _cur_time);

  // 特征点均匀化
  void setMask();

  void addPoints();

  bool updateID(unsigned int i);

  void readIntrinsicParameter(const string &calib_file);

  void showUndistortion(const string &name);

  void rejectWithF();

  void undistortedPoints();

  cv::Mat mask; // 图像掩码
  cv::Mat fisheye_mask;
  cv::Mat prev_img, cur_img, forw_img;             // 上上次/上一帧图像数据/当前图像数据
  vector<cv::Point2f> n_pts;                       // 每帧中新提取的特征点
  vector<cv::Point2f> prev_pts, cur_pts, forw_pts; // 对应图像特征点
  vector<cv::Point2f> prev_un_pts, cur_un_pts;     // 归一化相机坐标下的坐标
  vector<cv::Point2f> pts_velocity;                // 当前帧相对前一帧特征点沿x,y方向的像素移动速度
  vector<int> ids;                                 // 能够被追踪到特征点id
  vector<int> track_cnt;                           // 特征点追踪次数
  map<int, cv::Point2f> cur_un_pts_map;            // 构建id与归一化坐标的id，见undistortedPoints()
  map<int, cv::Point2f> prev_un_pts_map;
  camodocal::CameraPtr m_camera; // 相机模型
  double cur_time;
  double prev_time;

  static int n_id; // static 静态变量 全局遍历的每一特征点附上id
};
