/*
 * @Date: 2023-03-27 16:03:03
 * @LastEditors: lxp
 * @LastEditTime: 2023-03-29 18:50:30
 * @FilePath: /catkin_vinsmono/src/VINS_Mono/feature_tracker/src/parameters.cpp
 */
#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
//是否均衡化
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;//虚拟焦距（文件定义）
int FISHEYE;
bool PUB_THIS_FRAME;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
  T ans;
  if (n.getParam(name, ans))
  {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

/**
 * @description: 读取配置参数，通过roslaunch文件的参数服务器获得
 * @param {NodeHandle} &n ros节点
 * @return {*}
 */
void readParameters(ros::NodeHandle &n)
{
  std::string config_file;
  // 首先获取配置文件的路径
  config_file = readParam<std::string>(n, "config_file");
  //使用opencv的yaml文件接口来读取文件
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }
  std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

  fsSettings["image_topic"] >> IMAGE_TOPIC;
  fsSettings["imu_topic"] >> IMU_TOPIC;
  MAX_CNT = fsSettings["max_cnt"];   //单帧图像最大特征点数目
  MIN_DIST = fsSettings["min_dist"]; //两个特征点之间最短像素距离
  ROW = fsSettings["image_height"];
  COL = fsSettings["image_width"];
  FREQ = fsSettings["freq"]; // 相机图像发布频率
  F_THRESHOLD = fsSettings["F_threshold"];    //对极约束ransac求解的inlier门限值
  SHOW_TRACK = fsSettings["show_track"];
  EQUALIZE = fsSettings["equalize"];       //是否做均衡化处理
  FISHEYE = fsSettings["fisheye"];
  if (FISHEYE == 1)
    FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
  CAM_NAMES.push_back(config_file);

  WINDOW_SIZE = 20;
  STEREO_TRACK = false;
  FOCAL_LENGTH = 460;
  PUB_THIS_FRAME = false;

  if (FREQ == 0)
    FREQ = 100;

  fsSettings.release();
}
