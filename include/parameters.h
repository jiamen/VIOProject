//
// Created by zlc on 2020/1/18.
//

#ifndef _JMVIO_PARAMETERS_H_
#define _JMVIO_PARAMETERS_H_

#pragma once
// #include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
// #include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include <fstream>

// feature tracker
// extern int ROW;          // 图像宽度
// extern int COL;          // 图像高度
const int NUM_OF_CAM = 1;   // 相机数量


extern int FOCAL_LENGTH;    // 焦距
extern std::string IMAGE_TOPIC;     // 图像话题: ros topic
extern std::string IMU_TOPIC;       // IMU话题: ros topic
extern std::string FISHEYE_MASK;    // 鱼眼相机mask图的位置, 用于去除image信息的边缘噪声
extern std::vector<std::string> CAM_NAMES;  // 相机参数配置文件名
extern int MAX_CNT;     // 特征点最大个数
extern int MIN_DIST;    // 特征点之间的最小间隔
//extern int WINDOW_SIZE;
extern int FREQ;                    // 控制发布次数的频率
extern double F_THRESHOLD;          // ransac算法的门限
extern int SHOW_TRACK;              // 是否发布跟踪点的图像
extern bool STEREO_TRACK;           // 双目跟踪则为1
extern int EQUALIZE;                // 光太亮或太暗则为1，进行直方图均衡化
extern int FISHEYE;                 // 如果是鱼眼相机则为 1
extern bool PUB_THIS_FRAME;         // 是否需要发布特征点


// estimator
//const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;         // 滑动窗口的大小
// const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
// #define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;         // ☆☆☆ 共视关系中的视差, 最少10个像素, 也是关键帧选择的判断依据 ☆☆☆
extern int ESTIMATE_EXTRINSIC;


// 噪声相关
extern double ACC_N, ACC_W;     // 加速度计噪声：高斯噪声和随机游走噪声
extern double GYR_N, GYR_W;     // 陀螺仪噪声： 高斯噪声和随机游走噪声


extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;                   // 重力加速度

extern double BIAS_ACC_THRESHOLD;           // 加速度BIAS阈值
extern double BIAS_GYR_THRESHOLD;           // 陀螺仪BIAS阈值
extern double SOLVER_TIME;                  // 求解时间
extern int NUM_ITERATIONS;                  // 迭代次数
extern std::string EX_CALIB_RESULT_PATH;    //
extern std::string VINS_RESULT_PATH;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;


// void readParameters(ros::NodeHandle &n);

void readParameters(std::string config_file);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,              // 相机位姿6自由度7变量表示： q 旋转四元数, x,y,z平移3个
    SIZE_SPEEDBIAS = 9,         // 滑动窗口11帧对应的速度,ba,bg,9自由度
    SIZE_FEATURE = 1            // 特征点： 逆深度表示?
};

enum StateOrder
{
    O_P = 0,            // 位移
    O_R = 3,            // 旋转
    O_V = 6,            // 速度
    O_BA = 9,           // 加速度bias
    O_BG = 12           // 陀螺仪bias
};

enum NoiseOrder     // 噪声序列
{
    O_AN = 0,       // 加速度计的测量噪声，高斯噪声
    O_GN = 3,       // 陀螺仪的测量噪声，高斯噪声
    O_AW = 6,       // 加速度计的随机游走 噪声
    O_GW = 9        // 陀螺仪的随机游走 噪声
};


#endif // _JMVIO_PARAMETERS_H
