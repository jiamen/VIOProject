//
// Created by zlc on 2020/1/18.
//

#ifndef JMVIO_SYSTEM_H
#define JMVIO_SYSTEM_H

#pragma once

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>    // 互斥量

#include <fstream>
#include <condition_variable> // 条件变量


#include <pangolin/pangolin.h>

#include "estimator.h"
#include "parameters.h"
#include "feature_tracker.h"



// IMU for VIO      IMU数据
// 输入为IMU传感器得到的线性加速度和角速度，
// 输出为积分得到的位姿(位置和角度)
struct IMU_MSG
{
    double header;
    Eigen::Vector3d liner_acceleration;
    Eigen::Vector3d angular_velocity;
};
typedef std::shared_ptr<IMU_MSG const> ImuConstPtr;


// Image for VIO    图像
struct IMG_MSG
{
    double header;
    std::vector<Vector3d> points;
    std::vector<int> id_of_point;
    std::vector<float> u_of_point;
    std::vector<float> v_of_point;
    std::vector<float> velocity_x_of_point;
    std::vector<float> velocity_y_of_point;
};
typedef std::shared_ptr<IMG_MSG const> ImgConstPtr;


class System
{
private:

    // feature tracker
    std::vector<uchar> r_status;
    std::vector<float> r_err;
    // std::queue<ImageConstPtr> img_buf;

    // ros::Publisher pub_img, pub_match;
    // ros::Publisher pub_restart;

    FeatureTracker trackerData[NUM_OF_CAM];
    double first_image_time;
    int pub_count = 1;
    bool first_image_flag = true;
    double last_image_time = 0;
    bool init_pub = 0;

    // estimator
    Estimator estimator;

    std::condition_variable con;
    double current_time = -1;
    std::queue<ImuConstPtr> imu_buf;
    std::queue<ImgConstPtr> feature_buf;
    // std::queue<PointCloudConstPtr> relo_buf;
    int sum_of_wait = 0;

    std::mutex m_buf;
    std::mutex m_state;
    std::mutex i_buf;
    std::mutex m_estimator;


    double latest_time;
    Eigen::Vector3d tmp_P;          // IMU位移
    Eigen::Quaterniond tmp_Q;       // IMU旋转
    Eigen::Vector3d tmp_V;          // IMU速度
    Eigen::Vector3d tmp_Ba;         // IMU加速度随机游走
    Eigen::Vector3d tmp_Bg;         // IMU陀螺仪随机游走
    Eigen::Vector3d acc_0;          // 加速度值
    Eigen::Vector3d gyr_0;          // 陀螺仪值

    bool init_feature = 0;
    bool init_imu = 1;
    double last_imu_t = 0;
    std::ofstream ofs_pose;
    std::vector<Eigen::Vector3d> vPath_to_draw;
    bool bStart_backend;
    std::vector< std::pair< std::vector<ImuConstPtr>, ImgConstPtr > > getMeasurements();


public:
    System(const std::string sConfig_files);

    void PubImageData(double dStampSec, cv::Mat &img);
    void PubImuData(double dStampSec, const Eigen::Vector3d &vGyr, const Eigen::Vector3d &vAcc);

    // thread: visual-inertial odometry
    void ProcessBackEnd();
    void Draw();

    pangolin::OpenGlRenderState s_cam;
    pangolin::View d_cam;

    ~System();


// 兼容性代码
#ifdef __APPLE__
    void InitDrawGL();
    void DrawGLFrame();
#endif

};

#endif //JMVIO_SYSTEM_H
