//
// Created by zlc on 2020/1/18.
//

#ifndef _JMVIO_ESTIMATOR_H_
#define _JMVIO_ESTIMATOR_H_

#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"

#include "factor/intergration_base.h"

#include "backend/problem.h"


#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>


class Estimator
{
public:
    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    ///////////////////// OUR SOLVER //////////////////////
    MatXX Hprior_;
    VecX bprior_;
    VecX errprior_;
    MatXX Jprior_inv_;

    Eigen::Matrix2d project_sqrt_info_;

    ///////////////////// OUR SOLVER /////////////////////
    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag;
    Vector3d g;
    MatrixXd Ap[2], backup_A;
    VectorXd bp[2], backup_b;

    Matrix3d ric[NUM_OF_CAM];           // IMU和camera 旋转外参数
    Vector3d tic[NUM_OF_CAM];           // IMU和camera 平移外参数

    Vector3d Ps[(WINDOW_SIZE + 1)];     // IMU 预积分相关参数
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];     // IMU 预积分得到的旋转
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    double Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    // MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    //relocalization variable
    bool relocalization_info;
    double relo_frame_stamp;
    double relo_frame_index;
    int relo_frame_local_index;
    vector<Vector3d> match_points;
    double relo_Pose[SIZE_POSE];
    Matrix3d drift_correct_r;
    Vector3d drift_correct_t;
    Vector3d prev_relo_t;
    Matrix3d prev_relo_r;
    Vector3d relo_relative_t;
    Quaterniond relo_relative_q;
    double relo_relative_yaw;



    Estimator();

    void setParameter();            // 设置部分参数

    // interface, 处理IMU数据, 完成预积分
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);

    // 处理图像特征数据
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double header);
    // 重定位操作
    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);

    // internal
    void clearState();              // 清空或初始化滑动窗口中的所有状态量
    bool initialStructure();        // 视觉的结构初始化
    bool visualInitialAlign();      // 视觉惯性联合初始化
    // 判断两帧有足够视差30且内点数目大于12则可进行初始化，同时得到R和T
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();             // 滑动窗口法
    void solveOdometry();           // VIO非线性优化求解里程计
    void slideWindowNew();          // 滑动窗口法
    void slideWindowOld();
    void optimization();            // 基于滑动窗口的紧耦合的非线性优化, 残差项的构造和求解
    void backendOptimization();

    void problemSolve();
    void MargOldFrame();
    void MargNewFrame();

    void vector2double();           // vector转换成double数组, 因为ceres使用数值数组
    void double2vector();           // 数据转换, vector2double的相反过程
    bool failureDetection();        // 检测系统运行是否失败


};


#endif // _JMVIO_ESTIMATOR_H
