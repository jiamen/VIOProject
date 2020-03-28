//
// Created by zlc on 2020/1/18.
//
#include "parameters.h"
#include <iostream>
#include <opencv2/calib3d//calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

double INIT_DEPTH;      // 初始深度
double MIN_PARALLAX;
double ACC_N, ACC_W;    // 加速度计噪声—  — 高斯噪声, 随机游走
double GYR_N, GYR_W;    // 陀螺仪噪声 — — 高斯噪声, 随机游走

vector<Eigen::Matrix3d> RIC;
vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};   // 重力加速度

double BIAS_ACC_THRESHOLD;      // 加速度bias阈值
double BIAS_GYR_THRESHOLD;      // 陀螺仪bias阈值
double SOLVER_TIME;             // 求解时间

int NUM_ITERATIONS;             // 迭代次数
int ESTIMATE_EXTRINSIC;         // 外参数估计
int EXTIMATE_TD;
int ROLLING_SHUTTER;
string EX_CALIB_RESULT_PATH;    // 外参数标定结果路径  相机模型 K 矩阵
string VINS_RESULT_PATH;
double ROW, COL;

double TD, TR;

int FOCAL_LENGTH;    // 焦距
std::string IMAGE_TOPIC;     // 图像话题: ros topic
std::string IMU_TOPIC;       // IMU话题: ros topic
std::string FISHEYE_MASK;    // 鱼眼相机mask图的位置, 用于去除image信息的边缘噪声
std::vector<std::string> CAM_NAMES;  // 相机参数配置文件名
int MAX_CNT;     // 特征点最大个数
int MIN_DIST;    // 特征点之间的最小间隔
//int WINDOW_SIZE;
int FREQ;                    // 控制发布次数的频率
double F_THRESHOLD;          // ransac算法的门限
int SHOW_TRACK;              // 是否发布跟踪点的图像
bool STEREO_TRACK;       // 双目跟踪则为1
int EQUALIZE;                // 光太亮或太暗则为1，进行直方图均衡化
int FISHEYE;                 // 如果是鱼眼相机则为 1
bool PUB_THIS_FRAME;     // 是否需要发布特征点

void readpPatameters(string config_file)
{
    // 1. 读取文件
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if( !fsSettings.isOpened() )
    {
        cerr << "1 readParameters ERROR: Wrong path to settings!" << endl;
        return ;
    }


    // 2.根据读取文件内容设置参数
    FOCAL_LENGTH = 460;             // 设置焦距
    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];  // 限制最大迭代次数,保证实时性
    MIN_PARALLAX = fsSettings["keyframe_parallax"];     // 最小共视关系(最小视差), 10个像素
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;     // 真实世界最小视差=最小视差/焦距=10.0/460.0

    string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.txt";


    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    // ROS_INFO("ROW: %f, COL: %f", ROW, COL);


    // IMU 和 相机坐标系下的外部参数变换
    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if ( ESTIMATE_EXTRINSIC == 2 )
    {
        // ROS_WARN("have no prior extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());     // 假如没有IMU坐标系与camera坐标系之间的变换矩阵，需要在初始时刻晃动相机进行参数计算
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
    }
    else
    {
        if ( ESTIMATE_EXTRINSIC == 1 )
        {
            // ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if ( ESTIMATE_EXTRINSIC == 0 )
        {
            cout << "fix extrinsic param " << endl;
        }

        // Extrinsic parameter between IMU and camera 相机和IMU之间的变换, 旋转矩阵和平移矩阵
        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;

        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);  // 转换为四元数
        eigen_R = Q.normalized();

        RIC.push_back(eigen_R);         // 确定IMU与camera的旋转矩阵
        TIC.push_back(eigen_T);         // 确定IMU与camera的平移矩阵
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    // 设置td 和 tr
    TD = fsSettings["td"];   // initial value of time offset. unit: s. readed image clock + td = real image clock (IMU clock) 读图像时间 + td = 真实图像时间
    ESTIMATE_TD = fsSettings["estimate_td"];    // 相机和IMU同步校准得到的时间差

    // 是否为卷帘门相机
    ROLLING_SHUTTER = fsSettings["rolling_shutter"];   // 0: global shutter camera, 1: rolling shutter camera
    if ( ROLLING_SHUTTER )
    {
        TR = fsSettings["rolling_shutter_tr"];
    }
    else
    {
        TR = 0;     // 全局相机, 这里tr = 0
    }

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;   // 字符流, 所以用 ">>"
    MAX_CNT  = fsSettings["max_cnt"];       // 特征跟踪中最大特征点数
    MIN_DIST = fsSettings["min_dist"];      // 两个特征之间的最小距离

    FREQ = fsSettings["freq"];              // 控制发布次数的频率

    F_THRESHOLD = fsSettings["F_threshold"];

    SHOW_TRACK = fsSettings["show_track"];  // =1, publish tracking image as topic

    EQUALIZE = fsSettings["equalize"];      // =1, if image is too dark or light, trun on equalize to find enough features
    FISHEYE = fsSettings["fisheye"];            // =0, 不是鱼眼镜头

    CAM_NAMES.push_back(config_file);       // // 相机参数配置文件名

    // WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    PUB_THIS_FRAME = false;

    if ( FREQ == 0 )
    {
        FREQ = 10;
    }
    fsSettings.release();


    // 3.打印刚设置的参数
    cout << "1 readParameters: "
         << "\n INIT_DEPTH: "   << INIT_DEPTH
         << "\n MIN_PARALLAX: " << MIN_PARALLAX
         << "\n ACC_N: " << ACC_N
         << "\n ACC_W: " << ACC_W
         << "\n GYR_N: " << GYR_N
         << "\n GYR_W: " << GYR_W
         << "\n RIC: " << RIC[0]
         << "\n TIC: " << TIC[0]
         << "\n G： "  << G.transpose()
         << "\n BIAS_ACC_THRESHOLD: " << BIAS_ACC_THRESHOLD
         << "\n BIAS_GYR_THRESHOLD: " << BIAS_GYR_THRESHOLD
         << "\n SOLVER_TIME: " << SOLVER_TIME
         << "\n NUM_ITERATIONS: " << NUM_ITERATIONS
         << "\n ESTIMATE_EXTRINSIC: " << ESTIMATE_EXTRINSIC
         << "\n ESTIMATE_TD: " << ESTIMATE_TD
         << "\n ROLLING_SHUTTER: " << ROLLING_SHUTTER
         << "\n ROW: " << ROW
         << "\n COL: " << COL
         << "\n TD: "  << TD
         << "\n TR: "  << TR
         << "\n FOCAL_LENGTH: " << FOCAL_LENGTH
         << "\n IMAGE_TOPIC: "  << IMAGE_TOPIC
         << "\n IMU_TOPIC: "    << IMU_TOPIC
         << "\n FISHEYE_MASK: " << FISHEYE_MASK
         << "\n CAM_NAMES[0]: " << CAM_NAMES[0]
         << "\n MAX_CNT: "      << MAX_CNT
         << "\n MIN_DIST: "     << MIN_DIST
         << "\n FREQ: "         << FREQ
         << "\n F_THRESHOLD: "  << F_THRESHOLD
         << "\n SHOW_TRACK: "   << SHOW_TRACK
         << "\n STERED_TRACK: " << STEREO_TRACK
         << "\n EQUALIZE: "     << EQUALIZE
         << "\n FISHEYE: "      << FISHEYE
         << "\n PUB_THIS_FRAME: " << PUB_THIS_FRAME
         << endl;

}
