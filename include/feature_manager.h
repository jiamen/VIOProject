//
// Created by zlc on 2020/1/18.
//

#ifndef _JMVIO_FEATURE_MANAGER_H_
#define _JMVIO_FEATURE_MANAGER_H_


#include <list>
#include <algorithm>
#include <vector>
#include <numeric>  // 此头文件是数值库的一部分
#include <map>

using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

// #include <ros/console.h>
// #include <ros/assert.h>

#include "parameters.h"


class FeaturePerFrame   // 每帧的基本数据, 每个路标点在一帧图像中的信息变量。
{               // _point 每帧的特征点[x, y, z, u, v, vx, vy]  ,  cur_td IMU和cam同步时间差
public:
    double cur_td;
    Vector3d point;     // 相机坐标系下的 3D特征点坐标
    Vector2d uv;
    Vector2d velocity;

    double z;           // 特征点的深度
    bool is_used;       // 是否被用了
    double parallax;    // 视差
    MatrixXd A;         // 变换矩阵
    VectorXd b;
    double dep_gradient;

    FeaturePerFrame(const Eigen::Matrix<double, 7, 1>& _point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);

        uv.x() = _point(3);
        uv.y() = _point(4);

        velocity.x() = _point(5);
        velocity.y() = _point(6);

        cur_td = td;
    }
};


class FeaturePerId      // 某feature_id下的所有FeaturePerFrame。 每个路标点由多个连续的图像帧观测到
{                       // 常用feature_id和观测第一帧start_frame、最后一帧endFrame
public:
    const int feature_id;       // 特征点ID索引
    int start_frame;            // 首次被观测到时, 该帧的索引
    vector<FeaturePerFrame> feature_per_frame;      // 能够观测到某个特征点的所有相关帧

    int     used_num;           // 该特征点出现的次数
    bool    is_outlier;         // 是否为外点
    bool    is_margin;          // 是否Marg边缘化
    double  estimated_depth;    // 估计的逆深度
    int     solve_flag;         // 求解器： 0 haven't solve yet ; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {

    }

    int endFrame();         // 返回最后一次观测到这个特征点的图像帧索引ID
};


class FeatureManager        // 管理所有特征点, 通过list容器存储特征点属性
{
private:
    double compensatedParallax2(const FeaturePerId& it_per_id, int frame_count);
    const Matrix3d* Rs;
    Matrix3d ric[NUM_OF_CAM];

public:
    list<FeaturePerId> feature;     // 存储每一个特征点, 及其对应的每帧的属性
    int last_track_num;

    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);

    void clearState();

    int getFeatureCount();      // 窗口中被跟踪的特征点数量

    // 特征点进入时检查视差,是否为关键帧
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& image, double td);

    void debugShow();

    // 前后两帧之间匹配特征点3D坐标
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    // void updateDepth(const VectorXd &x);

    void setDepth(const VectorXd& x);       // 设置特征点逆深度
    void removeFailures();
    void clearDepth(const VectorXd& x);
    VectorXd getDepthVector();
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);    // 特征点三角化求深度(SVD分解)
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);

    void removeBack();                      // 边缘化最老帧, 直接将特征点保存的帧号前移
    void removeFront(int frame_count);      // 边缘化次新帧，对特征点在次新帧的信息移除

    void removeOutlier();       // 移除外点

};


#endif //JMVIO_FEATURE_MANAGER_H
