//
// Created by zlc on 2020/4/7.
//

#ifndef _JMVIO_INITIAL_SFM_H_
#define _JMVIO_INITIAL_SFM_H_

#pragma once

#include <ceres//ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace std;

// 将f_manager中的所有feature保存到vector<SFMFeature> sfm_f中，代码略
// 这里解释一下SFMFeature，其存放的是特征点的信息
struct SFMFeature
{
    bool state;     // 状态(是否被三角化)
    int id;
    vector<pair<int, Vector2d>> observation; // 所有观测到该特征点的图像帧ID和图像坐标
    double position[3];     // ?坐标系下的3D坐标
    double depth;           // 深度
};

//
struct ReprojectionError3D
{
    double observed_u;
    double observed_v;

    ReprojectionError3D(double observed_u, double observed_v)
        : observed_u(observed_u), observed_v(observed_v)
    {}

    template <typename T>
    bool operator() (const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const
    {
        T p[3];
        ceres::QuaternionRotatePoint(camera_R, point, p);

        p[0] += camera_T[0];
        p[1] += camera_T[1];
        p[2] += camera_T[2];

        T xp = p[0] / p[2];     // 重投影坐标
        T yp = p[1] / p[2];

        residuals[0] = xp - T(observed_u);  // 冲投影视觉残差
        residuals[1] = yp - T(observed_v);

        return true;
    }

    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y)
    {
        return ( new ceres::AutoDiffCostFunction<
                 ReprojectionError3D, 2, 4, 3, 3> (
                        new ReprojectionError3D(observed_x, observed_y) ) );
    }
};

//
class GlobalSFM
{
private:
    int feature_num;    // 滑动窗口内特征点数量

    /* 根据当前已经三角化的特征点估计某一帧的R,t */
    bool solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i, vector<SFMFeature> &sfm_f);

    /* 输入两个pose和2D观测点, 三角化3D特征点 */
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                            Vector2d &point0, Vector2d &point1, Vector3d &point_3d);

    /* 输入两帧的pose, 三角化它们共同观测的特征点, 之前已经被三角化的特征点不再处理 */
    void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
                              int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
                              vector<SFMFeature> &sfm_f);

public:
    GlobalSFM();

    /*  外部调用接口, 主要处理函数, 输入第l帧和最后一帧的相对R,t,根据特征点的观测估计所有帧的位姿和特征点的3D坐标
     *  param[in] :  frame_num：pose的个数, elements in q, T
     *  param[out]:  q: SFM结果, 每帧在l帧参考系下的quaternion
     *  param[in] :  T: SFM结果, 每帧在l帧参考系下的position
     *  param[in] :  l: 以第l帧为参考系,即l帧的pose为坐标原点
     *  param[in] :  relative_R: 第l帧到最后一帧的相对旋转
     *  param[in] :  relative_T: 第l帧到最后一帧的相对平移
     *  param[in] :  sfm_f: feature list,每个SFMFeature中包含多个观测
     *  param[out]:  sfm_tracked_point: 优化后的3D特征点在l帧参考系的position
     * */
    // 纯视觉sfm, 求解窗口中所有图像帧的位姿QT(相对于第1帧) 和 特征点坐标 sfm_tracked_points
    bool construct(int frame_num, Quaterniond* q, Vector3d* T, int l,
            const Matrix3d relative_R, const Vector3d relative_T,
            vector<SFMFeature> &sfm_f, map<int, Vector3d> &sfm_tracked_points);
};


#endif // _JMVIO_INITIAL_SFM_H_
