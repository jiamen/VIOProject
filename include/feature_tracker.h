//
// Created by zlc on 2020/1/18.
//

#ifndef _JMVIO_FEATURE_TRACKER_H_
#define _JMVIO_FEATURE_TRACKER_H_

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>


#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);       // 判断点是否在图像内

// 根据状态status进行重组, 将status中为1的对应点保存下来, 表示跟踪成功, 为0的对应点去掉
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);


class FeatureTracker
{
public:
    cv::Mat mask;               // 图形掩膜
    cv::Mat fisheye_mask;       // 鱼眼相机的掩膜
    cv::Mat prev_img, cur_img, forw_img;
    // prev_img 是上一次发布的帧的图像数据
    // cur_img 是光流跟踪前一帧的图像数据
    // forw_img 是光流跟踪的后一帧的图像数据
    vector<cv::Point2f> n_pts;  // 每一帧中新提取的特征点
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;    // 对应的图像特征点
    vector<cv::Point2f> prev_un_pts, cur_un_pts;        // 归一化相机坐标系下的坐标
    vector<cv::Point2f> pts_velocity;      // 当前帧相对前一帧特征点沿x, y方向的像素移动速度
    vector<int> ids;        // 能够被跟踪到的特征点的id
    vector<int> track_cnt;  // 当前帧forw_img中每个特征点被追踪的次数
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;  // 把特征点的id和特征点位置封装起来的数据类型

    camodocal::CameraPtr m_camera;  // 相机模型
    double cur_time;
    double prev_time;

    static int n_id;    // 用来作为特征点id, 每检测一个新的特征点, 就将n_id作为该特征点的id, 然后n_id+1


    FeatureTracker();

    void readImage(const cv::Mat &_img, double _cur_time);      // ☆☆☆△△△ 读取图像

    void setMask();     // 对跟踪点进行排序并去除密集点

    void addPoints();   // 对跟踪点进行排序并去除密集点

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();             // 通过基本矩阵（F）去除外点outliers

    void undistortedPoints();       // 进行畸变矫正

};


#endif // _JMVIO_FEATURE_TRACKER_H
