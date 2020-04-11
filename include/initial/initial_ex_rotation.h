//
// Created by zlc on 2020/4/7.
//

#ifndef _JMVIO_INITIAL_EX_ROTATION_H_
#define _JMVIO_INITIAL_EX_ROTATION_H_

#pragma once

#include <vector>
#include "../parameters.h"

using namespace std;

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>
using namespace Eigen;
// #include <ros/console.h>

/* This class help you to calibrate extrinsic rotation between imu and camera
 * when your totally don't know the extrinsic parameter*/
// 当外参完全不知道的时候, VINS 也可以在线对其进行估计(rotation),
// 先在processImage内进行初步估计, 然后在后续优化时, 会在optimize函数中再次优化。

class InitialEXRotation
{
private:
    vector<Matrix3d> Rc;    //
    vector<Matrix3d> Rimu;
    vector<Matrix3d> Rc_g;  //
    Matrix3d ric;           // camera 到 imu之间的变换

    int frame_count;

    Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres);

    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);

    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);


public:
    InitialEXRotation();
    bool CalibrationExRotation(vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu, Matrix3d &calib_ric_result);

};


#endif //JMVIO_INITIAL_EX_ROTATION_H
