//
// Created by zlc on 2020/4/7.
//

#ifndef _JMVIO_SOLVE_5PTS_H_
#define _JMVIO_SOLVE_5PTS_H_

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
// #include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

// #include <ros/console.h>

class MotionEstimator
{
private:
    double testTriangulation(const vector<cv::Point2f> &l,
                             const vector<cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);

    void decomposeE(cv::Mat E,
                    cv::Mat_<double> &R1, cv::Mat_<double> &R2,
                    cv::Mat_<double> &t1, cv::Mat_<double> &t2);


public:
    bool solveRelativeRT(const vector<pair<Vector3d, Vector3d>> &corres, Matrix3d &R, Vector3d &T);

};


#endif // _JMVIO_SOLVE_5PTS_H_
