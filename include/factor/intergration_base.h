//
// Created by zlc on 2020/4/7.
//

#ifndef _JMVIO_INTERGRATION_BASE_H_
#define _JMVIO_INTERGRATION_BASE_H_

#pragma once

#include "../utility/utility.h"
#include "../parameters.h"
#include <ceres/ceres.h>

using namespace Eigen;


class IntegrationBase
{
public:
    double dt;
    Eigen::Vector3d acc_0, gyr_0;       // 上一时刻 加速度和角速度
    Eigen::Vector3d acc_1, gyr_1;       // 本时刻 加速度和角速度

    const Eigen::Vector3d linearized_acc, linearized_gyr;
    Eigen::Vector3d linearized_ba, linearized_bg;           // 线性化加速度a bias 和 线性化角速度w bias

    Eigen::Matrix<double, 15, 15> jacobian, covariance;
    Eigen::Matrix<double, 15, 15> step_jacobian;
    Eigen::Matrix<double, 15, 18> step_V;
    Eigen::Matrix<double, 18, 18> noise;        // 噪声项的对角协方差矩阵

    double sum_dt;
    Eigen::Vector3d delta_p;        // 位移预积分值 αbibj
    Eigen::Quaterniond delta_q;
    Eigen::Vector3d delta_v;

    std::vector<double> dt_buf;
    std::vector<Eigen::Vector3d> acc_buf;
    std::vector<Eigen::Vector3d> gyr_buf;


    IntegrationBase() = delete;
    IntegrationBase(const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                    const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
            : acc_0{_acc_0}, gyr_0{_gyr_0}, linearized_acc{_acc_0}, linearized_gyr{_gyr_0},
              linearized_ba{_linearized_ba}, linearized_bg{_linearized_bg},
              jacobian{Eigen::Matrix<double, 15, 15>::Identity()}, covariance{Eigen::Matrix<double, 15, 15>::Zero()},
              sum_dt{0.0}, delta_p{Eigen::Vector3d::Zero()}, delta_q{Eigen::Quaterniond::Identity()}, delta_v{Eigen::Vector3d::Zero()}

    {
        // 噪声项的对角协方差矩阵
        noise = Eigen::Matrix<double, 18, 18>::Zero();
        noise.block<3, 3>(0, 0) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(3, 3) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(6, 6) =  (ACC_N * ACC_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(9, 9) =  (GYR_N * GYR_N) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(12, 12) =  (ACC_W * ACC_W) * Eigen::Matrix3d::Identity();
        noise.block<3, 3>(15, 15) =  (GYR_W * GYR_W) * Eigen::Matrix3d::Identity();
    }


    void push_back(double dt, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyr)
    {
        dt_buf.push_back(dt);
        acc_buf.push_back(acc);
        gyr_buf.push_back(gyr);
        propagate(dt, acc, gyr);
    }


    //
    void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1)
    {
        dt = _dt;
        acc_1 = _acc_1;
        gyr_1 = _gyr_1;

        Vector3d result_delta_p;
        Quaterniond result_delta_q;
        Vector3d result_delta_v;
        Vector3d result_linearized_ba;
        Vector3d result_linearized_bg;


        // 进行中值积分
        midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v,
                            linearized_ba, linearized_bg,
                            result_delta_p, result_delta_q, result_delta_v,
                            result_linearized_ba, result_linearized_bg, 1);

        //checkJacobian(_dt, acc_0, gyr_0, acc_1, gyr_1, delta_p, delta_q, delta_v,
        //                    linearized_ba, linearized_bg);
        delta_p = result_delta_p;
        delta_q = result_delta_q;
        delta_v = result_delta_v;
        linearized_ba = result_linearized_ba;
        linearized_bg = result_linearized_bg;
        delta_q.normalize();
        sum_dt += dt;

        acc_0 = acc_1;  // 本时刻成为上一时刻
        gyr_0 = gyr_1;

    }


    void midPointIntegration(double _dt,
                             const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                             const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                             const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, const Eigen::Vector3d &delta_v,
                             const Eigen::Vector3d &linearized_ba, const Eigen::Vector3d &linearized_bg,
                             Eigen::Vector3d &result_delta_p, Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                             Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg, bool update_jacobian)
    {
        //ROS_INFO("midpoint integration");
        // delta_q = q_bi_bk  这个关系来自手写VIO第3讲 44页, 也可以见崔博解析中的2.5节 两帧之间PVQ增量的中值法离散形式
        Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
        Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;  // 1/2 w均值, 陀螺仪 角速度w均值
        // Q 增量 中值积分 result_delta_q = q_bi_bk+1
        result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * _dt / 2, un_gyr(1) * _dt / 2, un_gyr(2) * _dt / 2);

        Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);      // 加速度a中值

        // delta_v = v_bi_bk
        result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;    // P位移, 前面几行主要是为了得到 un_acc
        // result_delta_v = v_bi_bk+1
        result_delta_v = delta_v + un_acc * _dt;        // 速度 V
        result_linearized_ba = linearized_ba;
        result_linearized_bg = linearized_bg;


        if ( update_jacobian )  // 更新雅克比, 崔博论文中PVQ增量误差分析中2.7节 的式(15)中的 F 和 V
        {
            Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
            Vector3d a_0_x = _acc_0 - linearized_ba;
            Vector3d a_1_x = _acc_1 - linearized_ba;
            Matrix3d R_w_x, R_a_0_x, R_a_1_x;

            // [w]×  论文第4页 公式(4)
            R_w_x<<0, -w_x(2), w_x(1),
                    w_x(2), 0, -w_x(0),
                    -w_x(1), w_x(0), 0;
            R_a_0_x<<0, -a_0_x(2), a_0_x(1),
                    a_0_x(2), 0, -a_0_x(0),
                    -a_0_x(1), a_0_x(0), 0;
            R_a_1_x<<0, -a_1_x(2), a_1_x(1),
                    a_1_x(2), 0, -a_1_x(0),
                    -a_1_x(1), a_1_x(0), 0;

            // 详见崔华坤大佬的《VINS论文推导及代码解析》2.7节内容, 离散形式的PVQ增量误差分析
            MatrixXd F = MatrixXd::Zero(15, 15);
            F.block<3, 3>(0, 0) = Matrix3d::Identity();
            F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * _dt * _dt +
                                  -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
            F.block<3, 3>(0, 6) = MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt * _dt;
            F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * _dt * -_dt;
            F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
            F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3,3) * _dt;
            F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * _dt +
                                  -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
            F.block<3, 3>(6, 6) = Matrix3d::Identity();
            F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * _dt;
            F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * _dt * -_dt;
            F.block<3, 3>(9, 9) = Matrix3d::Identity();
            F.block<3, 3>(12, 12) = Matrix3d::Identity();
            //cout<<"A"<<endl<<A<<endl;


            // 详见手写VIO课件的公式(46)和(47), 崔华坤解析中差一个负号
            MatrixXd V = MatrixXd::Zero(15,18);
            V.block<3, 3>(0, 0) =  0.25 * delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 3) =  0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * _dt * 0.5 * _dt;
            V.block<3, 3>(0, 6) =  0.25 * result_delta_q.toRotationMatrix() * _dt * _dt;
            V.block<3, 3>(0, 9) =  V.block<3, 3>(0, 3);
            V.block<3, 3>(3, 3) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(3, 9) =  0.5 * MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(6, 0) =  0.5 * delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 3) =  0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x  * _dt * 0.5 * _dt;
            V.block<3, 3>(6, 6) =  0.5 * result_delta_q.toRotationMatrix() * _dt;
            V.block<3, 3>(6, 9) =  V.block<3, 3>(6, 3);
            V.block<3, 3>(9, 12) = MatrixXd::Identity(3,3) * _dt;
            V.block<3, 3>(12, 15) = MatrixXd::Identity(3,3) * _dt;

            //step_jacobian = F;
            //step_V = V;
            // 离散形式的PVQ增量误差的Jacobian和协方差
            jacobian = F * jacobian;    // 崔华坤解析公式(16), 手写VIO公式(78)
            covariance = F * covariance * F.transpose() + V * noise * V.transpose();  // VQV^T
        }

    }


    void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg)
    {
        sum_dt = 0.0;
        acc_0 = linearized_acc;
        gyr_0 = linearized_gyr;
        delta_p.setZero();
        delta_q.setIdentity();
        delta_v.setZero();
        linearized_ba = _linearized_ba;
        linearized_bg = _linearized_bg;
        jacobian.setIdentity();
        covariance.setZero();

        // 在dt_buf时间内, 新的bias重新进行计算预积分
        for (int i = 0; i < static_cast<int>(dt_buf.size()); i++)
            propagate(dt_buf[i], acc_buf[i], gyr_buf[i]);
    }



    /*  下面函数构造IMU残差 */
    Eigen::Matrix<double, 15, 1> evaluate(const Eigen::Vector3d &Pi, const Eigen::Quaterniond &Qi, const Eigen::Vector3d &Vi, const Eigen::Vector3d &Bai, const Eigen::Vector3d &Bgi,
                                          const Eigen::Vector3d &Pj, const Eigen::Quaterniond &Qj, const Eigen::Vector3d &Vj, const Eigen::Vector3d &Baj, const Eigen::Vector3d &Bgj)
    {
        Eigen::Matrix<double, 15, 1> residuals;

        // 首先获取预积分的PQV向量对陀螺仪和加速度的偏置的雅克比矩阵, 然后计算偏置的变化量, 根据偏置的更新对预积分的值进行矫正,
        // 下面的corrected_delta_q, corrected_delta_v, corrected_delta_p 是 矫正后的预积分
        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);   // (0, 9)
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);   // (0, 12)

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);   // (3, 12)

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);   // (6, 9)
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);   // (6, 12)

        Eigen::Vector3d dba = Bai - linearized_ba;
        Eigen::Vector3d dbg = Bgi - linearized_bg;

        // 第3讲PPT 70页 公式(77)
        // 在求解预积分时, 是假设IMU的偏置ba, bw已经确定, 实际上在后续的优化中对偏置也进行了优化。
        // 那么每次优化时, ba, bw发生改变需要重新根据公式求得所有帧之间的IMU的预积分。
        // 当偏置变化很小时，可以将预积分值按其对偏置的一阶近似来调整，否则就进行重新传递reprogagte。
        Eigen::Quaterniond corrected_delta_q = delta_q * Utility::deltaQ(dq_dbg * dbg); // 这里看deltaQ函数, 确实是1/2 J_bg δb_i
        Eigen::Vector3d corrected_delta_v = delta_v + dv_dba * dba + dv_dbg * dbg;
        Eigen::Vector3d corrected_delta_p = delta_p + dp_dba * dba + dp_dbg * dbg;

        // 下面正式开始为 15*1 的残差项赋值, 在第3讲PPT的69页
        // Qi: q_w_bi ;  Qi.inverse(): q_bi_w   rp = q_bi_w * ( 1/2 g_w △t^2  )
        residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
        residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * sum_dt + Vj - Vi) - corrected_delta_v;
        residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;

        return residuals;
    }

};

#endif // _JMVIO_INTERGRATION_BASE_H_
