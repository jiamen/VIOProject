//
// Created by zlc on 2020/1/18.
//

// VINS的状态估计器模块（estimator）
/*
 * 这个模块可以说是VINS的最核心模块，从论文的内容上来说，
 * 里面的内容包括了VINS的估计器初始化、基于滑动窗口的非线性优化实现紧耦合，
 * 即论文第五章（V. ESTIMATOR INITIALIZATION）
 * 第六章（VI. TIGHTLY-COUPLED MONOCULAR VIO）。
 * 此外还包括了关键帧的选择，
 * 即论文第四章（IV. MEASUREMENT PREPROCESSING A. Vision Processing Front-end） 的部分内容。
 * */

#include "estimator.h"

#include "backend/vertex_inverse_depth.h"
#include "backend/vertex_pose.h"
#include "backend/vertex_speedbias.h"
#include "backend/edge_reprojection.h"
#include "backend/edge_imu.h"

#include <iostream>
#include <fstream>


using namespace JMVIO;

Estimator::Estimator() : f_manager{Rs}
{

}


// https://blog.csdn.net/qq_41839222/article/details/86290941#commentBox  看本函数讲解
// 下面是基于IMU模型中值积分的离散形式, 作用是将IMU积分出来第j时刻PVQ作为第j帧图像的优化初始值
void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if ( !first_imu )
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if ( !pre_integrations[frame_count] )
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    if ( frame_count != 0 )
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;                   // q_i(a_t - b_ai) - g_w
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];        // w_t 实际值中值
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();           // q_i+1 = q_w_bk+1
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;     // q_i+1(a_t+1 - b_ai) - g_w
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);                      // 真实值 = 1/2
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;                       // p_w_bk+1
        Vs[j] += dt * un_acc;
    }

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}
