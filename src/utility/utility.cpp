//
// Created by zlc on 2020/3/20.
//

#include "utility/utility.h"

// 通过将重力旋转到z轴上, 得到世界坐标系与摄像机坐标系c0之间的旋转矩阵
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized();        // 相机坐标系 c0 下的 归一化重力加速度
    Eigen::Vector3d ng2{0, 0, 1.0};     // 世界坐标系 w  下的 归一化重力加速度

    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();   // 从两个向量生成四元数，然后转成旋转矩阵
    double yaw = Utility::R2ypr(R0).x();        // 旋转矩阵到欧拉角, 然后取出 Θz, 偏航角

    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;   // Rwc0 = exp(Θu)
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;

    return R0;
}
