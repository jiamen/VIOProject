//
// Created by zlc on 2020/4/12.
//
#include "../thirdparty//Sophus/sophus/se3.hpp"
#include "backend/vertex_pose.h"
#include "backend/edge_reprojection.h"
#include "utility/utility.h"

#include <iostream>

namespace JMVIO
{

namespace backend
{
    /*
     std::vector<std::shared_ptr<Vertex>> verticies_;   // 该边对应的顶点
     VecX residual_;                    // 残差
     std::vector<MatXX> jacobians_;     // 雅克比. 每个雅克比维度是 redisual × vertex[i]
     MatXX information_;                // 信息矩阵
     VecX observation_;                 // 观测信息
     */

// 首先计算残差，这里的残差是按照第三讲中的视觉重投影误差计算的，
// 首先取得i帧下的逆深度，然后获得两帧相机的姿态，
// 之后通过坐标系的转换将i帧下的像素点坐标pts_i_转换到j帧的坐标系下，
// 最后将转换过来的像素坐标(pts_camera_j / dep_j)和j帧下的测量值取差得到残差。
void EdgeReprojection::ComputeResidual()
{
    // std::cout << pts_i.transpose() << " " << pts_j_.transpose() << std::endl;

    double inv_dep_i = verticies_[0]->Parameters()[0];

    VecX param_i = verticies_[1]->Parameters();
    Qd Qi(param_i[6], param_i[3], param_i[4], param_i[5]);
    Vec3 Pi = param_i.head<3>();

    VecX param_j = verticies_[2]->Parameters();
    Qd Qj(param_j[6], param_j[3], param_j[4], param_j[5]);
    Vec3 Pj = param_j.head<3>();

    VecX param_ext = verticies_[3]->Parameters();
    Qd qic(param_ext[6], param_ext[3], param_ext[4], param_ext[5]);
    Vec3 tic = param_ext.head<3>();

    Vec3 pts_camera_i = pts_i_ / inv_dep_i;         // 当前i帧相机坐标系下归一化点转3维坐标点, 带有z深度值
    Vec3 pts_imu_i = qic * pts_camera_i + tic;      // 当前i帧相机坐标系下三维点 --> i帧下body系
    Vec3 pts_w = Qi * pts_imu_i + Pi;               // 当前i帧body系 --> 世界坐标系
    Vec3 pts_imu_j = Qj.inverse() * (pts_w - Pj);   // 世界坐标系 --> 上一帧j帧body系
    Vec3 pts_camera_j = qic.inverse() * (pts_imu_j - tic);  // 上一阵j帧body系 --> j帧相机坐标系

    double dep_j = pts_camera_j.z();    // j帧深度
    residual_ = (pts_camera_j / dep_j).head<2>() - pts_j_.head<2>();  // j帧重投影值 - j帧真实观测
    /// J^t * J * delta_x = - J^t * r
//    residual_ = information_ * residual_;   // remove information here, we multi information matrix in problem solver
}

//void EdgeReprojection::SetTranslationImuFromCamera(Eigen::Quaterniond &qic_, Vec3 &tic_) {
//    qic = qic_;
//    tic = tic_;
//}


void EdgeReprojection::ComputeJacobians()
{
    double inv_dep_i = verticies_[0]->Parameters()[0];

    VecX param_i = verticies_[1]->Parameters(); // 本帧状态xi
    Qd Qi(param_i[6], param_i[3], param_i[4], param_i[5]);  // 旋转Qwi
    Vec3 Pi = param_i.head<3>();    // 平移Pwi

    VecX param_j = verticies_[2]->Parameters(); // 上一帧状态xj
    Qd Qj(param_j[6], param_j[3], param_j[4], param_j[5]);  // 旋转Qwj
    Vec3 Pj = param_j.head<3>();    // 平移Pwj

    VecX param_ext = verticies_[3]->Parameters();
    Qd qic(param_ext[6], param_ext[3], param_ext[4], param_ext[5]);
    Vec3 tic = param_ext.head<3>();

    Vec3 pts_camera_i = pts_i_ / inv_dep_i;
    Vec3 pts_imu_i = qic * pts_camera_i + tic;
    Vec3 pts_w = Qi * pts_imu_i + Pi;
    Vec3 pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Vec3 pts_camera_j = qic.inverse() * (pts_camera_j - tic);

    double dep_j = pts_camera_j.z();    // 上一帧j帧下的特征点深度

    Mat33 Ri = Qi.toRotationMatrix();   // Rwi
    Mat33 Rj = Qj.toRotationMatrix();
    Mat33 ric = qic.toRotationMatrix();
    Mat23 reduce(2, 3);
    reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
              0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);    // 第3讲PPT式(63)
    // reduce = information_ * reduce;

    // 对i时刻的状态量进行求导, 第3讲PPT64页过程
    Eigen::Matrix<double, 2, 6> jacobian_pose_i;
    Eigen::Matrix<double, 3, 6> jaco_i;
    jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();  // 对位移求导, 式64中, Rbc^T × Rwbj^T
    jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Sophus::SO3d::hat(pts_imu_j);  // 式(67), pts_imu_j, j时刻特征点坐标在body系下,具体变换看(61)第3个式子
    jacobian_pose_i.leftCols<6>() = reduce * jaco_i;    // 根据链式法则, 求出整体对i时刻状态量的导数

    // 对j时刻状态量求导
    Eigen::Matrix<double, 2, 6> jacobian_pose_j;
    Eigen::Matrix<double, 3, 6> jaco_j;
    jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();   // 对应j时刻位移求导, 式(68)
    jaco_j.rightCols<3>() = ric.transpose() * Sophus::SO3d::hat(pts_imu_j); // 式(70)
    jacobian_pose_j.leftCols<6>() = reduce * jaco_j;    // 根据链式法则, 求出整体对j时刻状态量的导数

    // 对特征点逆深度求导
    Eigen::Vector2d jacobian_feature;   // 下面是对特征点逆深度求导, PPT68页式(76)
    //jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i_ * -1.0 / (inv_dep_i * inv_dep_i);
    jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * -1.0 / inv_dep_i * pts_camera_i;

    // 对imu和相机之间的外参数求导
    Eigen::Matrix<double, 2, 6> jacobian_ex_pose;
    Eigen::Matrix<double, 3, 6> jaco_ex;
    jaco_ex.leftCols<3>() = ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());  // 式(71)
    Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;        // 下面是式(74) + 式(75)
    jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_camera_i) + Utility::skewSymmetric(tmp_r * pts_camera_i) +
                             Utility::skewSymmetric(ric.transpose() * (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
    jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;

    // 依次存放到视觉冲投影误差 雅克比矩阵中
    jacobians_[0] = jacobian_feature;       // 2×1
    jacobians_[1] = jacobian_pose_i;        // 2×6
    jacobians_[2] = jacobian_pose_j;        // 2×6
    jacobians_[3] = jacobian_ex_pose;       // 2×6

    ///------------- check jacobians -----------------
//    {
//        std::cout << jacobians_[0] <<std::endl;
//        const double eps = 1e-6;
//        inv_dep_i += eps;
//        Eigen::Vector3d pts_camera_i = pts_i_ / inv_dep_i;
//        Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
//        Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
//        Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
//        Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
//
//        Eigen::Vector2d tmp_residual;
//        double dep_j = pts_camera_j.z();
//        tmp_residual = (pts_camera_j / dep_j).head<2>() - pts_j_.head<2>();
//        tmp_residual = information_ * tmp_residual;
//        std::cout <<"num jacobian: "<<  (tmp_residual - residual_) / eps <<std::endl;
//    }

}

void EdgeReprojectionXYZ::ComputeResidual()
{
    Vec3 pts_w = verticies_[0]->Parameters();

    VecX param_i = verticies_[1]->Parameters();
    Qd Qi(param_i[6], param_i[3], param_i[4], param_i[5]);
    Vec3 Pi = param_i.head<3>();

    Vec3 pts_imu_i = Qi.inverse() * (pts_w - Pi);
    Vec3 pts_camera_i = qic.inverse() * (pts_imu_i - tic);

    double dep_i = pts_camera_i.z();
    residual_ = (pts_camera_i / dep_i).head<2>() - obs_.head<2>();
}

void EdgeReprojectionXYZ::SetTranslationImuFromCamera(Eigen::Quaterniond &qic_, Vec3 &tic_)
{
    qic = qic_;
    tic = tic_;
}

void EdgeReprojectionXYZ::ComputeJacobians()
{
    Vec3 pts_w = verticies_[0]->Parameters();       // 第一个顶点是世界坐标系下的(X,Y,Z)坐标

    VecX param_i = verticies_[1]->Parameters();     // 第2个顶点是i帧位姿 Twi
    Qd Qi(param_i[6], param_i[3], param_i[4], param_i[5]);  // qwi旋转
    Vec3 Pi = param_i.head<3>();        // Pwi 位移

    Vec3 pts_imu_i = Qi.inverse() * (pts_w - Pi);   // world系 --> camera系 Qi.inverse() = Q_imu_w
    Vec3 pts_camera_i = qic.inverse() * (pts_imu_i - tic);

    double dep_i = pts_camera_i.z();

    Mat33 Ri = Qi.toRotationMatrix();
    Mat33 ric = qic.toRotationMatrix();
    Mat23 reduce(2, 3);
    reduce << 1. / dep_i, 0, -pts_camera_i(0) / (dep_i * dep_i),
            0, 1. / dep_i, -pts_camera_i(1) / (dep_i * dep_i);

    Eigen::Matrix<double, 2, 6> jacobian_pose_i;
    Eigen::Matrix<double, 3, 6> jaco_i;
    jaco_i.leftCols<3>() = ric.transpose() * -Ri.transpose();
    jaco_i.rightCols<3>() = ric.transpose() * Sophus::SO3d::hat(pts_imu_i);
    jacobian_pose_i.leftCols<6>() = reduce * jaco_i;

    Eigen::Matrix<double, 2, 3> jacobian_feature;
    jacobian_feature = reduce * ric.transpose() * Ri.transpose();

    jacobians_[0] = jacobian_feature;
    jacobians_[1] = jacobian_pose_i;

}


void EdgeReprojectionPoseOnly::ComputeResidual()
{
    VecX pose_params = verticies_[0]->Parameters(); // 第一个顶点是Tcw
    Sophus::SE3d pose(
            Qd(pose_params[6], pose_params[3], pose_params[4], pose_params[5]),
            pose_params.head<3>()
    );

    Vec3 pc = pose * landmark_world_;       // world系 --> camera系
    pc = pc / pc[2];    // 归一化坐标系
    Vec2 pixel = (K_ * pc).head<2>() - observation_;
    // TODO:: residual_ = ????
    residual_ = pixel;
}

void EdgeReprojectionPoseOnly::ComputeJacobians()
{
    // TODO implement jacobian here
    VecX pose_params = verticies_[0]->Parameters(); // 第一个顶点是Tcw
    Qd Qcw(pose_params[6], pose_params[3], pose_params[4], pose_params[5]);
    Vec3 Pcw = pose_params.head<3>();

    Vec3 Pc = Qcw * landmark_world_ + Pcw;

    double dep_c = Pc.z();

    Mat33 Rcw = Qcw.toRotationMatrix();   // Rwi
    Mat23 reduce(2, 3);
    reduce << 1. / dep_c, 0, -Pc(0) / (dep_c * dep_c),
            0, 1. / dep_c, -Pc(1) / (dep_c * dep_c);    // 第3讲PPT式(63)


    Eigen::Matrix<double, 2, 6> jacobian_pose;
    Eigen::Matrix<double, 3, 6> jaco;
    //jaco.leftCols<3>() = K_ * Rcw;
    //jaco.rightCols<3>() = ric.transpose() * Sophus::SO3d::hat(pts_imu_i);
    jacobian_pose.leftCols<6>() = reduce * jaco;


    jacobians_[0] = jacobian_pose;
}

}

}

