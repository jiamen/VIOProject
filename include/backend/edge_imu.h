//
// Created by zlc on 2020/4/12.
//

#ifndef _JMVIO_EDGE_IMU_H_
#define _JMVIO_EDGE_IMU_H_

#include <memory>
#include <string>
#include "../thirdparty/Sophus/sophus/se3.hpp"

#include "edge.h"
#include "../factor/intergration_base.h"

namespace JMVIO
{

namespace backend
{

/*
 *  此边是IMU误差, 此边为4元边, 与之相连的顶点有: Pi Mi Pj Mj
 * */
class EdgeImu : public Edge
{
private:
    enum StateOrder
    {
        O_P = 0,
        O_R = 3,
        O_V = 6,
        O_BA = 9,
        O_BG = 12
    };
    IntegrationBase* pre_integration_;  // IMU预积分值
    static Vec3 gravity_;               // 重力向量

    Mat33 dp_dba_ = Mat33::Zero();
    Mat33 dp_dbg_ = Mat33::Zero();
    Mat33 dr_dbg_ = Mat33::Zero();
    Mat33 dv_dba_ = Mat33::Zero();
    Mat33 dv_dbg_ = Mat33::Zero();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit EdgeImu(IntegrationBase* _pre_integration) : pre_integration_(_pre_integration),
            Edge(15, 4, std::vector<std::string>{"VertexPose", "VertexSpeedBias", "VertexPose", "VertexSpeedBias"})
    {
        //    if (pre_integration_) {
        //        pre_integration_->GetJacobians(dr_dbg_, dv_dbg_, dv_dba_, dp_dbg_, dp_dba_);
        //        Mat99 cov_meas = pre_integration_->GetCovarianceMeasurement();
        //        Mat66 cov_rand_walk = pre_integration_->GetCovarianceRandomWalk();
        //        Mat1515 cov = Mat1515::Zero();
        //        cov.block<9, 9>(0, 0) = cov_meas;
        //        cov.block<6, 6>(9, 9) = cov_rand_walk;
        //        SetInformation(cov.inverse());
        //    }
    }

    // 返回边的类型信息
    virtual std::string TypeInfo() const override { return "EdgeImu"; }

    // 计算残差
    virtual void ComputeResidual() override;

    // 计算雅可比
    virtual void ComputeJacobians() override;

    //    static void SetGravity(const Vec3 &g) {
    //        gravity_ = g;
    //    }
};

}

}



#endif // _JMVIO_EDGE_IMU_H_