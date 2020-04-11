//
// Created by zlc on 2020/4/9.
//

#ifndef _JMVIO_VERTEX_POSE_H_
#define _JMVIO_VERTEX_POSE_H_

#pragma once

#include <memory>
#include "vertex.h"

namespace JMVIO
{

namespace backend
{

/*
 * Pose vertex
 * parameters: tx, ty, tz, qx, qy, qz, qw, 7DoF
 * optimization is perform on mainfold, so update is 6 DoF, left multiplication
 *
 * pose is represented as Twb in VIO case
 * */
class VertexPose : public Vertex
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /*
     * 构造函数的两项参数为其变量的维数和实际自由度，比如姿态有6个自由度但是由于使用了四元数表示所以一共7维，这里就是7和6。同时对加法进行了override：
     * */
    VertexPose() : Vertex(7, 6) {  }

    // 加法, 可重定义
    // 默认是向量加
    virtual void Plus(const VecX &delta) override;

    std::string TypeInfo() const
    {
        return "VertexPose";
    }

    /**
     * 需要维护[H|b]矩阵中的如下数据块
     * p: pose, m:mappoint
     *
     *     Hp1_p2
     *     Hp2_p2    Hp2_m1    Hp2_m2    Hp2_m3     |    bp2
     *
     *                         Hm2_m2               |    bm2
     *                                   Hm2_m3     |    bm3
     * 1. 若该Camera为source camera，则维护vHessionSourceCamera；
     * 2. 若该Camera为measurement camera, 则维护vHessionMeasurementCamera；
     * 3. 并一直维护m_HessionDiagonal；
     */
};

}

}

#endif // _JMVIO_VERTEX_POSE_H_
