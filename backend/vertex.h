//
// Created by zlc on 2020/3/4.
//

#ifndef _INC_03_MYCURVEFITTING_LM_VERTEX_H_
#define _INC_03_MYCURVEFITTING_LM_VERTEX_H_

#include <backend/eigen_types.h>

namespace myslam
{

namespace backend
{

class Vertex
{
protected:
    VecX            parameters_;        // 实际存储的变量值
    int             local_dimension_;   // 局部参数化维度
    unsigned long   id_;                // 顶点的id, 自动生成

    // ordering id 是在problem中排序后的id, 用于寻找雅克比矩阵
    // ordering id 带有维度信息, 例如 ordering_id=6 则对应Hessian中的第6例
    // 从零开始
    unsigned long   ordering_id_ = 0;

    bool fixed_ = false;    // 是否固定

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 构造函数, num_dimension；顶点自身维度;
    // local_dimension: 本地参数化维度, 为-1时认为与本身维度一样
    explicit Vertex(int num_dimension, int local_dimension = -1);

    virtual ~Vertex();

    // 返回变量维度
    int Dimension() const;

    // 返回变量本地维度 ☆
    int LocalDimension() const;

    // 该顶点的id
    unsigned long Id() const
    {
        return id_;
    }

    // 返回参数值
    VecX Parameters() const { return parameters_; }

    // 返回参数值的引用
    VecX &Parameters() { return parameters_; }

    // 设置参数值
    void SetParameters(const VecX &params) { parameters_ = params; }

    // 加法, 可重定义, 默认是向量加
    virtual void Plus(const VecX &delta);

    // 返回顶点的名称, 在子类中实现
    virtual std::string TypeInfo() const = 0;

    int OrderingId() const { return ordering_id_; }

    void SetOrderingId(unsigned long id) { ordering_id_ = id; }

    // 固定该点的估计值
    void SetFixed(bool fixed = true)
    {
        fixed_ = fixed;
    }

    // 测试该点是否被固定
    bool IsFixed() const { return fixed_; }

};

}

}


#endif // _INC_03_MYCURVEFITTING_LM_VERTEX_H_
