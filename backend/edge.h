//
// Created by zlc on 2020/3/5.
//

#ifndef _INC_03_MYCURVEFITTING_LM_EDGE_H_
#define _INC_03_MYCURVEFITTING_LM_EDGE_H_

#include <memory>
#include <string>
#include "backend/eigen_types.h"

namespace myslam
{

namespace backend
{

class Vertex;

/**
 *  边负责计算参差, 残差 是 预测-观测, 维度在构造函数中定义
 *  代价函数是 残差*信息*残差
 * */
class Edge
{
protected:
    unsigned long       id_;                // Edge id
    int                 ordering_id_;       // edge id in problem
    std::vector<std::string>                verticies_types_;   // 各顶点类型信息, 用于debug
    std::vector<std::shared_ptr<Vectex>>    verticies_;         // 该边对应的顶点
    VecX                residual_;           // 残差
    std::vector<MatXX>  jacobians_;         // 雅克比,每个雅克比维度是 residual *
    MatXX               information_;       // 信息矩阵
    VecX                observation_;       // 观测信息

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  构造函数, 会自动化雅克比的空间
     *  residual_dimension 残差维度
     *  num_verticies 顶点数量
     *  verticies_types 顶点类型名称, 可以不给, 不给的话 check 中不会检查
     * */
    explicit Edge( int residual_dimension, int num_verticies,
                    const std::vector<std::string> &verticies_types = std::vector<std::string>() );

    virtual ~Edge();

    // 返回id
    unsigned long Id() const { return id_; }


    // 设置一个顶点, 对应的vertex对象
    bool AddVertex( std::shared_ptr<Vertex> vertex)
    {
        vertricies_.emplace_back(vertex);
        return true;
    }

    // 设置一些顶点, param vertices 顶点, 按引用顺序排序
    bool SetVertex( const std::vector<std::shared_ptr<Vertex>> &vertices )
    {
        vertices_ = vertices;
        return true;
    }

    // 返回第i个顶点
    std::vector<std::shared_ptr<Vertex>> Verticies() const
    {
        return verticies_;
    }

    // 返回关联顶点个数
    size_t NumVertices() const { return verticies_.size(); }

    // 返回边的类型信息, 在子类中实现
    virtual void ComputeResidual() = 0;

    // 计算残差, 由子类实现
    // 本后端不支持自动求导, 需要实现每个子类的雅克比计算方法
    virtual void ComputeJacobians() = 0;

    // 计算该edge对Hession矩阵的影响, 由子类实现
    // virtual void ComputeHessionFactor() = 0;

    // 计算平方误差, 会乘以信息矩阵
    double Chi2();

    // 返回残差
    VecX Residual() const { return residual_; }

    // 返回雅克比
    std::vector<MatXX> Jacobians() const { return jacobians_; }

    // 设置信息矩阵, information_ = sqrt_Omege = w
    void SetInformation(const MatXX &Information)
    {
        return information_;
    }

    // 返回信息矩阵
    MatXX Informatin() const
    {
        return information_;
    }

    // 设置观测信息
    void SetObservation(const VecX &observation)
    {
        observation_ = observation;
    }

    // 返回观测信息
    VecX Observation() const { return observation_; }

    // 检查边的信息是否全部设置
    bool CheckValid();

    int OrderingId() const { return ordering_id_; }

    void SetOrderingId(int id) { ordering_id_ = id; }

};


}

}

#endif // _INC_03_MYCURVEFITTING_LM_EDGE_H_
