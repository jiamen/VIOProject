//
// Created by zlc on 2020/3/23.
//

#ifndef _JMVIO_EDGE_H_
#define _JMVIO_EDGE_H_

#include <memory>
#include <string>
#include "eigen_types.h"
#include <eigen3/Eigen/Eigen>
#include "loss_function.h"

namespace JMVIO
{

namespace backend
{

class Vertex;

class Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    /**
     *  构造函数, 会自动化配雅克比的空间
     *  @param residual_dimension 残差维度
     *  @param num_verticies   顶点数量
     *  @param verticies_types 顶点类型名称, 可以不给, 不给的话check中不会检查
     * */
    explicit Edge(int residual_dimension, int num_verticies,
                  const std::vector<std::string> &verticies_types = std::vector<std::string>());

    virtual ~Edge();

    // 返回id
    unsigned long Id() const { return id_; }
};

}

}



#endif // _JMVIO_EDGE_H_
