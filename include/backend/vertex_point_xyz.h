//
// Created by zlc on 2020/4/9.
//

#ifndef _JMVIO_VERTEX_POINT_XYZ_H_
#define _JMVIO_VERTEX_POINT_XYZ_H_

#pragma once

#include "vertex.h"

namespace JMVIO
{

namespace backend
{

/* *
 *  @brief 以xyz形式参数化的顶点
 * */
class VertexPointXYZ : public Vertex
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPointXYZ() : Vertex(3) {  }

    std::string TypeInfo() const
    {
        return "VertexPointXYZ";
    }
};


}

}


#endif // _JMVIO_VERTEX_POINT_XYZ_H_
