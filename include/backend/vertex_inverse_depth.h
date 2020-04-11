//
// Created by zlc on 2020/4/9.
//

#ifndef _JMVIO_VERTEX_INVERSE_DEPTH_H_
#define _JMVIO_VERTEX_INVERSE_DEPTH_H_

#pragma once

#include "vertex.h"


namespace JMVIO
{

namespace backend
{

/*
 *  以逆深度形式存储的顶点
 * */
class VertexInverseDepth : public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexInverseDepth() : Vertex(1) {  }

    virtual std::string TypeInfo() const
    {
        return "VertexInverseDepth";
    }

};

}

}


#endif // _JMVIO_VERTEX_INVERSE_DEPTH_H_
