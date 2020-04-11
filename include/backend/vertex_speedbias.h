//
// Created by zlc on 2020/4/9.
//

#ifndef _JMVIO_VERTEX_SPEEDBIAS_H_
#define _JMVIO_VERTEX_SPEEDBIAS_H_

#include <memory>
#include "vertex.h"

namespace JMVIO
{

namespace backend
{

/*
 * SpeedBias vertex
 * parameters: v, ba, bg 9 DOF
 * */

class VertexSpeedBias : public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexSpeedBias() : Vertex(9) {  }

    std::string TypeInfo() const
    {
        return "VertexSpeedBias";
    }
};

}

}

#endif // _JMVIO_VERTEX_SPEEDBIAS_H_
