//
// Created by zlc on 2020/3/23.
//

#ifndef _JMVIO_PROBLEM_H_
#define _JMVIO_PROBLEM_H_

#include <unordered_map>
#include <map>
#include <memory>

#include "eigen_types.h"
#include "edge.h"
#include "vertex.h"

typedef unsigned long ulong;

namespace JMVIO
{

namespace backend
{

typedef std::map<unsigned long, std::shared_ptr<Vertex>> HashVertex;
typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> HashEdge;
typedef std::unordered_map<unsigned long, std::shared_ptr<Edge>> HashVertexIdToEdge;

class Problem
{
private:
    double currentLambda;
    double currentChi_;
    double stopThresholdLM_;


public:


};



}

}


#endif // _JMVIO_PROBLEM_H_
