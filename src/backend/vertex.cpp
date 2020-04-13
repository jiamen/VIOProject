//
// Created by zlc on 2020/3/16.
//

#include "backend/vertex.h"
#include <iostream>

using namespace std;

namespace JMVIO
{

namespace backend
{

unsigned long global_vertex_id = 0;

Vertex::Vertex(int num_dimension, int local_dimension)
{
    parameters_.resize(num_dimension, 1);
    local_dimension_ = local_dimension > 0 ? local_dimension : num_dimension;
    id_ = global_vertex_id ++;

    //cout << "Vertex construct num_dimension: " << num_dimension << endl
    //     << "local_dimension: " << local_dimension << endl
    //     << "id_: " << id_ << endl;
}

Vertex::~Vertex() {}

int Vertex::Dimension() const
{
    return parameters_.rows();
}

int Vertex::LocalDimension() const
{
    return local_dimension_;
}

void Vertex::Plus(const VecX& delta)    // 向量加法
{
    parameters_ += delta;
}

}

}
