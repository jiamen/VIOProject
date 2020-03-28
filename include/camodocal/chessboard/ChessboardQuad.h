//
// Created by zlc on 2020/1/13.
//

#ifndef _JMVIO_CHESSBOARDQUAD_H_
#define _JMVIO_CHESSBOARDQUAD_H_

#include <boost/shared_ptr.hpp>
#include "camodocal/chessboard/ChessboardCorner.h"

namespace  camodocal
{

class ChessboardQuad;
typedef boost::shared_ptr<ChessboardQuad> ChessboardQuadPtr;


class ChessboardQuad
{
public:
    int count;
    int group_idx;
    float edge_len;
    ChessboardCornerPtr corners[4];
    ChessboardQuadPtr neighbors[4];
    bool labeled;

    ChessboardQuad() : count(0), group_idx(-1), edge_len(FLT_MAX), labeled(false) {  }

};

}

#endif //JMVIO_CHESSBOARDQUAD_H
