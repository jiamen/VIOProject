//
// Created by zlc on 2020/1/13.
//

#ifndef _JMVIO_CHESSBOARDCORNER_H_
#define _JMVIO_CHESSBOARDCORNER_H_

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

namespace camodocal
{
class ChessboardCorner;
typedef boost::shared_ptr<ChessboardCorner> ChessboardCornerPtr;

class ChessboardCorner
{
public:
    cv::Point2f pt;         // x and y coordinates
    int row;
    int column;             // Row and Column of the corner in the found pattern
    bool needsNeighbor;     // Does the corner require a neighbor?
    int count;              // number of corner neighbors
    ChessboardCornerPtr neighbors[4];       // pointer to all corner neighbors


    ChessboardCorner() : row(0), column(0), needsNeighbor(true), count(0) { }

    float meanDist(int &n) const
    {
        float sum = 0;
        n = 0;

        for(int i = 0; i < 4; ++ i )
        {
            if ( neighbors[i].get() )
            {
                float dx = neighbors[i]->pt.x - pt.x;
                float dy = neighbors[i]->pt.y - pt.y;

                sum += sqrt( dx*dx + dy*dy  );      // calculate Euclidean distance
                n ++;
            }
        }

        return sum / std::max(n, 1);
    }

};

}

#endif // _JMVIO_CHESSBOARDCORNER_H_
