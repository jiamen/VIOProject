//
// Created by zlc on 2020/3/7.
//

#ifndef _INC_03_MYCURVEFITTING_LM_TIC_TOC_H_
#define _INC_03_MYCURVEFITTING_LM_TIC_TOC_H_

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
private:
    std::chrono::time_point<std::chrono::system_clock> start, end;

public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }
};

#endif // _INC_03_MYCURVEFITTING_LM_TIC_TOC_H_
