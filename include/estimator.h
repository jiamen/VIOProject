//
// Created by zlc on 2020/1/18.
//

#ifndef _JMVIO_ESTIMATOR_H_
#define _JMVIO_ESTIMATOR_H_

#pragma once

#include "parameters.h"
#include "feature_manager.h"
//#include "utility"
//#include "utility/tic_toc.h"


#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>


class Estimator
{
public:
    MatXX Hprior;

};


#endif // _JMVIO_ESTIMATOR_H
