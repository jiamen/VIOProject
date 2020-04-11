//
// Created by zlc on 2020/1/18.
//

// VINS的状态估计器模块（estimator）
/*
 * 这个模块可以说是VINS的最核心模块，从论文的内容上来说，
 * 里面的内容包括了VINS的估计器初始化、基于滑动窗口的非线性优化实现紧耦合，
 * 即论文第五章（V. ESTIMATOR INITIALIZATION）
 * 第六章（VI. TIGHTLY-COUPLED MONOCULAR VIO）。
 * 此外还包括了关键帧的选择，
 * 即论文第四章（IV. MEASUREMENT PREPROCESSING A. Vision Processing Front-end） 的部分内容。
 * */

#include "estimator.h"

#include "backend/vertex_inverse_depth.h"
#include "backend/vertex_pose.h"
#include "backend/vertex_speedbias.h"
#include "backend/"
