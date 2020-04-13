//
// Created by zlc on 2020/1/14.
//

#ifndef _JMVIO_LOSS_FUNCTION_H_
#define _JMVIO_LOSS_FUNCTION_H_


#include "eigen_types.h"

namespace JMVIO
{

namespace backend
{

/**
 * loss function 即鲁棒核函数
 * loss套在误差之上
 * 假设某条边的残差为r，信息矩阵为I, 那么平方误差为r^T*I*r，令它的开方为e
 * 那么loss就是Compute(e)
 * 在求导时，也必须将loss function放到求导的第一项
 *
 * LossFunction是各核函数的基类，它可以派生出各种Loss
 */

/**
 * compute the scaling factor for a error
 *
 * The error is e^T Ω e  残差项
 *
 * The output rho is
 *    rho[0]: The actual scaled error value             实际标度误差值
 *    rho[1]: First derivative of the scaling function  标度函数的一阶倒数
 *    rho[2]: Second derivative of the scaling function 标度函数的二阶导数
 *
 * LossFunction 是各核函数的基类, 它可以派生出各种Loss
 */

class LossFunction
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;   // 字节对齐问题

    virtual ~LossFunction() { }

    // virtual double Compute (double error) const = 0;
    virtual void Compute(double err2, Eigen::Vector3d& rho) const = 0;
};

/**
 *  common loss, do nothing
 *  使用 nullptr 作为 Loss function 时效果相同
 *
 *  TrivialLoss(e) = e^2   trivialLoss 轻微损失
 */
class TrivialLoss : public LossFunction
{
public:
    virtual void Compute(double err2, Eigen::Vector3d& rho) const override
    {
        // TODO:: whether multiply 1/2
        rho[0] = err2;
        rho[1] = 1;
        rho[2] = 0;
    }
};

/**
 *  Huber Loss
 *      Huber(e) = e^2      (err2 = e^T Ω e)    if e <= delta  err2 <= delta^2
 *      Huber(e) = delta * (2*e - delta)        if e >  delta
 */
class HuberLoss : public LossFunction
{
private:
    double delta_;

public:
    explicit HuberLoss(double delta) : delta_(delta) { }

    virtual void Compute(double err2, Eigen::Vector3d& rho) const override;
};

/**
 *  Cauchy Loss
 */
class CauchyLoss : public LossFunction
{
private:
    double delta_;

public:
    explicit CauchyLoss(double delta) : delta_(delta) { }

    virtual void Compute(double err2, Eigen::Vector3d& rho) const override;

};

/**
 *  Tukey Loss
 */
class TukeyLoss : public LossFunction
{
private:
    double delta_;

public:
    explicit TukeyLoss(double delta) : delta_(delta) { }

    virtual void Compute(double err2, Eigen::Vector3d& rho) const override;
};

}

}

#endif // _JMVIO_LOSS_FUNCTION_H_
