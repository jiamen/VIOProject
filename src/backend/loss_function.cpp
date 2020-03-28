//
// Created by zlc on 2020/1/16.
//

#include "backend/loss_function.h"

namespace JMVIO
{

namespace backend
{
    /**
     *  Huber Loss
     *      Huber(e) = e^2      (err2 = e^T Ω e)    if e <= delta  err2 <= delta^2
     *      Huber(e) = delta * (2*e - delta)        if e >  delta
     */
    void HuberLoss::Compute(double err2, Eigen::Vector3d &rho) const
    {
        double dsqr = delta_ * delta_;

        if (err2 <= dsqr)  // inlier
        {
            rho[0] = err2;          // Huber(e) = e^2 = err2
            rho[1] = 1.;
            rho[2] = 0.;
        }
        else    // outlier
        {
            double sqrte = sqrt(err2);          // absolute value of the error
            rho[0] = 2 * sqrte * delta_ - dsqr; // Huber(e) = delta * (2*e - delta)
            rho[1] = delta_ / sqrte;            // rho'(e)  = delta / sqrt(err2)
            rho[2] = -0.5 * rho[1] / err2;      // rho''(e) = -1 / (2*e^(3/2)) = -1/2 * (delta/e^(1/2)) / e
        }
    }

    void CauchyLoss::Compute(double err2, Eigen::Vector3d &rho) const
    {
        double dsqr = delta_ * delta_;          // c^2
        double dsqrReci = 1. / dsqr;            // 1/c^2
        double aux = dsqrReci * err2 + 1.0;     // 1 + e^2/c^2
        rho[0] = dsqr * log(aux);               // c^2 * log(1+e^2/c^2)
        rho[1] = 1. / aux;                      // rho'
        rho[2] = -dsqrReci * std::pow(rho[1],2);    // rho''
    }

    void TukeyLoss::Compute(double err2, Eigen::Vector3d &rho) const
    {
        const double err = sqrt(err2);
        const double delta2 = delta_ * delta_;

        if( err <= delta_ )
        {
            const double aux = err2 / delta2;
            rho[0] = delta2 * ( 1. - std::pow((1.-aux), 3) ) / 3.;
            rho[1] = std::pow((1. - aux), 2);
            rho[2] = -2. * (1. - aux) / delta2;
        }
        else
        {
            rho[0] = delta2 / 3;
            rho[1] = 0;
            rho[2] = 0;
        }
    }

}

}


