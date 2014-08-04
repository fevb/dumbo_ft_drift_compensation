/*
 *  ft_drift_compensation.cpp
 *
 *
 *  Created on: Jul 25, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <dumbo_ft_drift_compensation/ft_drift_compensation.h>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

FTDriftCompensation::FTDriftCompensation(FTDriftCompensationParams *params)
{
    m_params = params;
    m_t_start = ros::Time::now();
}

FTDriftCompensation::~FTDriftCompensation()
{

}

Eigen::Matrix<double, 2, 6> FTDriftCompensation::calibrate(const std::vector<geometry_msgs::WrenchStamped> &ft_measurements)
{
    unsigned int N = ft_measurements.size();
    Eigen::MatrixXd X(N, 2);
    Eigen::MatrixXd Y(N, 6);
    Eigen::Matrix<double, 2, 6> beta = Eigen::Matrix<double, 2, 6>::Zero();

    for(unsigned int i = 0; i < N; i++)
    {
        X(i, 0) = ((ft_measurements[i].header.stamp - m_t_start).toSec())/(60.0*60.0);
        X(i, 1) = 1.0;

        Eigen::Matrix<double, 6, 1> w;
        tf::wrenchMsgToEigen(ft_measurements[i].wrench, w);
        Y.row(i) = w.transpose();
    }

    Eigen::MatrixXd H = X.transpose()*X;

    for(unsigned int i = 0; i < 6; i++)
    {
        if(i==2 || i==5)
        {
            beta.col(i) = H.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(X.transpose()*Y.col(i));
        }
        else
        {
            beta(1, i) = Y.col(i).sum()/N;
        }
    }

    return beta;
}

void FTDriftCompensation::compensate(const geometry_msgs::WrenchStamped &ft,
                                     geometry_msgs::WrenchStamped &ft_drift_compensated)
{
    ft_drift_compensated = ft;

    Eigen::Matrix<double, 1, 2> X;
    X(0, 0) = (ft.header.stamp-m_t_start).toSec()/(60.0*60.0);
    X(0, 1) = 1.0;

    Eigen::Matrix<double, 1, 6> Y;
    Eigen::Matrix<double, 2, 6> beta = m_params->getCoefficients();

    Y = X*beta;

    Eigen::Matrix<double, 6, 1> w, w_drift_compensated;
    tf::wrenchMsgToEigen(ft.wrench, w);

    w_drift_compensated = w - Y.transpose();

    tf::wrenchEigenToMsg(w_drift_compensated, ft_drift_compensated.wrench);
}



