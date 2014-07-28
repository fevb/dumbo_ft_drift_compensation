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

FTDriftCompensation::FTDriftCompensation(FTDriftCompensationParams *params)
{
    m_params = params;
    m_t_start = ros::Time::now();
}

FTDriftCompensation::~FTDriftCompensation()
{

}

Eigen::Vector2d FTDriftCompensation::calibrate(const std::vector<geometry_msgs::WrenchStamped> &ft_gravity_compensated_measurements)
{
    unsigned int N = ft_gravity_compensated_measurements.size();
    Eigen::MatrixXd X(N, 2);
    Eigen::MatrixXd Y(N, 1);
    Eigen::Vector2d beta;

    for(unsigned int i = 0; i < N; i++)
    {
        X(i, 0) = ((ft_gravity_compensated_measurements[i].header.stamp - m_t_start).toSec())/(60.0*60.0);
        X(i, 1) = 1.0;
        Y(i, 0) = ft_gravity_compensated_measurements[i].wrench.force.z;
    }

    Eigen::MatrixXd H = X.transpose()*X;

    beta = H.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(X.transpose()*Y);

    return beta;
}

void FTDriftCompensation::compensate(const geometry_msgs::WrenchStamped &ft_gravity_compensated,
                                     geometry_msgs::WrenchStamped &ft_drift_compensated)
{
    ft_drift_compensated = ft_gravity_compensated;

    Eigen::Matrix<double, 1, 2> X;
    X(0, 0) = (ft_gravity_compensated.header.stamp-m_t_start).toSec()/(60.0*60.0);
    X(0, 1) = 1.0;

    double Y;
    Eigen::Vector2d beta = m_params->getCoefficients();

    Y = X*beta;

    ft_drift_compensated.wrench.force.z = ft_gravity_compensated.wrench.force.z-Y;
}



