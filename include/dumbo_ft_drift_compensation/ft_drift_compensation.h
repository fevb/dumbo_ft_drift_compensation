/*
 *  ft_drift_compensation.h
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

#ifndef FT_DRIFT_COMPENSATION_H_
#define FT_DRIFT_COMPENSATION_H_

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <dumbo_ft_drift_compensation/ft_drift_compensation_params.h>


class FTDriftCompensation
{
public:
    FTDriftCompensation(FTDriftCompensationParams *params);

    virtual ~FTDriftCompensation();

    // calculates the coefficients for linear time drift compensation
    // of Fz given a set of force-torque measurements using least-squares
    Eigen::Vector2d calibrate(const std::vector<geometry_msgs::WrenchStamped> &ft_gravity_compensated_measurements);

    // compensates for linear time drift (valid during 1st hour of sensor operation)
    void compensate(const geometry_msgs::WrenchStamped &ft_gravity_compensated,
                    geometry_msgs::WrenchStamped &ft_drift_compensated);

private:
    FTDriftCompensationParams *m_params;
    ros::Time m_t_start;
};


#endif
