/*
 *  ft_drift_compensation_params.cpp
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

#include <dumbo_ft_drift_compensation/ft_drift_compensation_params.h>



FTDriftCompensationParams::FTDriftCompensationParams()
{

}

FTDriftCompensationParams::~FTDriftCompensationParams()
{

}

void FTDriftCompensationParams::setCoefficients(const Eigen::Vector2d &beta)
{
    m_beta = beta;
}

Eigen::Vector2d FTDriftCompensationParams::getCoefficients()
{
    return m_beta;
}

void FTDriftCompensationParams::setCalibNumSamples(const unsigned int &calib_num_samples)
{
    m_calib_num_samples = calib_num_samples;
}

unsigned int FTDriftCompensationParams::getCalibNumSamples()
{
    return m_calib_num_samples;
}

void FTDriftCompensationParams::setCalibSamplingFreq(const double &calib_sampling_freq)
{
    m_calib_sampling_freq = calib_sampling_freq;
}

double FTDriftCompensationParams::getCalibSamplingFreq()
{
    return m_calib_sampling_freq;
}