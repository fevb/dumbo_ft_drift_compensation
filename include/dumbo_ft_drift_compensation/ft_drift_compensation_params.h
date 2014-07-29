/*
 *  ft_drift_compensation_params.h
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

#ifndef FT_DRIFT_COMPENSATION_PARAMS_H_
#define FT_DRIFT_COMPENSATION_PARAMS_H_

#include <eigen3/Eigen/Core>


class FTDriftCompensationParams
{
public:
    FTDriftCompensationParams();

    virtual ~FTDriftCompensationParams();

    // beta: 6 columns, 1 for each component of FT measurements
    // [Fx Fy Fz Tx Ty Tz]
    void setCoefficients(const Eigen::Matrix<double, 2, 6> &beta);
    Eigen::Matrix<double, 2, 6> getCoefficients();

    void setCalibNumSamples(const unsigned int &calib_num_samples);
    unsigned int getCalibNumSamples();

    void setCalibSamplingFreq(const double &calib_sampling_freq);
    double getCalibSamplingFreq();


private:

    // coefficients for drift compensation using accelerometer
    Eigen::Matrix<double, 2, 6> m_beta;

    // number of samples to take for calibrating the
    // parameters (linear coefficients) for time-drift compensation
    unsigned int m_calib_num_samples;

    // sampling frequency for calibrating the
    // parameters (linear coefficients) for time-drift compensation
    double m_calib_sampling_freq;
};

#endif
