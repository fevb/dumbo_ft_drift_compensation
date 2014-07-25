/*
 *  ft_drift_compensation_node.cpp
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


#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <dumbo_ft_drift_compensation/ft_drift_compensation.h>
#include <dumbo_ft_drift_compensation/ft_drift_compensation_params.h>


class FTDriftCompensationNode
{
public:
    ros::NodeHandle n_;

    ros::Subscriber topicSub_ft_compensated_;
    ros::Publisher topicPub_ft_drift_compensated_;

    FTDriftCompensationNode()
    {
        n_ = ros::NodeHandle("~");
        m_params  = new FTDriftCompensationParams();
        m_ft_drift_compensation = new FTDriftCompensation(m_params);

        topicSub_ft_compensated_ = n_.subscribe("ft_compensated", 1, &FTDriftCompensationNode::topicCallback_ft_compensated, this);
        topicPub_ft_drift_compensated_ = n_.advertise<geometry_msgs::WrenchStamped>("ft_drift_compensated", 1);
    }

    ~FTDriftCompensationNode()
    {
        delete m_params;
        delete m_ft_drift_compensation;
    }

    bool getROSParameters()
    {

        /// Get coefficients
        XmlRpc::XmlRpcValue coefficientsXmlRpc;
        Eigen::Vector2d coefficients;
        if (n_.hasParam("coefficients"))
        {
            n_.getParam("coefficients", coefficientsXmlRpc);
        }

        else
        {
            ROS_ERROR("Parameter 'coefficients' not set, shutting down node...");
            n_.shutdown();
            return false;
        }


        if(coefficientsrXmlRpc.size()!=6)
        {
            ROS_ERROR("Invalid F/T coefficients parameter size (should be size 6), shutting down node");
            n_.shutdown();
            return false;
        }

        for (int i = 0; i < coefficientsXmlRpc.size(); i++)
        {
            coefficients(i) = (double)coefficientsXmlRpc[i];
        }

        int calib_num_samples;
        n_.param<int>("calib_num_samples", calib_num_samples, 300);

        double calib_sampling_freq;
        n_.param<double>("calib_sampling_freq", calib_sampling_freq, 2.0);

    }

    void topicCallback_ft_compensated(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
        m_ft_compensated = *msg;
    }


protected:

    FTDriftCompensationParams *m_params;
    FTDriftCompensation *m_ft_drift_compensation;

    geometry_msgs::WrenchStamped m_ft_compensated;

};

int main(int argc, char **argv)
{



    return 0;
}

