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
#include <std_srvs/Empty.h>
#include <eigen_conversions/eigen_msg.h>
#include <dumbo_ft_drift_compensation/ft_drift_compensation.h>
#include <dumbo_ft_drift_compensation/ft_drift_compensation_params.h>
#include <boost/thread.hpp>


class FTDriftCompensationNode
{
public:
    ros::NodeHandle n_;

    ros::Subscriber topicSub_ft_;
    ros::Publisher topicPub_ft_drift_compensated_;

    ros::ServiceServer calibrate_service_server_;

    FTDriftCompensationNode()
    {
        n_ = ros::NodeHandle("~");
        m_params  = new FTDriftCompensationParams();
        m_ft_drift_compensation = new FTDriftCompensation(m_params);

        topicSub_ft_ = n_.subscribe("ft", 1, &FTDriftCompensationNode::topicCallback_ft, this);
        topicPub_ft_drift_compensated_ = n_.advertise<geometry_msgs::WrenchStamped>("ft_drift_compensated", 1);

        calibrate_service_server_ = n_.advertiseService("calibrate", &FTDriftCompensationNode::calibrateService, this);

        m_received_ft = false;
        m_calibrating = false;
        m_calibration_thread = NULL;
    }

    ~FTDriftCompensationNode()
    {
        delete m_params;
        delete m_ft_drift_compensation;
        delete m_compensation_thread;
        delete m_calibration_thread;
    }

    bool getROSParameters()
    {

        int calib_num_samples;
        n_.param<int>("calib_num_samples", calib_num_samples, 300);
        m_params->setCalibNumSamples(calib_num_samples);

        double calib_sampling_freq;
        n_.param<double>("calib_sampling_freq", calib_sampling_freq, 2.0);
        m_params->setCalibSamplingFreq(calib_sampling_freq);

        n_.param<double>("publish_rate", m_publish_rate, 650.0);

        m_compensation_thread = new boost::thread(boost::bind(&FTDriftCompensationNode::compensate,
                                                         this));

        return true;
    }

    bool calibrateService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        if(!m_calibrating)
        {
            m_calibrating = true;
            m_calibration_thread = new boost::thread(boost::bind(&FTDriftCompensationNode::calibrate, this));
        }
        return true;
    }

    void calibrate()
    {
        try
        {
            ROS_INFO("Starting FT sensor drift calibration, it will take %f minutes. DO NOT MOVE THE ROBOT!!!",
                     m_params->getCalibNumSamples()/(m_params->getCalibSamplingFreq()*60.0));
            unsigned int i = 0;
            ros::Rate loop_rate(m_params->getCalibSamplingFreq());
            std::vector<geometry_msgs::WrenchStamped> ft_measurements;
            ft_measurements.resize(m_params->getCalibNumSamples());

            while(ros::ok() && i < m_params->getCalibNumSamples())
            {
                ft_measurements[i] = m_ft;
                i++;
                loop_rate.sleep();
            }

            Eigen::Matrix<double, 2, 6> beta = m_ft_drift_compensation->calibrate(ft_measurements);
            m_params->setCoefficients(beta);

            m_calibrating = false;
            ROS_INFO("Finished FT sensor drift calibration");
            ROS_INFO("Calculated coefficients: ");
            std::cout << std::endl << beta << std::endl;
            return;
        }

        catch(boost::thread_interrupted&)
        {
            m_calibrating = false;
            return;
        }

        m_calibrating = false;
        return;
    }

    void topicCallback_ft(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
        m_ft_mutex.lock();
        m_ft = *msg;
        m_ft_mutex.unlock();
        m_received_ft = true;
    }

    void compensate()
    {
        try
        {
            while(ros::ok())
            {
                ros::Rate publish_rate(m_publish_rate);

                if(m_received_ft && !m_calibrating)
                {
                    geometry_msgs::WrenchStamped ft_drift_compensated;

                    m_ft_mutex.lock();
                    m_ft_drift_compensation->compensate(m_ft, ft_drift_compensated);
                    m_ft_mutex.unlock();

                    topicPub_ft_drift_compensated_.publish(ft_drift_compensated);
                }
                publish_rate.sleep();
            }

            return;
        }

        catch(boost::thread_interrupted&)
        {
            return;
        }
    }


private:

    FTDriftCompensationParams *m_params;
    FTDriftCompensation *m_ft_drift_compensation;

    geometry_msgs::WrenchStamped m_ft;
    bool m_received_ft;
    boost::mutex m_ft_mutex;

    double m_publish_rate;

    boost::thread *m_compensation_thread;
    boost::thread *m_calibration_thread;

    bool m_calibrating;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dumbo_ft_drift_compensation");
    FTDriftCompensationNode ft_drift_compensation_node;

    if(!ft_drift_compensation_node.getROSParameters())
    {
        ROS_ERROR("Error getting ROS parameters");
        ft_drift_compensation_node.n_.shutdown();
        return 0;
    }

    ros::spin();

    return 0;
}

