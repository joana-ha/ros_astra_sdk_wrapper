/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Joana Haase
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Joana Haase
 *********************************************************************/

#ifndef SRC_STREAM_WRAPPER_H
#define SRC_STREAM_WRAPPER_H

//Astra SDK
#include <astra/capi/astra.h>
#include <astra/astra.hpp>
#include <astra/streams/Image.hpp>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>

//boost
#include <boost/thread/thread.hpp>

//internal
#include "ros_astra_sdk_wrapper/astra_stream_manager.h"
#include "ros_astra_sdk_wrapper/astra_device_manager.h"
#include <ros_astra_sdk_wrapper/BodyTracking.h>

namespace ros_astra_sdk_wrapper  {
class StreamWrapper {

public:
    StreamWrapper(ros::NodeHandle& n, ros::NodeHandle& pnh);

    ~StreamWrapper();

    void newColorFrameCallback(sensor_msgs::ImagePtr image);
    void newDepthFrameCallback(sensor_msgs::ImagePtr image);
    void newBodyFrameCallback(BodyTracking skeleton);

private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;

    boost::shared_ptr<AstraDeviceManager> device_manager_;

    //config
    std::string ir_frame_id_;
    std::string color_frame_id_;
    std::string depth_frame_id_ ;
    std::string color_info_url_, ir_info_url_;
    std::string device_id_;
    ros::Duration color_time_offset_;
    ros::Duration depth_time_offset_;

    //publishers
    image_transport::CameraPublisher pub_color_;
    image_transport::CameraPublisher pub_depth_;
    image_transport::CameraPublisher pub_depth_raw_;
    ros::Publisher pub_body_;

    //subscribers
    //ros::Subscriber sub_vel_;

    //num of publisher subscribers
    bool depth_raw_subscribers_;

    //threading with boost
    boost::mutex connect_mutex_;

    void advertiseROSTopics();

    void colorConnectCallback();
    void depthConnectCallback();
    void bodyConnectCallback();

    void readConfigFromParameterServer();
    //void retrieveRobotVelocity();
    //void newVelocityCallback(const geometry_msgs::Twist& twist);

    sensor_msgs::CameraInfoPtr getColorCameraInfo(int width, int height, ros::Time time) const;
    sensor_msgs::CameraInfoPtr getDepthCameraInfo(int width, int height, ros::Time time) const;

    sensor_msgs::ImageConstPtr rawToFloatingPointConversion(sensor_msgs::ImageConstPtr raw_image);
};
};


#endif //SRC_STREAM_WRAPPER_H
