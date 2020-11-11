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

#ifndef SRC_ASTRA_DEVICE_MANAGER_H
#define SRC_ASTRA_DEVICE_MANAGER_H

//Astra SDK
#include <astra/capi/astra.h>
#include <astra/astra.hpp>
#include <astra/astra.hpp>
#include <astra/streams/Image.hpp>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//ros
#include <sensor_msgs/distortion_models.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

//boost
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>

//internal
#include "ros_astra_sdk_wrapper/astra_stream_manager.h"

namespace ros_astra_sdk_wrapper{

typedef boost::function<void(sensor_msgs::ImagePtr image)> FrameCallbackFunction;

class AstraDeviceManager {

//static void loop();

public:
    bool stream_initialized_;
    bool color_stream_initialized_;
    bool depth_stream_initialized_;
    bool body_stream_initialized_;

    BodyStreamListener body_stream_listener_;

    AstraDeviceManager();

    virtual ~AstraDeviceManager();

    static boost::shared_ptr<AstraDeviceManager> getSingelton();

    void startColorStream();
    void startDepthStream();
    void startBodyStream();

    void setColorFrameCallback(FrameCallbackFunction callback);
    void setDepthFrameCallback(FrameCallbackFunction callback);
    void setBodyFrameCallback(BodyFrameCallbackFunction callback);

    sensor_msgs::CameraInfoPtr getColorCamInfo();
    sensor_msgs::CameraInfoPtr getDepthCamInfo();

    void shutdown();


private:

    //for getting calibration params
    sensor_msgs::CameraInfoPtr info_;

    const char* licenseString_;
    astra::StreamSet streamSet_;
    astra::StreamReader reader_;

    astra::ImageStreamMode colorMode_;
    astra::ImageStreamMode depthMode_;

    orbbec_camera_params param_;
    bool do_shutdown_;

    DepthStreamListener depth_stream_listener_;
    ColorStreamListener color_stream_listener_;

    bool color_calibration_received_;
    bool depth_calibration_received_;

    void loop();

    boost::thread* t_;

protected:
    static boost::shared_ptr<AstraDeviceManager> singelton_;


};

}


#endif //SRC_ASTRA_DEVICE_MANAGER_H
