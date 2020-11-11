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

#ifndef SRC_ASTRA_STREAM_MANAGER_H
#define SRC_ASTRA_STREAM_MANAGER_H

//Astra SDK
#include <astra/capi/astra.h>
#include <astra/astra.hpp>
#include <astra/streams/Image.hpp>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//ros
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

//boost
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

//internal
#include <ros_astra_sdk_wrapper/BodyTracking.h>

#define KEY_JOINT_TO_TRACK    ASTRA_JOINT_RIGHT_HIP

namespace ros_astra_sdk_wrapper{

typedef boost::function<void(sensor_msgs::ImagePtr image)> FrameCallbackFunction;
typedef boost::function<void(BodyTracking skeleton)> BodyFrameCallbackFunction;

class DepthStreamListener : public astra::FrameListener {

public:
    DepthStreamListener();

    ~DepthStreamListener();

    void setCallback(FrameCallbackFunction& func){
        callback_ = func;
    }

private:
    bool initialized_;
    float prev_time_stamp_;

    FrameCallbackFunction callback_;

    void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override;


};

    class ColorStreamListener : public astra::FrameListener {

    public:
        ColorStreamListener();

        ~ColorStreamListener();

        void setCallback(FrameCallbackFunction& func){
            callback_ = func;
        }

    private:
        bool initialized_;
        float prev_time_stamp_;

        FrameCallbackFunction callback_;

        void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override;


    };

   class BodyStreamListener : public astra::FrameListener {

    public:
        BodyStreamListener();

        ~BodyStreamListener();

        void setCallback(BodyFrameCallbackFunction& func){
            callback_ = func;
        }

       void setCurrent_robot_vel(double velocity);

    private:
        std::string name_;
        ros::Time prev_time_stamp_;
        int prev_valid_body_id_;
        //double current_robot_vel_;
        BodyTracking prev_skeleton_data_;

        BodyFrameCallbackFunction callback_;

        void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override;
        BodyTracking output_bodyframe(astra::Body body, int frame_index);

    };
};



#endif //SRC_ASTRA_STREAM_MANAGER_H
