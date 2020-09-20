//
// Created by turtlebot on 25.08.20.
//

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
