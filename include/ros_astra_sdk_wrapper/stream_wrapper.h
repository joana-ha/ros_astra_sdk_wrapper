//
// Created by turtlebot on 25.08.20.
//

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
