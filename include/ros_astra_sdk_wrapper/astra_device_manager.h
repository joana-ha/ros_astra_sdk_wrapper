//
// Created by turtlebot on 26.08.20.
//

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
