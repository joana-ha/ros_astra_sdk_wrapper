//
// Created by turtlebot on 26.08.20.
//

#include "ros_astra_sdk_wrapper/astra_device_manager.h"

namespace ros_astra_sdk_wrapper{

    boost::shared_ptr<AstraDeviceManager> AstraDeviceManager::singelton_;

    void AstraDeviceManager::loop(){
        do{
            astra_update();
            boost::this_thread::yield();
        }while(!do_shutdown_);

    }

    AstraDeviceManager::AstraDeviceManager() : do_shutdown_(false), stream_initialized_(false), color_stream_initialized_(false), depth_stream_initialized_(false), color_calibration_received_(false), depth_calibration_received_(false), param_(), colorMode_(), depthMode_(), depth_stream_listener_(DepthStreamListener()), color_stream_listener_(ColorStreamListener())
    {

        info_ = boost::make_shared<sensor_msgs::CameraInfo>();
        licenseString_ = "";

        //set_key_handler();
        for(int i=0; i < 100; i++){
            boost::this_thread::yield();
        }

        astra::initialize();
        orbbec_body_tracking_set_license(licenseString_);
        colorMode_.set_width(640);
        colorMode_.set_height(480);
        colorMode_.set_fps(30);
        colorMode_.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);

        depthMode_.set_width(640);
        depthMode_.set_height(480);
        depthMode_.set_fps(30);
        depthMode_.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
        reader_ = streamSet_.create_reader();
        if(streamSet_.is_available()){
            astra::DeviceController astra_device_info(streamSet_);
            ROS_INFO("streamset available");
            astra_device_info.get_orbbec_camera_params( param_);
        }else{
            ROS_INFO("streamset not availablle");
        }

        t_ = new boost::thread(boost::bind(&AstraDeviceManager::loop, this));

        //t_(&AstraDeviceManager::loop);

        stream_initialized_ = true;

    }

    AstraDeviceManager::~AstraDeviceManager() {

    }

    boost::shared_ptr<AstraDeviceManager> AstraDeviceManager::getSingelton()
    {
        if (singelton_.get()==0)
            singelton_ = boost::make_shared<AstraDeviceManager>();

        return singelton_;
    }

    void AstraDeviceManager::setColorFrameCallback(FrameCallbackFunction callback)
    {
        color_stream_listener_.setCallback(callback);
    }

    void AstraDeviceManager::setDepthFrameCallback(FrameCallbackFunction callback)
    {
        depth_stream_listener_.setCallback(callback);
    }

    void AstraDeviceManager::setBodyFrameCallback(BodyFrameCallbackFunction callback)
    {
        body_stream_listener_.setCallback(callback);
    }


    void AstraDeviceManager::startColorStream() {
        auto colorStream = reader_.stream<astra::ColorStream>();
        colorStream.set_mode(colorMode_);
        if(colorStream.mirroring_enabled()) {
            colorStream.enable_mirroring(false);
        } else {
            colorStream.enable_mirroring(true);
        }
        colorStream.start();
        color_stream_initialized_ = true;
        reader_.add_listener(color_stream_listener_);

    }

    void AstraDeviceManager::startDepthStream() {
        auto depthStream = reader_.stream<astra::DepthStream>();
        depthStream.set_mode(colorMode_);
        if(depthStream.mirroring_enabled()) {
            depthStream.enable_mirroring(false);
        } else {
            depthStream.enable_mirroring(true);
        }
        depthStream.start();
        depth_stream_initialized_ = true;
        reader_.add_listener(depth_stream_listener_);
    }


    void AstraDeviceManager::startBodyStream() {
        auto bodyStream = reader_.stream<astra::BodyStream>();
        bodyStream.start();
        body_stream_initialized_ = true;
        reader_.add_listener(body_stream_listener_);
    }

    sensor_msgs::CameraInfoPtr AstraDeviceManager::getColorCamInfo(){

        if(!color_calibration_received_){

            //height, width
            info_->height = colorMode_.height();
            info_->width = colorMode_.width();

            //distortion parameters matrix D
            info_->D.push_back(param_.l_k[0]);
            info_->D.push_back(param_.l_k[1]);
            info_->D.push_back(param_.l_k[2]);
            info_->D.push_back(param_.l_k[3]);
            info_->D.push_back(param_.l_k[4]);

            //calibration matrix K
            info_->K[0] = param_.l_intr_p[0];
            info_->K[1] = 0;
            info_->K[2] = param_.l_intr_p[2];
            info_->K[3] = 0;
            info_->K[4] = param_.l_intr_p[1];
            info_->K[5] = param_.l_intr_p[3];
            info_->K[6] = 0;
            info_->K[7] = 0;
            info_->K[8] = 1;

            //rectify matrix R
            info_->R[0] = 1.0;
            info_->R[1] = 0.0;
            info_->R[2] = 0.0;
            info_->R[3] = 0.0;
            info_->R[4] = 1.0;
            info_->R[5] = 0.0;
            info_->R[6] = 0.0;
            info_->R[7] = 0.0;
            info_->R[8] = 1.0;

            //projection matrix P is same as K for color camera
            info_->P[0] = param_.l_intr_p[0];
            info_->P[1] = 0;
            info_->P[2] = param_.l_intr_p[2];
            info_->P[3] = 0;
            info_->P[4] = 0;
            info_->P[5] = param_.l_intr_p[1];
            info_->P[6] = param_.l_intr_p[3];
            info_->P[7] = 0;
            info_->P[8] = 0;
            info_->P[9] = 0;
            info_->P[10] = 1;
            info_->P[11] = 0;

            info_->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

            color_calibration_received_ = true;
            return info_;
        }else{
            return info_;
        }
    }


    sensor_msgs::CameraInfoPtr AstraDeviceManager::getDepthCamInfo(){

        if(!depth_calibration_received_){

            //height, width
            info_->height = depthMode_.height();
            info_->width = depthMode_.width();

            //distortion parameters matrix D
            info_->D.push_back(param_.r_k[0]);
            info_->D.push_back(param_.r_k[1]);
            info_->D.push_back(param_.r_k[2]);
            info_->D.push_back(param_.r_k[3]);
            info_->D.push_back(param_.r_k[4]);

            //calibration matrix K
            info_->K[0] = param_.r_intr_p[0];
            info_->K[1] = 0;
            info_->K[2] = param_.r_intr_p[2];
            info_->K[3] = 0;
            info_->K[4] = param_.r_intr_p[1];
            info_->K[5] = param_.r_intr_p[3];
            info_->K[6] = 0;
            info_->K[7] = 0;
            info_->K[8] = 1;

            //rectify matrix R
            info_->R[0] = 1.0;
            info_->R[1] = 0.0;
            info_->R[2] = 0.0;
            info_->R[3] = 0.0;
            info_->R[4] = 1.0;
            info_->R[5] = 0.0;
            info_->R[6] = 0.0;
            info_->R[7] = 0.0;
            info_->R[8] = 1.0;

            //projection matrix P is same as K for color camera
            info_->P[0] = param_.r_intr_p[0];
            info_->P[1] = 0;
            info_->P[2] = param_.r_intr_p[2];
            info_->P[3] = 0;
            info_->P[4] = 0;
            info_->P[5] = param_.r_intr_p[1];
            info_->P[6] = param_.r_intr_p[3];
            info_->P[7] = 0;
            info_->P[8] = 0;
            info_->P[9] = 0;
            info_->P[10] = 1;
            info_->P[11] = 0;

            info_->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

            depth_calibration_received_ = true;
            return info_;
        }else{
            return info_;
        }
    }

    void AstraDeviceManager::shutdown(){

        reader_.remove_listener(color_stream_listener_);
        reader_.remove_listener(depth_stream_listener_);
        reader_.remove_listener(body_stream_listener_);
        color_stream_listener_.~ColorStreamListener();
        depth_stream_listener_.~DepthStreamListener();
        body_stream_listener_.~BodyStreamListener();
        ROS_INFO("Stream listeners removed");
        do_shutdown_ = true;
        t_->join();

        astra::terminate();
        ROS_INFO("astra terminated");

    }
}

