//
// Created by turtlebot on 25.08.20.
//

//boost
#include <boost/bind.hpp>

//ros
#include <sensor_msgs/distortion_models.h>

#include "ros_astra_sdk_wrapper/stream_wrapper.h"

namespace ros_astra_sdk_wrapper{

    StreamWrapper::StreamWrapper(ros::NodeHandle& n, ros::NodeHandle& pnh) :
                nh_(n),
                pnh_(pnh),
                //sub_vel_(),
                device_manager_(AstraDeviceManager::getSingelton()),
                color_time_offset_(-1.0),// TODO: check
                depth_time_offset_(-1.0),// TODO: check
                depth_raw_subscribers_(false)
    {
        ROS_INFO("Started astra sdk nodelet");
        readConfigFromParameterServer();
        advertiseROSTopics();
    }

    StreamWrapper::~StreamWrapper() {
        device_manager_->shutdown();
    }

    void StreamWrapper::advertiseROSTopics() {
        // Allow remapping namespaces rgb, ir, depth, depth_registered, body
        ros::NodeHandle color_nh(nh_, "rgb");
        image_transport::ImageTransport color_it(color_nh);
        ros::NodeHandle depth_nh(nh_, "depth");
        image_transport::ImageTransport depth_it(depth_nh);
        ros::NodeHandle depth_raw_nh(nh_, "depth");
        image_transport::ImageTransport depth_raw_it(depth_raw_nh);
        ros::NodeHandle body_nh(nh_, "body");

        // Advertise all published topics

        // Prevent connection callbacks from executing until we've set all the publishers. Otherwise
        // connectCb() can fire while we're advertising (say) "depth/image_raw", but before we actually
        // assign to pub_depth_raw_. Then pub_depth_raw_.getNumSubscribers() returns 0, and we fail to start
        // the depth generator.
        boost::lock_guard<boost::mutex> lock(connect_mutex_);

        image_transport::SubscriberStatusCallback c_itssc = boost::bind(&StreamWrapper::colorConnectCallback, this);
        ros::SubscriberStatusCallback c_rssc = boost::bind(&StreamWrapper::colorConnectCallback, this);
        pub_color_ = color_it.advertiseCamera("image", 1, c_itssc, c_itssc, c_rssc, c_rssc);

        image_transport::SubscriberStatusCallback d_itssc = boost::bind(&StreamWrapper::depthConnectCallback, this);
        ros::SubscriberStatusCallback d_rssc = boost::bind(&StreamWrapper::depthConnectCallback, this);
        pub_depth_raw_ = depth_it.advertiseCamera("image_raw", 1, d_itssc, d_itssc, d_rssc, d_rssc);
        pub_depth_ = depth_raw_it.advertiseCamera("image", 1, d_itssc, d_itssc, d_rssc, d_rssc);

        ros::SubscriberStatusCallback b_rssc = boost::bind(&StreamWrapper::bodyConnectCallback, this);
        pub_body_ = body_nh.advertise<BodyTracking>("skeleton", 10, b_rssc);

    }

    void StreamWrapper::colorConnectCallback(){

        boost::lock_guard<boost::mutex> lock(connect_mutex_);

        if (device_manager_->stream_initialized_ && !device_manager_->color_stream_initialized_)
        {

            device_manager_->setColorFrameCallback(boost::bind(&StreamWrapper::newColorFrameCallback, this, _1));

            ROS_INFO("Starting color stream.");
            device_manager_->startColorStream();

        }

    }

    void StreamWrapper::depthConnectCallback(){

        boost::lock_guard<boost::mutex> lock(connect_mutex_);

        depth_raw_subscribers_ = pub_depth_raw_.getNumSubscribers() > 0;

        if (device_manager_->stream_initialized_ && !device_manager_->depth_stream_initialized_)
        {

            device_manager_->setDepthFrameCallback(boost::bind(&StreamWrapper::newDepthFrameCallback, this, _1));

            ROS_INFO("Starting depth stream.");
            device_manager_->startDepthStream();

        }
    }

    void StreamWrapper::bodyConnectCallback(){

        boost::lock_guard<boost::mutex> lock(connect_mutex_);

        if (device_manager_->stream_initialized_ && !device_manager_->body_stream_initialized_)
        {
            device_manager_->setBodyFrameCallback(boost::bind(&StreamWrapper::newBodyFrameCallback, this, _1));

            ROS_INFO("Starting body stream.");
            device_manager_->startBodyStream();

            //retrieveRobotVelocity();

        }
    }

    /*void StreamWrapper::retrieveRobotVelocity() {
        ROS_INFO("jumped into received robot vel");
        ros_astra_sdk_wrapper::BodyTracking msg;
        ros::NodeHandle vel_sub_nh(nh_, "vel");
        sub_vel_ = vel_sub_nh.subscribe("/navigation_velocity_smoother/raw_cmd_vel", 10, &StreamWrapper::newVelocityCallback, this);
        ROS_INFO("subscription ready");
    }*/

    /*void StreamWrapper::newVelocityCallback(const geometry_msgs::Twist& twist){
        device_manager_->body_stream_listener_.setCurrent_robot_vel(twist.linear.x);
        //ROS_INFO("new velocity received");
    }*/

    void StreamWrapper::newColorFrameCallback(sensor_msgs::ImagePtr image){

        image->header.frame_id = color_frame_id_;
        image->header.stamp = image->header.stamp;
        pub_color_.publish(image, getColorCameraInfo(image->width, image->height, image->header.stamp));

    }

    //get color camera info because depth_registered is set to true in launch file, rgb and depth image are aligned by Astra SDK
    void StreamWrapper::newDepthFrameCallback(sensor_msgs::ImagePtr image){

        image->header.frame_id = depth_frame_id_;
        image->header.stamp = image->header.stamp;

        if(depth_raw_subscribers_){
            pub_depth_raw_.publish(image, getColorCameraInfo(image->width, image->height, image->header.stamp));
        }

        sensor_msgs::ImageConstPtr floating_point_image = rawToFloatingPointConversion(image);
        pub_depth_.publish(image, getColorCameraInfo(image->width, image->height, image->header.stamp));
    }

    void StreamWrapper::newBodyFrameCallback(BodyTracking skeleton){
        pub_body_.publish(skeleton);
    }

    //TODO device_id nochmal schauen
    void StreamWrapper::readConfigFromParameterServer()
    {
        if (!pnh_.getParam("device_id", device_id_))
        {
            device_id_ = "#1";
        }

        // Camera TF frames
        //pnh_.param("ir_frame_id", ir_frame_id_, std::string("/openni_ir_optical_frame"));
        pnh_.param("rgb_frame_id", color_frame_id_, std::string("/camera_rgb_optical_frame"));
        pnh_.param("depth_frame_id", depth_frame_id_, std::string("/camera_depth_optical_frame"));

        ROS_DEBUG("ir_frame_id = '%s' ", ir_frame_id_.c_str());
        ROS_DEBUG("rgb_frame_id = '%s' ", color_frame_id_.c_str());
        ROS_DEBUG("depth_frame_id = '%s' ", depth_frame_id_.c_str());

        pnh_.param("rgb_camera_info_url", color_info_url_, std::string());
        pnh_.param("depth_camera_info_url", ir_info_url_, std::string());

    }


    // TODO
    sensor_msgs::CameraInfoPtr StreamWrapper::getColorCameraInfo(int width, int height, ros::Time time) const
    {
        sensor_msgs::CameraInfoPtr info;

        info = device_manager_->getColorCamInfo();

        // Fill in header
        info->header.stamp    = time;
        info->header.frame_id = color_frame_id_;

        return info;
    }
/*
    sensor_msgs::CameraInfoPtr StreamWrapper::getDepthCameraInfo(int width, int height, ros::Time time) const
    {
        sensor_msgs::CameraInfoPtr info;

        info = device_manager_->getDepthCamInfo();

        // Fill in header
        info->header.stamp    = time;
        info->header.frame_id = depth_frame_id_;

        return info;
    }
*/

    sensor_msgs::ImageConstPtr StreamWrapper::rawToFloatingPointConversion(sensor_msgs::ImageConstPtr raw_image)
    {
        static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

        sensor_msgs::ImagePtr new_image = boost::make_shared<sensor_msgs::Image>();

        new_image->header = raw_image->header;
        new_image->width = raw_image->width;
        new_image->height = raw_image->height;
        new_image->is_bigendian = 0;
        new_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1; //TODO
        new_image->step = sizeof(float)*raw_image->width;

        std::size_t data_size = new_image->width*new_image->height;
        new_image->data.resize(data_size*sizeof(float));

        const unsigned short* in_ptr = reinterpret_cast<const unsigned short*>(&raw_image->data[0]);
        float* out_ptr = reinterpret_cast<float*>(&new_image->data[0]);

        for (std::size_t i = 0; i<data_size; ++i, ++in_ptr, ++out_ptr)
        {
            if (*in_ptr==0 || *in_ptr==0x7FF)
            {
                *out_ptr = bad_point;
            } else
            {
                *out_ptr = static_cast<float>(*in_ptr)/1000.0f;
            }
        }

        return new_image;
    }
}
