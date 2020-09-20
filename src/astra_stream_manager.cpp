//
// Created by turtlebot on 25.08.20.
//

#include "ros_astra_sdk_wrapper/astra_stream_manager.h"

namespace ros_astra_sdk_wrapper{

    DepthStreamListener::DepthStreamListener() :
            initialized_(false),
            callback_(0),
            prev_time_stamp_(0.0f)
    {
        ros::Time::init();

    }

    DepthStreamListener::~DepthStreamListener() {

    }

    void DepthStreamListener::on_frame_ready(astra::StreamReader& reader, astra::Frame& frame){

        if(frame.is_valid()) {
            const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

            sensor_msgs::ImagePtr image(new sensor_msgs::Image);

            //set time stamp
            ros::Time ros_now = ros::Time::now();
            image->header.stamp = ros_now;
            //ROS_DEBUG("Time interval between frames: %.4f ms", (float)((ros_now.toSec()-prev_time_stamp_)*1000.0));
            prev_time_stamp_ = ros_now.toSec();

            image->width = depthFrame.width();
            image->height = depthFrame.height();

            std::size_t data_size = depthFrame.byte_length();
            //ROS_DEBUG("byte length: ", (int) data_size);
            std::size_t data_length = depthFrame.length();
            //ROS_DEBUG("byte length: ", (int) data_length);

            image->data.resize(data_size);

            memcpy(&image->data[0], depthFrame.data(), data_size);

            image->is_bigendian = 0;

            //image->encoding = sensor_msgs::image_encodings::TYPE_8UC1;
            //image->step = sizeof(char) * image->width; //TODO (?)

            image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
            image->step = sizeof(char) * 2 * image->width; //TODO (?)

            callback_(image);
        }


    }


    ColorStreamListener::ColorStreamListener() :
            initialized_(false),
            callback_(0),
            prev_time_stamp_(0.0f)
    {
        ros::Time::init();
    }

    ColorStreamListener::~ColorStreamListener() {

    }

    void ColorStreamListener::on_frame_ready(astra::StreamReader& reader, astra::Frame& frame){

        if(frame.is_valid()){

            const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();

            sensor_msgs::ImagePtr image(new sensor_msgs::Image);

            //set time stamp
            ros::Time ros_now = ros::Time::now();
            image->header.stamp = ros_now;
            //ROS_DEBUG("Time interval between color frames: %.4f ms", (float)((ros_now.toSec()-prev_time_stamp_)*1000.0));
            prev_time_stamp_ = ros_now.toSec();

            image->width = colorFrame.width();
            image->height = colorFrame.height();

            std::size_t data_size = colorFrame.byte_length();
            //ROS_DEBUG("byte length: ", (int) data_size);
            std::size_t data_length = colorFrame.length();
            //ROS_DEBUG("byte length: ", (int) data_length);

            image->data.resize(data_size);

            memcpy(&image->data[0], colorFrame.data(), data_size);

            image->is_bigendian = 0;

            image->encoding = sensor_msgs::image_encodings::RGB8;
            image->step = sizeof(unsigned char) * 3 * image->width;

            callback_(image);

        }
    }

    BodyStreamListener::BodyStreamListener() :
            callback_(0),
            prev_skeleton_data_(),
            //current_robot_vel_(0.0),
            prev_valid_body_id_(-1)
    {
        ros::Time::init();

        prev_time_stamp_ = ros::Time::now();
        prev_skeleton_data_.body_id = -1;


        //boost::thread* robotVelocityListenerThr_ = new boost::thread(boost::bind(&BodyStreamListener::retrieve_robot_vel, this));
        //start thread to listen for robot velocity in x direction
    }

    BodyStreamListener::~BodyStreamListener() {

    }

    /*void BodyStreamListener::setCurrent_robot_vel (double velocity){
        current_robot_vel_ = velocity;
    }*/

    void BodyStreamListener::on_frame_ready(astra::StreamReader& reader, astra::Frame& frame){
        const astra::BodyFrame bodyFrame = frame.get<astra::BodyFrame>();

        if(bodyFrame.is_valid() && !bodyFrame.bodies().empty()){


            BodyTracking skeleton_data;
            astra::BodyList bodyList = bodyFrame.bodies();

            for (int i = 0; i < bodyList.size(); ++i) {
                skeleton_data = output_bodyframe(bodyList[i], bodyFrame.frame_index());

                if (skeleton_data.valid_body_status == true){
                    this->callback_(skeleton_data);
                }

            }

        }
    }

    BodyTracking BodyStreamListener::output_bodyframe(astra::Body body, int frame_index) {

        int current_bodyId = (int) body.id();
        ros::Time current_time_stamp = ros::Time::now();
        double time_difference;
        double current_human_vel;

        BodyTracking skeleton_data;
        astra::Joint jointLFoot;
        astra::Joint jointRFoot;
        astra::Joint jointLKnee;
        astra::Joint jointRKnee;
        astra::Joint jointLHip;
        astra::Joint jointRHip;
        astra::Joint jointBaseSpine;
        astra::Joint jointMidSpine;

        skeleton_data.header.stamp = current_time_stamp;
        skeleton_data.header.frame_id = std::to_string(frame_index);
        skeleton_data.body_id = current_bodyId;

        //right foot
        jointRFoot = body.joints()[15];
        skeleton_data.rfoot_tracking_status = (int) jointRFoot.status();
        if(jointRFoot.status() == astra::JointStatus::NotTracked){
            skeleton_data.joint_position_rfoot.x = 0;
            skeleton_data.joint_position_rfoot.y = 0;
            skeleton_data.joint_position_rfoot.z = 0;
        }else{
            skeleton_data.joint_position_rfoot.x = jointRFoot.world_position().z / 1000.0;
            skeleton_data.joint_position_rfoot.y = jointRFoot.world_position().x / 1000.0;
            skeleton_data.joint_position_rfoot.z = jointRFoot.world_position().y / 1000.0;
        }

        //left foot
        jointLFoot = body.joints()[12];
        skeleton_data.lfoot_tracking_status = (int) jointLFoot.status();
        if(jointLFoot.status() == astra::JointStatus::NotTracked){
            skeleton_data.joint_position_lfoot.x = 0;
            skeleton_data.joint_position_lfoot.y = 0;
            skeleton_data.joint_position_lfoot.z = 0;
        }else{
            skeleton_data.joint_position_lfoot.x = jointLFoot.world_position().z / 1000.0;
            skeleton_data.joint_position_lfoot.y = jointLFoot.world_position().x / 1000.0;
            skeleton_data.joint_position_lfoot.z = jointLFoot.world_position().y / 1000.0;
        }


        //right knee
        jointRKnee = body.joints()[14];
        skeleton_data.rknee_tracking_status = (int) jointRKnee.status();
        if(jointRKnee.status() == astra::JointStatus::NotTracked){
            skeleton_data.joint_position_rknee.x = 0;
            skeleton_data.joint_position_rknee.y = 0;
            skeleton_data.joint_position_rknee.z = 0;
        }else{
            skeleton_data.joint_position_rknee.x = jointRKnee.world_position().z / 1000.0;
            skeleton_data.joint_position_rknee.y = jointRKnee.world_position().x / 1000.0;
            skeleton_data.joint_position_rknee.z = jointRKnee.world_position().y / 1000.0;
        }

        //left knee
        jointLKnee = body.joints()[11];
        skeleton_data.lknee_tracking_status = (int) jointLKnee.status();
        if(jointLKnee.status() == astra::JointStatus::NotTracked){
            skeleton_data.joint_position_lknee.x = 0;
            skeleton_data.joint_position_lknee.y = 0;
            skeleton_data.joint_position_lknee.z = 0;
        }else{
            skeleton_data.joint_position_lknee.x = jointLKnee.world_position().z / 1000.0;
            skeleton_data.joint_position_lknee.y = jointLKnee.world_position().x / 1000.0;
            skeleton_data.joint_position_lknee.z = jointLKnee.world_position().y / 1000.0;
        }

        //right hip
        jointRHip = body.joints()[13];
        skeleton_data.rhip_tracking_status = (int) jointRHip.status();
        if(jointRHip.status() == astra::JointStatus::NotTracked){
            skeleton_data.joint_position_rhip.x = 0;
            skeleton_data.joint_position_rhip.y = 0;
            skeleton_data.joint_position_rhip.z = 0;
        }else{
            skeleton_data.joint_position_rhip.x = jointRHip.world_position().z / 1000.0;
            skeleton_data.joint_position_rhip.y = jointRHip.world_position().x / 1000.0;
            skeleton_data.joint_position_rhip.z = jointRHip.world_position().y / 1000.0;
        }

        //left hip
        jointLHip = body.joints()[10];
        skeleton_data.lhip_tracking_status = (int) jointLHip.status();
        if(jointLHip.status() == astra::JointStatus::NotTracked){
            skeleton_data.joint_position_lhip.x = 0;
            skeleton_data.joint_position_lhip.y = 0;
            skeleton_data.joint_position_lhip.z = 0;
        }else{
            skeleton_data.joint_position_lhip.x = jointLHip.world_position().z / 1000.0;
            skeleton_data.joint_position_lhip.y = jointLHip.world_position().x / 1000.0;
            skeleton_data.joint_position_lhip.z = jointLHip.world_position().y / 1000.0;
        }

        //base spine
        jointBaseSpine = body.joints()[9];
        skeleton_data.base_spine_tracking_status = (int) jointBaseSpine.status();
        if(jointBaseSpine.status() == astra::JointStatus::NotTracked){
            skeleton_data.joint_position_base_spine.x = 0;
            skeleton_data.joint_position_base_spine.y = 0;
            skeleton_data.joint_position_base_spine.z = 0;
        }else{
            skeleton_data.joint_position_base_spine.x = jointBaseSpine.world_position().z / 1000.0;
            skeleton_data.joint_position_base_spine.y = jointBaseSpine.world_position().x / 1000.0;
            skeleton_data.joint_position_base_spine.z = jointBaseSpine.world_position().y / 1000.0;
        }

        //mid spine
        jointMidSpine = body.joints()[8];
        skeleton_data.mid_spine_tracking_status = (int) jointMidSpine.status();
        if(jointMidSpine.status() == astra::JointStatus::NotTracked){
            skeleton_data.joint_position_mid_spine.x = 0;
            skeleton_data.joint_position_mid_spine.y = 0;
            skeleton_data.joint_position_mid_spine.z = 0;
        }else{
            skeleton_data.joint_position_mid_spine.x = jointMidSpine.world_position().z / 1000.0;
            skeleton_data.joint_position_mid_spine.y = jointMidSpine.world_position().x / 1000.0;
            skeleton_data.joint_position_mid_spine.z = jointMidSpine.world_position().y / 1000.0;
        }

        if(skeleton_data.joint_position_base_spine.x == prev_skeleton_data_.joint_position_base_spine.x){
            skeleton_data.valid_body_status = false;
        }else{
            skeleton_data.valid_body_status = true;
        }

        /*if(prev_skeleton_data_.body_id != -1 && skeleton_data.base_spine_tracking_status == 2 && current_robot_vel_ != 0) {
            time_difference = current_time_stamp.toSec() - prev_time_stamp_.toSec();
            current_human_vel = sqrt(pow(
                    (skeleton_data.joint_position_base_spine.x - prev_skeleton_data_.joint_position_base_spine.x),
                    2.0)) / time_difference;

            std::cout << "current human vel: " << current_human_vel << std::endl;

            //ROS_INFO("current human vel was calculated");
        }else{
            prev_valid_body_id_ = -1;
            current_human_vel = -1.0;
        }

        if((current_human_vel < (current_robot_vel_*1.3))){
            prev_valid_body_id_ = -1;
            skeleton_data.valid_body_status = true;
        }else{
            skeleton_data.valid_body_status = true;
            prev_valid_body_id_ = current_bodyId;
        }*/

        prev_skeleton_data_ = skeleton_data;
        prev_time_stamp_ = current_time_stamp;

        return skeleton_data;
    }

};