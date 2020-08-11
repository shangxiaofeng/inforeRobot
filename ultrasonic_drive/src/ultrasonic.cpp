#include "ultrasonic_drive/ultrasonic.h"
#include "ros/ros.h"
#include <cstring>
#include <iostream>

Ultrasonic::Ultrasonic(ros::NodeHandle& nh, const std::string& topic_name, const std::string& frame_id)
{
   //base_link_name_ = base_link_name;
   frame_id_ = frame_id;
   ultrasonic_msg_.header.frame_id = frame_id_;
   ultrasonic_pub_ = nh.advertise<sensor_msgs::Range>(topic_name, 100);
}

void Ultrasonic::set_sensor_param(const float min_range, const float max_range, const float field_of_view)
{
    ultrasonic_msg_.min_range = min_range;
    ultrasonic_msg_.max_range = max_range;
    ultrasonic_msg_.field_of_view = field_of_view;
    ultrasonic_msg_.radiation_type = ultrasonic_msg_.ULTRASOUND;
}

void Ultrasonic::set_cmd_param(unsigned char cmd_data[], size_t cmd_size)
{
    memcpy(this->cmd_data_, cmd_data, cmd_size);
    this->cmd_size_ = cmd_size;
//    for (uint i = 0; i < cmd_size; i++)
//        std::cout <<" " << std::hex << static_cast<int>(cmd_data[i]);
//    std::cout << std::endl;
}

bool Ultrasonic::update_sensor_data(serial::Serial &ser)
{
    unsigned char read_buf[2];
    ser.write(cmd_data_, cmd_size_);
    size_t size_read = ser.read(read_buf, sizeof (read_buf));
    if (size_read > 0) {
        ultrasonic_msg_.header.stamp = ros::Time::now();
        ultrasonic_msg_.range = ((read_buf[0] << 8) + read_buf[1]) * 0.001f;
        if (ultrasonic_msg_.range < ultrasonic_msg_.min_range)
          ultrasonic_msg_.range = ultrasonic_msg_.min_range;
        if (ultrasonic_msg_.range > ultrasonic_msg_.max_range)
          ultrasonic_msg_.range = ultrasonic_msg_.max_range;
        ultrasonic_pub_.publish(ultrasonic_msg_);
        //ultrasonic_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), base_link_name_, frame_id_));
        std::cout << ultrasonic_msg_.header.frame_id << " dist:" << ultrasonic_msg_.range << std::endl;
    } else {
      ROS_ERROR("read ultrasonic data failed!");
      FILE* fp = fopen("ultrasonic_read_err.log", "a+");
      if (fp!= NULL){
        fprintf(fp, "ultrasonic.cpp update_sensor_data() read range sensor failed!, frame id:%s", frame_id_.c_str());
        fclose(fp);
      } else
        ROS_ERROR("write ultrasonic read error failed!");
    }
}

void Ultrasonic::set_position(float pos_x, float pos_y, float pos_z, float roll, float pitch, float yaw)
{
//    transform_.setOrigin(pose_x, pose_y, pose_z);
//    transform_.setRotation(tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw));
//    ultrasonic_broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), base_link_name_, frame_id_));
}
