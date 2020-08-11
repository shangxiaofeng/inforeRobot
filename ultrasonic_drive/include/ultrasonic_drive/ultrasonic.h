#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <serial/serial.h>


class Ultrasonic
{
public:
    Ultrasonic(ros::NodeHandle& nh, const std::string& topic_name, const std::string& frame_id);
    void set_sensor_param(const float min_range, const float max_range, const float field_of_view);
    void set_cmd_param(unsigned char cmd_data[], size_t cmd_size);
    bool update_sensor_data(serial::Serial& ser);
    void set_sleep_time(unsigned int ms);
    void set_position(float pos_x, float pos_y, float pos_z, float roll, float pitch, float yaw);
private:
    sensor_msgs::Range ultrasonic_msg_;
    ros::Publisher ultrasonic_pub_;
    size_t cmd_size_;
    unsigned char cmd_data_[128];
    std::string base_link_name_;
    std::string frame_id_;
    //unsigned int sleep_time_ms;
//    tf::TransformBroadcaster ultrasonic_broadcaster_;
//    tf::Transform transform_;
};

#endif // ULTRASONIC_H
