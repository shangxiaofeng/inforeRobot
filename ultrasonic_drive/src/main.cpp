#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "ultrasonic_drive/ultrasonic.h"


//void init_ultrasonic_port(serial::Serial& ser) {
//    //init serial port
//    ser.setPort("/dev/ttyUSB0");
//    ser.setBaudrate(115200);
//    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
//    ser.setTimeout(to);
//    ser.open();
//}


//int main(int argc, char* argv[]) {
//  ros::init(argc, argv, "serial_example_node");
//  ros::NodeHandle nh;
//  ros::Publisher ultrasonic_pub_1 = nh.advertise<sensor_msgs::Range>("/ultrasonic_1", 1);

//  serial::Serial ser;
//  init_ultrasonic_port(ser);

//  //open serial port
//  if (ser.isOpen()) {
//    std::cout << "Serial Port initialized" << std::endl;
//  } else {
//    std::cout << "Serial Port open failed!" << std::endl;
//    return -1;
//  }

//  sensor_msgs::Range ultrasonic_1;
//  ultrasonic_1.min_range = 0.03f;
//  ultrasonic_1.max_range = 3.5f;
//  ultrasonic_1.field_of_view = 1;
//  ultrasonic_1.radiation_type = ultrasonic_1.ULTRASOUND;
//  ultrasonic_1.header.frame_id = "ultrasonic_1_link";


//  unsigned char read_buf[2];
//  unsigned char request[3] = {0xda, 0x02, 0xbc};
//  ros::Rate loop_rate(10);
//  ros::Duration duration_sleep(0.001);
//  while (ros::ok()) {
//    ser.write(request, sizeof (request));
//    duration_sleep.sleep();
//    size_t size_read = ser.read(read_buf, sizeof (read_buf));
//    if (size_read > 0) {
//      ultrasonic_1.range = ((read_buf[0] << 8) + read_buf[1]) * 0.001f;
//    } else {
//        std::cout <<  "read failed!" << std::endl;
//    }
//    std::cout << "dist:" << ultrasonic_1.range << std::endl;

//    ros::spinOnce();
//    loop_rate.sleep();
//  }
//}


void init_ultrasonic_port(serial::Serial& ser, const std::string& dev_name, unsigned int baud_rate = 115200, unsigned int timeout_ms = 100) {
    //init serial port
    ser.setPort(dev_name);
    ser.setBaudrate(baud_rate);
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout_ms);
    ser.setTimeout(to);
}


int main(int argc, char* argv[]) {

  ros::init(argc, argv, "serial_example_node");
  ros::NodeHandle nh;

  std::string dev_name = "/dev/ttyUSB4";
  nh.getParam("port", dev_name);

  float min_dist = 0.03;
  nh.getParam("min_dist", min_dist);

  float max_dist = 3.0;
  nh.getParam("max_dist", max_dist);

  float max_angle = 50.0;
  nh.getParam("max_angle", max_angle);

  serial::Serial ser;
  init_ultrasonic_port(ser, dev_name);

  std::cout << "dev_name:" << dev_name << " min_dist:" << min_dist <<" max_dist:" << max_dist << " max_angle:" << max_angle << std::endl;

  //open serial port
  ser.open();
  if (ser.isOpen()) {
    std::cout << "Serial Port initialized" << std::endl;
  } else {
    std::cout << "Serial Port open failed!" << std::endl;
    return -1;
  }

  unsigned char cmd_data[][3] = {
      {0xda, 0x02, 0xbc},
      {0xd6, 0x02, 0xbc},
      {0xd6, 0x02, 0xbc},
      {0xdc, 0x02, 0xbc},
      {0xda, 0x02, 0xbc},
  };

//  const float min_dist = 0.03f;
//  const float max_dist = 3.0f;
  const float field_of_view = max_angle / 180.0f * 3.1416f;


//  std::string topic_name, parent_link, frame_id;
//  float pose_x, pose_y, pose_z, pose_roll, pose_pitch, pose_yaw;

//  topic_name = "ultrasonic_1";
//  frame_id = topic_name + "_link";
//  parent_link = ;

  Ultrasonic ultrasonic_1(nh, "ultrasonic_1", "ultrasonic_link_1");
  ultrasonic_1.set_sensor_param(min_dist, max_dist, field_of_view);
  ultrasonic_1.set_cmd_param(cmd_data[0], sizeof (cmd_data[0]));

  Ultrasonic ultrasonic_2(nh, "ultrasonic_2", "ultrasonic_link_2");
  ultrasonic_2.set_sensor_param(min_dist, max_dist, field_of_view);
  ultrasonic_2.set_cmd_param(cmd_data[1], sizeof (cmd_data[1]));

  Ultrasonic ultrasonic_3(nh, "ultrasonic_3", "ultrasonic_link_3");
  ultrasonic_3.set_sensor_param(min_dist, max_dist, field_of_view);
  ultrasonic_3.set_cmd_param(cmd_data[2], sizeof (cmd_data[2]));

  Ultrasonic ultrasonic_4(nh, "ultrasonic_4", "ultrasonic_link_4");
  ultrasonic_4.set_sensor_param(min_dist, max_dist, field_of_view);
  ultrasonic_4.set_cmd_param(cmd_data[3], sizeof (cmd_data[3]));

  Ultrasonic ultrasonic_5(nh, "ultrasonic_5", "ultrasonic_link_5");
  ultrasonic_5.set_sensor_param(min_dist, max_dist, field_of_view);
  ultrasonic_5.set_cmd_param(cmd_data[4], sizeof (cmd_data[4]));

  ros::Rate loop_rate(20);
  while(ros::ok()) {
      ultrasonic_1.update_sensor_data(ser);
//      ultrasonic_2.update_sensor_data(ser);
      ultrasonic_3.update_sensor_data(ser);
//      ultrasonic_4.update_sensor_data(ser);
//      ultrasonic_5.update_sensor_data(ser);
      loop_rate.sleep();
  }
}


