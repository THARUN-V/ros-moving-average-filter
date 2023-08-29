#include "sensor_filter_kit/sensor_filter_kit_lib.h"
#include "sensor_msgs/Imu.h"
#include "sensor_filter_kit/SensorKitData.h"
#include "ros/ros.h"
#include "iostream"

uulong_t get_millis_since_epoch()
{
  uulong_t millis_since_epoch =
       std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::system_clock::now().time_since_epoch()).count();
  
  return millis_since_epoch;
}

struct imu_data
{
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;      // Yaw angular acceleration [degrees/s^2]
  unsigned long long timestamp;      // Unix timestamp [ms]
};

imu_data imu_data;

void imu_data_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  imu_data.accel_x = imu_msg->linear_acceleration.x;
  imu_data.accel_y = imu_msg->linear_acceleration.y;
  imu_data.accel_z = imu_msg->linear_acceleration.z;

  imu_data.gyro_x = imu_msg->angular_velocity.x;
  imu_data.gyro_y = imu_msg->angular_velocity.y;
  imu_data.gyro_z = imu_msg->angular_velocity.z;
}

int main(int argc, char **argv)
{
  // int publishing_freq= 0;
  // if(argc < 2)
  // {
  //   ROS_ERROR("Missing param! Try: <publishing_freq>.");
  //   return 0;
  // }
  // if(atoi(argv[1]) != 0)
  // {
  //   publishing_freq = atoi(argv[1]);
  //   ROS_INFO_STREAM("Publishing freq. set to: " << publishing_freq);
  // }
  // else
  // {
  //   ROS_ERROR("You must pass a recording freq. above 0, quitting.");
  //   return 0;
  // }

  const uint window_size = 5;
  const uint SENSOR_NUMBER = 6;
  uint sensors[SENSOR_NUMBER] = {ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y, GYRO_Z};
  float sensor_readings[SENSOR_NUMBER];

  FilterKit filter_kit(SENSOR_NUMBER, window_size);

  ROS_INFO("Successfully constructed FilterKit class..");

  ros::init(argc, argv, "sensor_filter_kit_node");
  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("/imu0", 1000, imu_data_callback);
  ros::Publisher publisher = n.advertise<sensor_msgs::Imu>("/Imu/Filtered", 1000);
  // ros::Rate loop_rate(publishing_freq);
  
  // sensor_filter_kit::SensorKitData sensor_kit_data;
  sensor_msgs::Imu imu_filtered;

  std::vector<double> features;
  std::cout << std::fixed;
  std::cout << std::setprecision(4);
  
  while(ros::ok())
  {
    ROS_INFO_STREAM_ONCE("Started advertising on topic sensor_kit_data..");
    
    sensor_readings[0] = imu_data.accel_x;
    sensor_readings[1] = imu_data.accel_y;
    sensor_readings[2] = imu_data.accel_z;
    sensor_readings[3] = imu_data.gyro_x;
    sensor_readings[4] = imu_data.gyro_y;
    sensor_readings[5] = imu_data.gyro_z;

    filter_kit.window(sensor_readings, sensors, SMA);

    features = filter_kit.get_features();
    
    imu_filtered.linear_acceleration.x = features.at(0);
    imu_filtered.linear_acceleration.y = features.at(1);
    imu_filtered.linear_acceleration.z = features.at(2);
    imu_filtered.angular_velocity.x = features.at(3);
    imu_filtered.angular_velocity.y = features.at(4);
    imu_filtered.angular_velocity.z = features.at(5);
  
    // sensor_kit_data.timestamp = get_millis_since_epoch();
    imu_filtered.header.stamp = ros::Time::now();
    imu_filtered.header.frame_id = "imu_filtered";
    
    publisher.publish(imu_filtered);
    ros::spinOnce();
    // loop_rate.sleep();
  }
}