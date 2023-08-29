#include "sensor_filter_kit/sensor_filter_kit_lib.h"
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "iostream"

#define SENSOR_NUMBER 6

class SensorFilterNode
{
    private:
        struct imu_data
        {
            float accel_x,accel_y,accel_z;
            float gyro_x,gyro_y,gyro_z; // Yaw angular acceleration [degrees/s^2]
            unsigned long long timestamp; // unix timestamp [ms]

        };

        imu_data imu_data;

        ros::Subscriber imu_sub;
        ros::Publisher imu_pub;

        std::string IMU_RAW_TOPIC,IMU_FILTERED_TOPIC;
        int WINDOW_SIZE=100;

        uint sensors[SENSOR_NUMBER] = {ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z};
        float sensor_readings[SENSOR_NUMBER];

        std::vector<double> features;

        sensor_msgs::Imu imu_filtered;

        FilterKit *filter_kit;

    public:

        SensorFilterNode(ros::NodeHandle *nh)
        {
            nh->getParam("/imu_raw_topic",IMU_RAW_TOPIC);
            nh->getParam("/imu_filtered_topic",IMU_FILTERED_TOPIC);
            nh->getParam("/window_size",WINDOW_SIZE);

            filter_kit = new FilterKit(SENSOR_NUMBER,WINDOW_SIZE);

            ROS_INFO("Successfully constructed FilterKit class ...");

            imu_sub = nh->subscribe(IMU_RAW_TOPIC,1000,&SensorFilterNode::imu_callback,this);

            imu_pub = nh->advertise<sensor_msgs::Imu>(IMU_FILTERED_TOPIC,1000);

        }

        void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
        {
            imu_data.accel_x = imu_msg->linear_acceleration.x;
            imu_data.accel_y = imu_msg->linear_acceleration.y;
            imu_data.accel_z = imu_msg->linear_acceleration.z;

            imu_data.gyro_x = imu_msg->angular_velocity.x;
            imu_data.gyro_y = imu_msg->angular_velocity.y;
            imu_data.gyro_z = imu_msg->angular_velocity.z;

            ROS_INFO_STREAM_ONCE("Started advertising on topic : "<<IMU_FILTERED_TOPIC);

            sensor_readings[0] = imu_data.accel_x;
            sensor_readings[1] = imu_data.accel_y;
            sensor_readings[2] = imu_data.accel_z;
            sensor_readings[3] = imu_data.gyro_x;
            sensor_readings[4] = imu_data.gyro_y;
            sensor_readings[5] = imu_data.gyro_z;

            filter_kit->window(sensor_readings,sensors,SMA);

            features = filter_kit->get_features();

            imu_filtered.linear_acceleration.x = features.at(0);
            imu_filtered.linear_acceleration.y = features.at(1);
            imu_filtered.linear_acceleration.z = features.at(2);
            imu_filtered.angular_velocity.x = features.at(3);
            imu_filtered.angular_velocity.x = features.at(4);
            imu_filtered.angular_velocity.x = features.at(5);

            imu_filtered.header.stamp = ros::Time::now();
            imu_filtered.header.frame_id = "imu_filtered";

            imu_pub.publish(imu_filtered);

        }



};

int main(int argc,char **argv)
{
    ros::init(argc,argv,"SensorFilterNode");
    ros::NodeHandle n;
    SensorFilterNode Sfn = SensorFilterNode(&n);

    ros::spin();
    return 0;
}