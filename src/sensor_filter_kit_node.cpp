#include "sensor_filter_kit/sensor_filter_kit_lib.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define SENSOR_NUMBER 6

class SensorFilterNode : public rclcpp::Node
{
    private:
        struct imu_data
        {
            float accel_x,accel_y,accel_z;
            float gyro_x,gyro_y,gyro_z; // Yaw angular acceleration [degrees/s^2]
            unsigned long long timestamp; // unix timestamp [ms]

        };

        imu_data imu_data;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ImuPub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ImuSub;

        std::string IMU_RAW_TOPIC,IMU_FILTERED_TOPIC;
        int WINDOW_SIZE;

        uint sensors[SENSOR_NUMBER] = {ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z};
        float sensor_readings[SENSOR_NUMBER];

        std::vector<double> features;

        sensor_msgs::msg::Imu imu_filtered;

        FilterKit *filter_kit;

    public:

        SensorFilterNode(std::string NodeName) : Node(NodeName)
        {
            this->declare_parameter("/imu_raw_topic","/imu0");
            this->declare_parameter("/imu_filtered_topic","/imu/filtered");
            this->declare_parameter("/window_size",10);

            this->get_parameter("/imu_raw_topic",IMU_RAW_TOPIC);
            this->get_parameter("/imu_filtered_topic",IMU_FILTERED_TOPIC);
            this->get_parameter("/window_size",WINDOW_SIZE);

            filter_kit = new FilterKit(SENSOR_NUMBER,WINDOW_SIZE);
            
            RCLCPP_INFO(this->get_logger(),"Successfully constructed FilterKit class ...");

            ImuSub = this->create_subscription<sensor_msgs::msg::Imu>(IMU_RAW_TOPIC,1000,std::bind(&SensorFilterNode::imu_callback,this,std::placeholders::_1));
            ImuPub = this->create_publisher<sensor_msgs::msg::Imu>(IMU_FILTERED_TOPIC,1000);
        }

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
        {
            imu_data.accel_x = imu_msg->linear_acceleration.x;
            imu_data.accel_y = imu_msg->linear_acceleration.y;
            imu_data.accel_z = imu_msg->linear_acceleration.z;

            imu_data.gyro_x = imu_msg->angular_velocity.x;
            imu_data.gyro_y = imu_msg->angular_velocity.y;
            imu_data.gyro_z = imu_msg->angular_velocity.z;

            RCLCPP_INFO_ONCE(this->get_logger(),"Started advertising on topic : %s",IMU_FILTERED_TOPIC.c_str());

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
            imu_filtered.angular_velocity.y = features.at(4);
            imu_filtered.angular_velocity.z = features.at(5);

            imu_filtered.header.stamp = this->get_clock()->now();
            imu_filtered.header.frame_id = "imu_filtered";

            ImuPub->publish(imu_filtered);
        }
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SensorFilterNode>("SensorFilterNode"));

    rclcpp::shutdown();
    return 0;
}