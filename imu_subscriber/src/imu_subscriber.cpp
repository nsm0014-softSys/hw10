#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "imu_subscriber/imu.h"

using std::placeholders::_1;

class imu_sub : public rclcpp::Node
{
public:
    IMU *imu;
    double q[4] = {1.0, 0.0, 0.0, 0.0};

    imu_sub()
        : Node("imu_sub")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "topic", 10, std::bind(&imu_sub::topic_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_repub", 10);
        double dt = 1.0 / 128.0;
        imu = new IMU(q, dt);
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double gyro[3], acc[3];

        gyro[0] = msg->angular_velocity.x;
        gyro[1] = msg->angular_velocity.y;
        gyro[2] = msg->angular_velocity.z;

        acc[0] = msg->linear_acceleration.x; // assign message values
        acc[1] = msg->linear_acceleration.y;
        acc[2] = msg->linear_acceleration.z;

        imu->update(q, gyro);

        auto imu_msg = sensor_msgs::msg::Imu();

        imu_msg.header.stamp = rclcpp::Clock().now();
        imu_msg.header.frame_id = "IMU DADDY";

        imu_msg.angular_velocity.x = gyro[0];
        imu_msg.angular_velocity.y = gyro[1];
        imu_msg.angular_velocity.z = gyro[2];
        imu_msg.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        imu_msg.linear_acceleration.x = acc[0];
        imu_msg.linear_acceleration.y = acc[1];
        imu_msg.linear_acceleration.z = acc[2];
        imu_msg.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        imu_msg.orientation.w = q[0];
        imu_msg.orientation.x = q[1];
        imu_msg.orientation.y = q[2];
        imu_msg.orientation.z = q[3];

        publisher_->publish(imu_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imu_sub>());
    rclcpp::shutdown();
    return 0;
}