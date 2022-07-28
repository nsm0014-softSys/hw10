
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    std::string frame_id = "ok";
    std::string input_file = "/home/noahfireball1/hw10_ws/src/imu_publisher/src/imu_data.txt"; // declare file name
    std::ifstream infile;
    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
    {
        infile.open(input_file);
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("topic", 10);
        // File Reading Setup

        timer_ = this->create_wall_timer(
            7.8ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto imu_msg = sensor_msgs::msg::Imu();

        double acc[3], gyro[3], mag[3];

        imu_msg.header.stamp = rclcpp::Clock().now();
        imu_msg.header.frame_id = frame_id;

        infile >> acc[0] >> acc[1] >> acc[2] >> gyro[0] >> gyro[1] >> gyro[2] >> mag[0] >> mag[1] >> mag[2]; // passes input file values to corresponding variables

        imu_msg.linear_acceleration.x = acc[0]; // assign message values
        imu_msg.linear_acceleration.y = acc[1];
        imu_msg.linear_acceleration.z = acc[2];
        imu_msg.linear_acceleration_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        imu_msg.angular_velocity.x = gyro[0];
        imu_msg.angular_velocity.y = gyro[1];
        imu_msg.angular_velocity.z = gyro[2];
        imu_msg.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        publisher_->publish(imu_msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
