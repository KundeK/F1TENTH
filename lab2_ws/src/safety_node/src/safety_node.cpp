#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));

        // Subscriber to the /ego_racecar/odom topic
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, std::placeholders::_1));

        // Publisher to the /drive topic
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        RCLCPP_INFO(this->get_logger(), "Safety node initialized.");
    }

private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        speed = msg->twist.twist.linear.x;
        RCLCPP_INFO(this->get_logger(), "Current speed: %f", speed);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        double ttc_threshold = 0.5; // Threshold for braking
        bool should_brake = false;

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            double range = scan_msg->ranges[i];
            if (range < scan_msg->range_min || range > scan_msg->range_max) {
                continue; // Ignore invalid ranges
            }

            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            double relative_speed = speed * std::cos(angle);

            if (relative_speed > 0) {
                double ttc = range / relative_speed;
                if (ttc < ttc_threshold) {
                    should_brake = true;
                    break;
                }
            }
        }

        /// TODO: publish drive/brake message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = should_brake ? 0.0 : speed;
        drive_publisher_->publish(drive_msg);

        if (should_brake) {
            RCLCPP_WARN(this->get_logger(), "Emergency brake triggered!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Normal driving.");
        }

    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}