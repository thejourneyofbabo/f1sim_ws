#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        // LaserScan 메시지를 구독하여 scan_callback 호출
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));

        // Odometry 메시지를 구독하여 odom_callback 호출
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&WallFollow::odom_callback, this, std::placeholders::_1));

        // AckermannDriveStamped 메시지를 발행하는 퍼블리셔 생성
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

private:
    /*std::string lidarscan_topic = "/scan";*/
    /*std::string drive_topic = "/drive";*/
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    std::vector<float> preprocess_lidar(std::vector<float>& ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // Cuttoff with threwindow /swapfileshold
        double scan_threshold = 2.5;
        int window_size = 9;
        int padding = window_size / 2;
        int data_size = ranges.size();

        std::vector<float> padded(data_size + padding * 2);

        // left padding
        std::fill(padded.begin(), padded.begin() + padding, ranges.front());

        // copy origin
        std::copy(ranges.begin(), ranges.end(), padded.begin() + padding;

        // right padding
        std::fill(padded.begin() + padding + data_size, padded.end(), ranges.back());

        // 1.Rejecting high values (eg. > 3m)
        for(int i = 0; i < data_size + window_size; i++){
          if(padded[i] > scan_threshold)
            padded[i] = scan_threshold;
        }
  
        // 2.Setting each value to the mean over some window
        for(int i = 0; i < data_size; i++){
          float sum = 0.0;
          for(int j = 0; j < window_size; j++){
            sum += padded[i + j];
          }
          ranges[i] = sum / window_size;
        }

        return ranges;
    }

    void find_max_gap(float* ranges, int* indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        return;
    }

    void find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        return;
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        std::vector<float> processed_ranges;
        processed_ranges = preprocess_lidar(scan_msg->ranges);

        /// TODO:
        // Find closest point to LiDAR

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 

        // Find the best point in the gap 

        // Publish Drive message
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
