#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <queue>
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>

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

    double min_range = 0.0;
    int min_index = 0;
    int data_size = 0;

    std::vector<float> preprocess_lidar(std::vector<float>& ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // Cuttoff with threwindow /swapfileshold
        double scan_threshold = 2.5;
        int window_size = 9;
        int padding = window_size / 2;
        data_size = ranges.size();

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
          
          if(ranges[i] < min_range) {
            min_range = ranges[i];
            min_index = i;
          }
        }
        return ranges;
    }

    /*void find_max_gap(int index)*/
    /*{   */
    /*    // Return the start index & end index of the max gap in free_space_ranges*/
    /*    int middle_index = data_size / 2;*/
    /*    if(index > data_size)*/
    /*    return;*/
    /*}*/

    int find_best_point(std::vector<float> ranges, int bubble_point_num)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	      // Naive: Choose the furthest point within ranges and go there
        double car_width = 0.36;
        double car_radius = car_width / 2;
        int half_bubble_points = bubble_point_num / 2;
        int middle_index = data_size / 2;
        double max_dist = 0.0;
        int max_index = 0;
        if(min_index > middle_index){ // steer left
            for(int i = 0; i <= min_index - half_bubble_points; i++){
                if(ranges[i] > max_dist) {
                  max_dist = ranges[i];
                  max_index = i;
                }
            }
        } else{ // steer right
            for(int i = min_index + half_bubble_points; i < data_size; i++){
                if(ranges[i] > max_dist) {
                  max_dist = ranges[i];
                  max_index = i;
                }
            }
        }

        return max_index;
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        std::vector<float> processed_ranges;
        processed_ranges = preprocess_lidar(scan_msg->ranges);

        /// TODO:
        // Find closest point to LiDAR
        // min_range, min_index

        // Eliminate all points inside 'bubble' (set them to zero) 
        int bubble_point_num = (car_radius / min_range) / scan_msg->angle_increment;

        // Find max length gap 
        // Find the best point in the gap 
        int best_point_index = find_best_point(processed_ranges, bubble_point_num);
        
        // Publish Drive message
        
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        double steering_angle = scan_msg->ang_min + (angle_increment * best_point_index);
        double drive_speed = 0.0;
        double steering_degree = std::abs(steering_angle * 180 / M_PI);

        if (steering_degree <= 5.0) {  // 거의 직진
            drive_speed = 1.5;
        } else if (steering_degree <= 10.0) {  // 약간의 커브
            drive_speed = 1.2;
        } else if (steering_degree <= 15.0) {  // 완만한 커브
            drive_speed = 1.0;
        } else if (steering_degree <= 20.0) {  // 중간 커브
            drive_speed = 0.8;
        } else if (steering_degree <= 25.0) {  // 급한 커브
            drive_speed = 0.6;
        } else {  // 매우 급한 커브
            drive_speed = 0.4;
        }

        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;
        
        drive_pub_->publish(drive_msg);

    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
