#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <cwchar>
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
            "/scan", 10, std::bind(&ReactiveFollowGap::scan_callback, this, std::placeholders::_1));

        /*// Odometry 메시지를 구독하여 odom_callback 호출*/
        /*odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(*/
        /*    "/odom", 10, std::bind(&ReactiveFollowGap::odom_callback, this, std::placeholders::_1));*/

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

    int data_size = 0;
    double left_wing = M_PI / 2;
    double right_wing = -(M_PI / 2);
    int left_wing_index = 0;
    int right_wing_index = 0;

    std::vector<float> preprocess_lidar(std::vector<float>& ranges)
    {   
      if (ranges.empty()) {
          return ranges;
      }

        // Preprocess the LiDAR scan array. Expert implementation includes:
        // Cuttoff with threwindow /swapfileshold
        double scan_threshold = 2.5;
        double min_range = *std::min_element(ranges.begin(), ranges.end());
        if (min_range < 0.1) {
          scan_threshold = 0.7;
        }
        else if (min_range < 0.2) {
            scan_threshold = 0.8; // 너무 가까운 장애물에 대해서는 1m 이하로 설정
        }
        else if (min_range < 0.3) {
            scan_threshold = 1.0;
        }
        // 거리가 2~5m일 경우
        else if (min_range < 0.6) {
            scan_threshold = 3.5; // 기본적인 2m threshold 설정
        }
        // 그 외에는 3m로 설정
        else {
            scan_threshold = 4.0;
        }

        int window_size = 9;
        int padding = window_size / 2;
        data_size = static_cast<int>(ranges.size());

        std::vector<float> padded(data_size + padding * 2);

        // left padding
        std::fill(padded.begin(), padded.begin() + padding, ranges.front());

        // copy origin
        std::copy(ranges.begin(), ranges.end(), padded.begin() + padding);

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

    int find_max_gap(std::vector<float> ranges,int min_index, int bubble_point_num)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        int half_bubble_points = bubble_point_num / 2;
        int middle_index = data_size / 2;
        double max_dist = 0.0;
        int max_index = 0;
        std::vector<int> max_points_vec;

        if ((min_index - half_bubble_points) >= 0 && 
            (min_index + half_bubble_points) <= 1079) 
        {
            // 정상 범위 처리
            for(int i = min_index - half_bubble_points; 
                i <= min_index + half_bubble_points; i++) {
                ranges[i] = 0;
            }
        }
        // 왼쪽 경계를 벗어나는 경우
        else if ((min_index - half_bubble_points) < 0) 
        {
            for(int i = 0; i < min_index + half_bubble_points; i++) {
                ranges[i] = 0;
            }
        } 
        // 오른쪽 경계를 벗어나는 경우
        else if ((min_index + half_bubble_points) > 1079) 
        {
            for(int i = min_index - half_bubble_points; i < 1080; i++) {
                ranges[i] = 0;
            }
        }
        
        // 최대값 찾기
        for(int i = right_wing_index; i <= left_wing_index; i++) {
            if(ranges[i] > max_dist) {
                max_dist = ranges[i];
            }
        }
        
        // 최대값을 가진 인덱스들 찾기
        for(int i = right_wing_index; i <= left_wing_index; i++) {
            if(ranges[i] == max_dist) {
                max_points_vec.push_back(i);
            }
        }

        // 연속된 인덱스들의 가장 긴 구간 찾기
        int current_start = max_points_vec[0];
        int current_length = 1;
        int max_gap_start = current_start;
        int max_gap_length = 1;
        
        for(int i = 1; i < max_points_vec.size(); i++) {
            // 연속된 인덱스인 경우
            if(max_points_vec[i] == max_points_vec[i-1] + 1) {
                current_length++;
            } 
            // 연속이 끊긴 경우
            else {
                // 현재까지의 구간이 최대 길이보다 크면 갱신
                if(current_length > max_gap_length) {
                    max_gap_length = current_length;
                    max_gap_start = current_start;
                }
                // 새로운 구간 시작
                current_start = max_points_vec[i];
                current_length = 1;
            }
        }
        
        // 마지막 구간 체크
        if(current_length > max_gap_length) {
            max_gap_length = current_length;
            max_gap_start = current_start;
        }
        
        // 가장 긴 연속 구간의 중간점을 최적의 주행 포인트로 선택
        int best_point_index = max_gap_start + (max_gap_length / 2);

        /*RCLCPP_INFO(this->get_logger(), "Bestpoint_index_debug: %d", best_point_index);*/
        return best_point_index;
    }

    /*int find_best_point(std::vector<float> ranges, int bubble_point_num)*/
    /*{   */
    /*    // Start_i & end_i are start and end indicies of max-gap range, respectively*/
    /*    // Return index of best point in ranges*/
    /*   // Naive: Choose the furthest point within ranges and go there*/
    /**/
    /*    return max_index;*/
    /*}*/


    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) 
    {   
        if (scan_msg->ranges.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty scan message received");
            return;
        }

        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        std::vector<float> processed_ranges(scan_msg->ranges.begin(), scan_msg->ranges.end());
        processed_ranges = preprocess_lidar(scan_msg->ranges);
        double min_range = 100.0;
        int min_index = 0;


        /// TODO:
        // Find closest point to LiDAR
        // min_range, min_index
        for(int i = 0; i < data_size; i++) {
          if(processed_ranges[i] < min_range) {
            min_range = processed_ranges[i];
            min_index = i;
          }
        }
        // Eliminate all points inside 'bubble' (set them to zero) 
        double car_width = 0.5;
        double car_radius = car_width;
        int bubble_point = (car_radius / min_range) / scan_msg->angle_increment;
        int bubble_point_num = std::min(bubble_point, 450);
        RCLCPP_INFO(this->get_logger(), "Bubble_Point_num: %d", bubble_point_num);

        // Find max length gap 
        // Find the best point in the gap 
        int best_point_index = find_max_gap(processed_ranges, min_index, bubble_point_num);
        
        // Publish Drive message
        
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        double steering_angle = scan_msg->angle_min + (scan_msg->angle_increment * best_point_index);
        double drive_speed = 0.0;
        double steering_degree = std::abs(steering_angle * 180 / M_PI);

        if (steering_degree <= 5.0) {  // 거의 직진
            drive_speed = 1.2;
        } else if (steering_degree <= 10.0) {  // 약간의 커브
            drive_speed = 1.0;
        } else if (steering_degree <= 15.0) {  // 완만한 커브
            drive_speed = 0.8;
        } else {  // 중간 커브
            drive_speed = 0.5;
        }
        /*} else if (steering_degree <= 25.0) {  // 급한 커브*/
        /*    drive_speed = 0.4;*/
        /*} else {  // 매우 급한 커브*/
        /*    drive_speed = 0.2;*/
        /*}*/
        double safe_dist = car_width * 0.5;
        
        left_wing_index = (left_wing - scan_msg->angle_min) / scan_msg->angle_increment;
        right_wing_index = (right_wing - scan_msg->angle_min) / scan_msg->angle_increment;

        /*double min_index_angle = scan_msg->angle_min + (min_index * scan_msg->angle_increment);*/
        /*double thres_ang = std::asin(car_width / min_range);*/
        /*if(steering_angle - min_index_angle > 0 && steering_angle - min_index_angle < thres_ang){ // Left*/
        /*    steering_angle = min_index_angle + thres_ang;*/
        /*}*/
        /*else if(steering_angle - min_index_angle < 0 && min_index_angle - steering_angle < thres_ang){ // Right*/
        /*    steering_angle = min_index_angle - thres_ang;*/
        /*}*/
        
        // Cornor Safety Option
        /*for(int i = 0; i < right_wing_index; i++){*/
        /*  if(processed_ranges[i] < safe_dist && steering_angle <= 0.0)*/
        /*    steering_angle = 0.0;*/
        /*}*/
        /**/
        /*for(int i = left_wing_index; i < data_size; i++){*/
        /*  if(processed_ranges[i] < safe_dist && steering_angle >= 0.0)*/
        /*    steering_angle = 0.0;*/
        /*}*/

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
