#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

using namespace std::chrono_literals;

class LineFollower : public rclcpp::Node
{
public:
  LineFollower() : Node("line_follower")
  {
    RCLCPP_INFO(this->get_logger(), "Line Follower Node baslatiliyor. 3 saniye bekleniyor...");

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    start_timer_ = this->create_wall_timer(3s, std::bind(&LineFollower::start_operations, this));

    waypoints_file_path_ = "/tmp/line_follow_waypoints.txt";

    cv::namedWindow("ROI Görüntüsü");
    cv::namedWindow("Maskelenmis Cizgi");
  }

  ~LineFollower()
  {
    cv::destroyAllWindows();
  }

private:
  const double LINEAR_SPEED = 0.15; 
  const double SEARCH_SPEED = 0.05; 
  const double Kp = 0.003;         
  const int ROI_HEIGHT_RATIO = 5;  
  const double MIN_CONTOUR_AREA = 1000.0;
  
  int log_counter_ = 0;
  const int LOG_INTERVAL = 150;     
  
  bool is_active_ = false;
  
  int missing_line_count_ = 0;
  const int MAX_MISSING_COUNT_BEFORE_STOP = 300; 
  
  bool shutdown_process_started_ = false;
  int shutdown_countdown_ = 5; 

  const cv::Scalar LOWER_RED_1 = cv::Scalar(0, 100, 100);    
  const cv::Scalar UPPER_RED_1 = cv::Scalar(10, 255, 255); 
  const cv::Scalar LOWER_RED_2 = cv::Scalar(160, 100, 100); 
  const cv::Scalar UPPER_RED_2 = cv::Scalar(180, 255, 255); 

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr countdown_timer_;
  
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  geometry_msgs::msg::PoseStamped last_saved_pose_;
  double min_distance_between_waypoints_ = 0.5; 
  bool first_pose_received_ = false;
  std::string waypoints_file_path_; 

  void start_operations()
  {
      start_timer_.reset(); 
      RCLCPP_INFO(this->get_logger(), "Line Follower: Robot hareket etmeye hazir. Islem baslatiliyor.");
      
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&LineFollower::image_callback, this, std::placeholders::_1));
      
      odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&LineFollower::odom_callback, this, std::placeholders::_1));
        
      is_active_ = true;
      
      std::ofstream file(waypoints_file_path_, std::ios::trunc);
      if (file.is_open()) {
          file << "# Line Follow Waypoints - Format: x y z qx qy qz qw\n";
          file.close();
          RCLCPP_INFO(this->get_logger(), "Waypoints dosyası hazırlandı: %s", waypoints_file_path_.c_str());
      }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
      if (!is_active_ || shutdown_process_started_) return;
      
      geometry_msgs::msg::PoseStamped current_pose;
      current_pose.header = msg->header;
      current_pose.pose = msg->pose.pose;
      
      if (!first_pose_received_) {
          save_waypoint(current_pose);
          first_pose_received_ = true;
          return;
      }
      
      double distance = calculate_distance(last_saved_pose_.pose, current_pose.pose);
      if (distance >= min_distance_between_waypoints_) {
          save_waypoint(current_pose);
      }
  }
  
  double calculate_distance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2)
  {
      double dx = pose1.position.x - pose2.position.x;
      double dy = pose1.position.y - pose2.position.y;
      double dz = pose1.position.z - pose2.position.z;
      return sqrt(dx*dx + dy*dy + dz*dz);
  }
  
  void save_waypoint(const geometry_msgs::msg::PoseStamped& pose)
  {
      waypoints_.push_back(pose);
      last_saved_pose_ = pose;
      
      std::ofstream file(waypoints_file_path_, std::ios::app);
      if (file.is_open()) {
          file << std::fixed << std::setprecision(3) 
               << pose.pose.position.x << " "
               << pose.pose.position.y << " " 
               << pose.pose.position.z << " "
               << pose.pose.orientation.x << " "
               << pose.pose.orientation.y << " "
               << pose.pose.orientation.z << " "
               << pose.pose.orientation.w << "\n";
          file.close();
      }
      
      // RCLCPP_INFO(this->get_logger(), "Waypoint kaydedildi #%zu: (%.2f, %.2f)", 
      //             waypoints_.size(), pose.pose.position.x, pose.pose.position.y);
  }

  void start_shutdown()
  {
      if (shutdown_process_started_) return;
      shutdown_process_started_ = true;

      geometry_msgs::msg::Twist stop_msg;
      stop_msg.linear.x = 0.0;
      stop_msg.angular.z = 0.0;
      publisher_->publish(stop_msg);
      
      RCLCPP_INFO(this->get_logger(), "*****************************************************");
      RCLCPP_INFO(this->get_logger(), "!!! CIZGI 10 SANiYEDiR KAYIP. PROGRAM TAMAMLANIYOR !!!");
      RCLCPP_INFO(this->get_logger(), "Toplam %zu waypoint kaydedildi.", waypoints_.size());
      RCLCPP_INFO(this->get_logger(), "Waypoints dosyası: %s", waypoints_file_path_.c_str());
      RCLCPP_INFO(this->get_logger(), "*****************************************************");
      
      countdown_timer_ = this->create_wall_timer(1s, std::bind(&LineFollower::countdown_and_exit, this));
  }
  
  void countdown_and_exit()
  {
      RCLCPP_INFO(this->get_logger(), "Program Kapatiliyor: %d...", shutdown_countdown_);
      shutdown_countdown_--;
      
      if (shutdown_countdown_ < 0)
      {
          countdown_timer_.reset();
          RCLCPP_INFO(this->get_logger(), "Kapatma Basarili. Program sonlandirildi.");
          rclcpp::shutdown();
      }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!is_active_ || shutdown_process_started_) return;
    
    log_counter_++;
    bool should_log = (log_counter_ % LOG_INTERVAL == 0);

    cv::Mat img_bgr;
    try
    {
      img_bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e) 
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge istisnasi: %s", e.what());
      return;
    }

    if (img_bgr.empty()) return; 

    int roi_start_row = img_bgr.rows * (ROI_HEIGHT_RATIO - 1) / ROI_HEIGHT_RATIO; 
    cv::Mat roi_img = img_bgr(cv::Range(roi_start_row, img_bgr.rows), cv::Range::all());
    cv::Mat img_hsv;
    cv::cvtColor(roi_img, img_hsv, cv::COLOR_BGR2HSV);
    
    cv::Mat mask_1, mask_2, mask; 
    cv::inRange(img_hsv, LOWER_RED_1, UPPER_RED_1, mask_1);
    cv::inRange(img_hsv, LOWER_RED_2, UPPER_RED_2, mask_2);
    cv::bitwise_or(mask_1, mask_2, mask);

    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    
    cv::Moments m = cv::moments(mask, true);
    double cx = 0; 
    geometry_msgs::msg::Twist twist_msg;
    
    if (m.m00 > MIN_CONTOUR_AREA)
    {
      missing_line_count_ = 0; 
      
      cx = m.m10 / m.m00;
      double image_center_x = roi_img.cols / 2.0;
      double error = image_center_x - cx;
      double angular_z = Kp * error;

      twist_msg.linear.x = LINEAR_SPEED;
      twist_msg.angular.z = angular_z;
      
      if (should_log) {
        RCLCPP_INFO(this->get_logger(), "TAKiP: Cizgi Merkezi X: %.2f, Hata: %.2f, Aci. Hiz: %.4f", cx, error, angular_z);
      }
    }
    else
    {
      missing_line_count_++;
      
      if (missing_line_count_ >= MAX_MISSING_COUNT_BEFORE_STOP)
      {
          start_shutdown();
          return;
      }

      twist_msg.linear.x = SEARCH_SPEED; 
      twist_msg.angular.z = 0.0; 
      
      if (should_log) {
        RCLCPP_WARN(this->get_logger(), "ARAMA: Cizgi Kayip. Yavasca ileri gidiliyor. (Gecen sure: %.1f saniye)", missing_line_count_ / 30.0);
      }
    }

    publisher_->publish(twist_msg);

    cv::Mat mask_bgr;
    cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);
    if (m.m00 > MIN_CONTOUR_AREA)
    {
        cv::circle(roi_img, cv::Point((int)cx, roi_img.rows / 2), 5, cv::Scalar(0, 255, 0), -1);
        cv::circle(mask_bgr, cv::Point((int)cx, mask_bgr.rows / 2), 5, cv::Scalar(0, 255, 0), -1);
    }
    cv::imshow("ROI Görüntüsü", roi_img);
    cv::imshow("Maskelenmis Cizgi", mask_bgr);
    cv::waitKey(1);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFollower>());
  return 0;
}