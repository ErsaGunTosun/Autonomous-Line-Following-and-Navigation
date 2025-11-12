#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <random>

class WaypointSelector : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    WaypointSelector() : Node("waypoint_selector")
    {
        this->action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        waypoints_file_path_ = "/tmp/line_follow_waypoints.txt";

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&WaypointSelector::select_and_send_waypoint, this));

        RCLCPP_INFO(this->get_logger(), "Waypoint Selector Node başlatıldı. 5 saniye sonra waypoint seçilecek...");
    }

private:
    struct Waypoint {
        double x, y, z;
        double qx, qy, qz, qw;
    };

    std::vector<Waypoint> load_waypoints_from_file()
    {
        std::vector<Waypoint> waypoints;
        std::ifstream file(waypoints_file_path_);
        
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Waypoints dosyası açılamadı: %s", waypoints_file_path_.c_str());
            return waypoints;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            std::istringstream iss(line);
            Waypoint wp;
            
            if (iss >> wp.x >> wp.y >> wp.z >> wp.qx >> wp.qy >> wp.qz >> wp.qw) {
                waypoints.push_back(wp);
            }
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), "Toplam %zu waypoint yüklendi.", waypoints.size());
        return waypoints;
    }

    void select_and_send_waypoint()
    {
        timer_->cancel();

        auto waypoints = load_waypoints_from_file();
        
        if (waypoints.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Hiç waypoint bulunamadı!");
            rclcpp::shutdown();
            return;
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        
        size_t start_idx = std::max(1UL, waypoints.size() / 4);  // İlk %25'lik kısmı atla
        size_t end_idx = std::min(waypoints.size() - 1, (waypoints.size() * 3) / 4);  // Son %25'lik kısmı atla
        
        if (start_idx >= end_idx) {
            start_idx = waypoints.size() / 2;
            end_idx = start_idx;
        }
        
        std::uniform_int_distribution<> dis(start_idx, end_idx);
        size_t selected_idx = dis(gen);
        
        Waypoint selected_wp = waypoints[selected_idx];
        
        RCLCPP_INFO(this->get_logger(), 
                   "Seçilen waypoint #%zu/%zu: (%.2f, %.2f)", 
                   selected_idx + 1, waypoints.size(),
                   selected_wp.x, selected_wp.y);

        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server mevcut değil!");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";
        
        goal_msg.pose.pose.position.x = selected_wp.x;
        goal_msg.pose.pose.position.y = selected_wp.y;
        goal_msg.pose.pose.position.z = selected_wp.z;
        
        goal_msg.pose.pose.orientation.x = selected_wp.qx;
        goal_msg.pose.pose.orientation.y = selected_wp.qy;
        goal_msg.pose.pose.orientation.z = selected_wp.qz;
        goal_msg.pose.pose.orientation.w = selected_wp.qw;

        RCLCPP_INFO(this->get_logger(), 
                   "Kaydedilen rotadan hedef gönderiliyor: x=%.2f, y=%.2f", 
                   selected_wp.x, selected_wp.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&WaypointSelector::goal_response_callback, this, std::placeholders::_1);
        
        send_goal_options.feedback_callback =
            std::bind(&WaypointSelector::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        
        send_goal_options.result_callback =
            std::bind(&WaypointSelector::result_callback, this, std::placeholders::_1);

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Hedef reddedildi!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Hedef kabul edildi. Kaydedilen rotadan noktaya navigasyon başlıyor...");
        }
    }

    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), 
                   "Mevcut pozisyon: x=%.2f, y=%.2f, kalan mesafe=%.2f", 
                   feedback->current_pose.pose.position.x,
                   feedback->current_pose.pose.position.y,
                   feedback->distance_remaining);
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Kaydedilen rotadaki hedefe başarıyla ulaşıldı!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Hedef iptal edildi!");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Hedef durduruldu!");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Bilinmeyen sonuç!");
                break;
        }
        
        rclcpp::shutdown();
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string waypoints_file_path_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<WaypointSelector>();
    
    RCLCPP_INFO(node->get_logger(), "Waypoint Selector Node çalışıyor...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
