#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

class GreenPointNavigator : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    struct GreenPoint {
        std::string name;
        double x, y, z;
        double yaw;  
    };

    GreenPointNavigator() : Node("green_point_navigator")
    {
        this->action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        green_points_ = {
            {"GreenPoint_1", 0.61401, -1.95798, 0.0, 2.61831},
            {"GreenPoint_2", -1.94599, -3.60798, 0.0, 0.0},       
            {"GreenPoint_3", -5.575, -3.050, 0.0, 0.782787}  
        };

        current_point_index_ = 0;

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&GreenPointNavigator::navigate_to_next_point, this));

        RCLCPP_INFO(this->get_logger(), "Green Point Navigator başlatıldı. %zu green point'e sırayla gidilecek.", green_points_.size());
        RCLCPP_INFO(this->get_logger(), "5 saniye sonra %s'e gidilecek...", green_points_[0].name.c_str());
    }

private:
    void navigate_to_next_point()
    {
        timer_->cancel();

        if (current_point_index_ >= green_points_.size()) {
            RCLCPP_INFO(this->get_logger(), "Tüm green point'lere başarıyla gidildi! Görev tamamlandı.");
            rclcpp::shutdown();
            return;
        }

        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server mevcut değil!");
            return;
        }

        const auto& target_point = green_points_[current_point_index_];

        auto goal_msg = NavigateToPose::Goal();
        
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";
        
        goal_msg.pose.pose.position.x = target_point.x;
        goal_msg.pose.pose.position.y = target_point.y;
        goal_msg.pose.pose.position.z = target_point.z;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, target_point.yaw);
        goal_msg.pose.pose.orientation = tf2::toMsg(q);


        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&GreenPointNavigator::goal_response_callback, this, std::placeholders::_1);
        
        send_goal_options.feedback_callback =
            std::bind(&GreenPointNavigator::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        
        send_goal_options.result_callback =
            std::bind(&GreenPointNavigator::result_callback, this, std::placeholders::_1);

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Hedef reddedildi!");
        } else {
            const auto& target_point = green_points_[current_point_index_];
            RCLCPP_INFO(this->get_logger(), "%s hedefe navigasyon başladı...", target_point.name.c_str());
        }
    }

    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        const auto& target_point = green_points_[current_point_index_];
        
        static auto last_feedback_time = this->now();
        auto current_time = this->now();
        
        if ((current_time - last_feedback_time).seconds() >= 5.0) {
            RCLCPP_INFO(this->get_logger(), 
                       "%s'e gidiyor... Mevcut: (%.2f, %.2f), Kalan: %.2fm", 
                       target_point.name.c_str(),
                       feedback->current_pose.pose.position.x,
                       feedback->current_pose.pose.position.y,
                       feedback->distance_remaining);
            last_feedback_time = current_time;
        }
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        const auto& target_point = green_points_[current_point_index_];
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), " %s'e başarıyla ulaşıldı!", target_point.name.c_str());
                
                current_point_index_++;
                
                if (current_point_index_ < green_points_.size()) {
                    RCLCPP_INFO(this->get_logger(), "3 saniye sonra %s'e gidilecek...", 
                               green_points_[current_point_index_].name.c_str());
                    
                    timer_ = this->create_wall_timer(
                        std::chrono::seconds(3),
                        std::bind(&GreenPointNavigator::navigate_to_next_point, this));
                } else {
                    RCLCPP_INFO(this->get_logger(), "Tüm green point'lere başarıyla gidildi! Görev tamamlandı.");
                    rclcpp::shutdown();
                }
                break;
                
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "%s hedefe ulaşılamadı! (İptal edildi)", target_point.name.c_str());
                rclcpp::shutdown();
                break;
                
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "%s hedefe navigasyon durduruldu!", target_point.name.c_str());
                rclcpp::shutdown();
                break;
                
            default:
                RCLCPP_ERROR(this->get_logger(), "Bilinmeyen sonuç!");
                rclcpp::shutdown();
                break;
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<GreenPoint> green_points_;
    size_t current_point_index_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GreenPointNavigator>();
    
    RCLCPP_INFO(node->get_logger(), "Green Point Navigator çalışıyor...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}