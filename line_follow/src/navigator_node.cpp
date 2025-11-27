#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <random>   
#include <algorithm> 

class Navigator : public rclcpp::Node
{
    public:
        Navigator() : Node("navigator")
        {
            subscriber_ = create_subscription<std_msgs::msg::Bool>("/is_task",10,
            std::bind(&Navigator::task_callback,this,std::placeholders::_1));

            nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this, "follow_waypoints");
            
            RCLCPP_INFO(get_logger(),"NAVIGATOR STARTED!");

            std::srand(std::time(nullptr));
        } 
    private:
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
        rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr nav2_client_;

        bool is_started = false;
        int point_count = 3;

        void task_callback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            if(msg->data == true && !is_started){
                is_started = true;
                navigator_task();
            }
        }

        void navigator_task(){
            if (!nav2_client_->wait_for_action_server(std::chrono::seconds(5))) {
                RCLCPP_ERROR(this->get_logger(), "Nav2 Not Found");
                is_started = false;
                return;
            }

            auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();

            auto all_points = read_waypoints_from_file("/tmp/line_follow_waypoints.txt");

            int i = 0;
            while (i < point_count)
            {
                int all_points_size = all_points.size();
                int start_index = all_points_size / 2;
                int gap = all_points_size - start_index;
                if (gap == 0) break;
                int random_index = (std::rand() % gap) + start_index;

                geometry_msgs::msg::PoseStamped selected_point = all_points[random_index];

                goal_msg.poses.push_back(selected_point);

                all_points.erase(all_points.begin() + random_index, all_points.end());
                i++;
            }

            auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
            nav2_client_->async_send_goal(goal_msg, send_goal_options);
        }

        std::vector<geometry_msgs::msg::PoseStamped> read_waypoints_from_file(const std::string& filepath)
        {
            std::vector<geometry_msgs::msg::PoseStamped> points;
            std::ifstream file(filepath);
            std::string line;

            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Dosya acilamadi: %s", filepath.c_str());
                return points;
            }

            while (std::getline(file, line)) {
                if (line.empty() || line[0] == '#') continue;

                std::stringstream ss(line);
                double x, y, z, qx, qy, qz, qw;
            
                if (ss >> x >> y >> z >> qx >> qy >> qz >> qw) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = "map";
                    pose.header.stamp = this->now();
                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = z;
                    pose.pose.orientation.x = qx;
                    pose.pose.orientation.y = qy;
                    pose.pose.orientation.z = qz;
                    pose.pose.orientation.w = qw;
                    points.push_back(pose);
                }
            }

            file.close();
            return points;
        }
};


int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Navigator>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}