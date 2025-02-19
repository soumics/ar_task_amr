#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Nav2WaypointFollower : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav2WaypointFollower() : Node("nav2_waypoint_follower"), goal_index_(0)
    {
        // Create an action client for navigation
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Publisher for setting initial pose
        //initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

        // Define waypoints (Modify as needed)
        waypoints_ = {
            {3.0, -2.0, 0.0},
            {4.0, 2.0, 1.57},
            {2.0, 0.0, -1.57}
        };

        // Set initial pose
        //set_initial_pose();

        // Wait for the action server to be available
        while (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
        }

        // Send the first goal
        send_next_goal();
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

    struct Waypoint
    {
        double x, y, yaw;
    };
    std::vector<Waypoint> waypoints_;
    size_t goal_index_;

    void send_next_goal()
    {
        if (goal_index_ >= waypoints_.size())
        {
            RCLCPP_INFO(this->get_logger(), "All goals reached.");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = waypoints_[goal_index_].x;
        goal_msg.pose.pose.position.y = waypoints_[goal_index_].y;
        goal_msg.pose.pose.orientation.z = sin(waypoints_[goal_index_].yaw / 2);
        goal_msg.pose.pose.orientation.w = cos(waypoints_[goal_index_].yaw / 2);

        RCLCPP_INFO(this->get_logger(), "Sending goal %ld: (%.2f, %.2f)", goal_index_, waypoints_[goal_index_].x, waypoints_[goal_index_].y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&Nav2WaypointFollower::goal_result_callback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_result_callback(const GoalHandleNav::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "reached");
            goal_index_++;
            send_next_goal();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to reach goal.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2WaypointFollower>());
    rclcpp::shutdown();
    return 0;
}
