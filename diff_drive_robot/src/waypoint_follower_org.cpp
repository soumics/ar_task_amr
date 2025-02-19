#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

class NavigateToPoseClient : public rclcpp::Node
{
public:
  NavigateToPoseClient() : Node("navigate_to_pose_client")
  {
    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

    // Wait for the action server to be available
    while (!client_->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for action server to be available...");
    }

    // Send the goal
    send_goal();
  }

private:
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;

  void send_goal()
  {
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    
    // Define the pose
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = 4.0;
    goal_msg.pose.pose.position.y = 1.0;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    // Send goal
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&NavigateToPoseClient::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending goal...");
    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), "Goal successfully reached!");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Goal failed to reach.");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigateToPoseClient>());
  rclcpp::shutdown();
  return 0;
}
