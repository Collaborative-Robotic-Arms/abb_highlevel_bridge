#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// REMOVE THIS: #include "abb_robot_msgs/srv/set_io_signal.hpp"
#include <abb_robot_msgs/srv/set_rapid_bool.hpp> // Include the correct service header
#include <abb_robot_msgs/msg/rapid_symbol_path.hpp> // Include the RapidSymbolPath message header

#include <chrono>
#include <memory>
#include <string>
#include <iostream> // For std::cin, std::cout
#include <thread>   // For std::thread

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>  
#include <abb_robot_msgs/srv/set_rapid_bool.hpp>
#include <abb_robot_msgs/msg/rapid_symbol_path.hpp>
#include <abb_robot_msgs/msg/service_responses.hpp>

using namespace std::chrono_literals;


class EndEffectorClient : public rclcpp::Node
{
public:
  EndEffectorClient()
  : Node("end_effector_bridge")
  {
    client_ = this->create_client<abb_robot_msgs::srv::SetRAPIDBool>("/rws_client/set_gripper_state");

    gripper_command_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(  
      "/gripper_command",
      10,
      std::bind(&EndEffectorClient::gripper_command_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "EndEffector bridge node initialized.");
    RCLCPP_INFO(this->get_logger(), "Waiting for '/rws_client/set_gripper_state' service to be available...");
  }

  // Public method to send the gripper command based on user input
  void send_gripper_command(bool open_gripper)
  {
    // Wait for service to be available before sending request
    // This is non-blocking if called from a different thread than the executor's spin
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Service '/rws_client/set_gripper_state' not available. Waiting...");
      return;
    }

    auto request = std::make_shared<abb_robot_msgs::srv::SetRAPIDBool::Request>();

    // Populate the RapidSymbolPath for the request
    // IMPORTANT: These must match what you configured in rws_service_provider_ros.cpp
    request->path.task = "T_Gripper";          // Default RAPID task name
    request->path.module = "egm"; // The module where your gripper bools are (e.g., MainModule, GripperControl)

    std::string action_description;

    if (open_gripper) {
      request->path.symbol = "gripper_close"; // RAPID bool for closing
      request->value = true;                  // Set to true
      action_description = "OPEN";
    } else {
      request->path.symbol = "gripper_close"; // RAPID bool for opening
      request->value = false;                  // Set to false
      action_description = "CLOSE";
    }

    RCLCPP_INFO(this->get_logger(), "Attempting to %s gripper via RAPID Bool: Task='%s', Module='%s', Symbol='%s', Value=%s",
                action_description.c_str(),
                request->path.task.c_str(),
                request->path.module.c_str(),
                request->path.symbol.c_str(),
                request->value ? "true" : "false");

    // Send the asynchronous service request
    // auto result_future = client_->async_send_request(request);
    client_->async_send_request(
  request,
  [this, open_gripper](rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedFuture future)
  {
    auto response = future.get();
    if (response->result_code == abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Gripper %s command successful. Message: %s",
                  open_gripper ? "OPEN" : "CLOSE", response->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to %s gripper. Result Code: %d, Message: %s",
                   open_gripper ? "OPEN" : "CLOSE", response->result_code, response->message.c_str());
    }
  });
    // RCLCPP_INFO(this->get_logger(), "BBBBBBBBBBBBBBBB");
    // // Using a callback for the future to avoid blocking the main thread
    // // The callback will be executed by the executor's thread
    // result_future.wait(); // Block this call until the future is complete
    //                       // This is fine because this method is called from the main thread,
    //                       // while the executor is spinning in a background thread.

    // RCLCPP_INFO(this->get_logger(), "AAAAAAAAAAA");
    // if (result_future.wait_for(0s) == std::future_status::ready) // Check if future is ready immediately
    // {
    //   auto response = result_future.get();
    //   if (response->result_code == abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS) // Assuming 0 is success
    //   {
    //     RCLCPP_INFO(this->get_logger(), "Gripper %s command successful. Message: %s",
    //                 action_description.c_str(), response->message.c_str());
    //   }
    //   else
    //   {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to %s gripper. Result Code: %d, Message: %s",
    //                  action_description.c_str(), response->result_code, response->message.c_str());
    //   }
    // }
    // else
    // {
    //   RCLCPP_ERROR(this->get_logger(), "Service call to SetRAPIDBool timed out or failed to complete.");
    // }
  }

private:
  rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_command_subscriber_; 

  void gripper_command_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received gripper command via topic: %s", msg->data ? "OPEN" : "CLOSE");
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    send_gripper_command(msg->data);
  }
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<EndEffectorClient>();

  // Create a MultiThreadedExecutor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Spin the executor in a separate thread
  // This allows the main thread to handle user input
  std::thread executor_thread([&]() {
    executor.spin();
  });

  while (rclcpp::ok()) { // Keep looping as long as ROS 2 is running
    
    // Small delay to prevent busy-waiting if input is very fast
    std::this_thread::sleep_for(100ms);
  }

  // Shutdown ROS 2
  rclcpp::shutdown();

  // Join the executor thread to ensure it finishes cleanly
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  return 0;
}

