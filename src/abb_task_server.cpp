#include <memory>
#include <thread>
#include <string>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"

// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>

// Custom Interfaces
#include "supervisor_package/action/execute_task.hpp"
#include "abb_robot_msgs/srv/set_rapid_bool.hpp"
#include "abb_robot_msgs/msg/service_responses.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AbbTaskServer : public rclcpp::Node
{
public:
    using ExecuteTask = supervisor_package::action::ExecuteTask;
    using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    AbbTaskServer() : Node("abb_task_server")
    {
        // 1. Initialize Action Server
        this->action_server_ = rclcpp_action::create_server<ExecuteTask>(
            this,
            "abb_control", 
            std::bind(&AbbTaskServer::handle_goal, this, _1, _2),
            std::bind(&AbbTaskServer::handle_cancel, this, _1),
            std::bind(&AbbTaskServer::handle_accepted, this, _1)
        );

        // 2. Initialize Gripper Service Client
        this->gripper_client_ = this->create_client<abb_robot_msgs::srv::SetRAPIDBool>("/rws_client/set_gripper_state");

        RCLCPP_INFO(this->get_logger(), "ABB Task Server initialized. Waiting for MoveGroup...");
    }

    void initialize_moveit()
    {
        static const std::string ROBOT_GROUP_NAME = "irb120_arm"; 
        try {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), ROBOT_GROUP_NAME);
            move_group_->setEndEffectorLink("tool0");
            move_group_->setGoalPositionTolerance(0.001); 
            move_group_->setGoalOrientationTolerance(0.017); 
            move_group_->setPlanningTime(10.0);
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface Ready for ABB.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt init failed: %s", e.what());
        }
    }

private:
    rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
    rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedPtr gripper_client_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received ABB goal: %s", goal->task_type.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTask>)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        std::thread{std::bind(&AbbTaskServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ExecuteTask::Feedback>();
        auto result = std::make_shared<ExecuteTask::Result>();

        if (!move_group_) {
            result->success = false;
            result->error_message = "MoveGroup not initialized!";
            goal_handle->abort(result);
            return;
        }

        // --- Logic for PICK ---
        if (goal->task_type == "PICK")
        {
            RCLCPP_INFO(this->get_logger(), "Executing standard PICK sequence");
            
            if (!set_gripper_state(true)) { // OPEN
                result->success = false;
                result->error_message = "PICK: Failed to open gripper";
                goal_handle->abort(result);
                return;
            }

            feedback->current_status = "MOVING_TO_PICKUP";
            feedback->progress = 0.5;
            goal_handle->publish_feedback(feedback);
            if (!move_to_pose(goal->target_pose)) {
                result->success = false;
                result->error_message = "PICK: MoveIt failed to reach pose";
                goal_handle->abort(result);
                return;
            }

            if (!set_gripper_state(false)) { // CLOSE
                result->success = false;
                result->error_message = "PICK: Failed to grasp object";
                goal_handle->abort(result);
                return;
            }
        }
        // --- Logic for PLACE ---
        else if (goal->task_type == "PLACE")
        {
            RCLCPP_INFO(this->get_logger(), "Executing standard PLACE sequence");

            feedback->current_status = "MOVING_TO_PLACE";
            feedback->progress = 0.5;
            goal_handle->publish_feedback(feedback);
            if (!move_to_pose(goal->target_pose)) {
                result->success = false;
                result->error_message = "PLACE: MoveIt failed to reach pose";
                goal_handle->abort(result);
                return;
            }

            if (!set_gripper_state(true)) { // OPEN
                result->success = false;
                result->error_message = "PLACE: Failed to release object";
                goal_handle->abort(result);
                return;
            }
        }
        // --- Logic for PICK_FROM_HANDOVER ---
        else if (goal->task_type == "PICK_FROM_HANDOVER")
        {
            if (!set_gripper_state(true)) { // OPEN
                result->success = false;
                result->error_message = "HANDOVER: Failed to open gripper";
                goal_handle->abort(result);
                return;
            }

            feedback->current_status = "GOTO_HANDOVER";
            feedback->progress = 0.5;
            goal_handle->publish_feedback(feedback);
            if (!move_to_pose(goal->target_pose)) {
                result->success = false;
                result->error_message = "HANDOVER: MoveIt failed to reach pose";
                goal_handle->abort(result);
                return;
            }

            if (!set_gripper_state(false)) { // CLOSE
                result->success = false;
                result->error_message = "HANDOVER: Failed to close gripper";
                goal_handle->abort(result);
                return;
            }
        }
        else {
            result->success = false;
            result->error_message = "Task " + goal->task_type + " not implemented for ABB";
            goal_handle->abort(result);
            return;
        }

        result->success = true;
        result->error_message = "None";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "ABB Task Completed Successfully.");
    }

    bool move_to_pose(const geometry_msgs::msg::Pose & target)
    {
        move_group_->setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return false;
    }

    bool set_gripper_state(bool open)
    {
        if (!gripper_client_->wait_for_service(std::chrono::seconds(2))) return false;
        auto request = std::make_shared<abb_robot_msgs::srv::SetRAPIDBool::Request>();
        
        // Corrected Task Name to T_ROB1
        request->path.task = "T_ROB1"; 
        request->path.module = "egm";
        request->path.symbol = "gripper_close";
        request->value = open; // Map: True -> Open (1 in RAPID), False -> Close (0 in RAPID)
        
        auto future = gripper_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
            return (future.get()->result_code == abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS);
        }
        return false;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<AbbTaskServer>();
    executor->add_node(node);
    node->initialize_moveit();
    executor->spin();
    rclcpp::shutdown();
    return 0;
}