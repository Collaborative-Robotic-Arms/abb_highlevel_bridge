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
#include "dual_arms_msgs/action/execute_task.hpp"
#include "abb_robot_msgs/srv/set_rapid_bool.hpp"
#include "abb_robot_msgs/msg/service_responses.hpp"

// Controller Interfaces
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AbbTaskServer : public rclcpp::Node
{
public:
    using ExecuteTask = dual_arms_msgs::action::ExecuteTask;
    using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;
    using TrajectoryAction = control_msgs::action::FollowJointTrajectory;

    AbbTaskServer() : Node("abb_task_server")
    {   
        // 1. Parameters
        this->declare_parameter("use_sim", true);
        this->use_sim_ = this->get_parameter("use_sim").as_bool();
        
        // 2. Initialize Action Server
        this->action_server_ = rclcpp_action::create_server<ExecuteTask>(
            this,
            "abb_control", 
            std::bind(&AbbTaskServer::handle_goal, this, _1, _2),
            std::bind(&AbbTaskServer::handle_cancel, this, _1),
            std::bind(&AbbTaskServer::handle_accepted, this, _1)
        );

        // 3. Initialize ARM Driver Client (The Key to Parallelism)
        this->arm_driver_client_ = rclcpp_action::create_client<TrajectoryAction>(
            this,
            "/irb120_trajectory_controller/follow_joint_trajectory"
        );

        // 4. Initialize Gripper Clients
        if (use_sim_) {
            RCLCPP_INFO(this->get_logger(), "Mode: SIMULATION. Connecting to Trajectory Controller...");
            this->sim_gripper_client_ = rclcpp_action::create_client<TrajectoryAction>(
                this, 
                "/irb120_gripper_controller/follow_joint_trajectory"
            );
        } else {
            RCLCPP_INFO(this->get_logger(), "Mode: REAL ROBOT. Connecting to RWS services...");
            this->real_gripper_client_ = this->create_client<abb_robot_msgs::srv::SetRAPIDBool>(
                "/rws_client/set_gripper_state"
            );
        }

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
            move_group_->setPoseReferenceFrame("abb_table");
            
            // Speed up simulation execution
            move_group_->setMaxVelocityScalingFactor(0.8);
            move_group_->setMaxAccelerationScalingFactor(0.4);
            
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface Ready for ABB.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt init failed: %s", e.what());
        }
    }

private:
    rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
    rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedPtr real_gripper_client_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp_action::Client<TrajectoryAction>::SharedPtr sim_gripper_client_;
    rclcpp_action::Client<TrajectoryAction>::SharedPtr arm_driver_client_;
    bool use_sim_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received ABB goal: %s", goal->task_type.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTask>)
    {
        RCLCPP_ERROR(this->get_logger(), "🛑 ABB EMERGENCY CANCEL RECEIVED! HALTING ARM!");
        
        if (move_group_) {
            move_group_->stop();
        }

        // Physical Freeze for ABB
        if (arm_driver_client_) {
            arm_driver_client_->async_cancel_all_goals();
            auto stop_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/irb120_trajectory_controller/joint_trajectory", 10);
            trajectory_msgs::msg::JointTrajectory empty_msg;
            empty_msg.header.stamp = this->get_clock()->now();
            empty_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
            stop_pub->publish(empty_msg);
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        std::thread{std::bind(&AbbTaskServer::execute, this, _1), goal_handle}.detach();
    }

    // --- HELPER MACRO: Safely exit if Supervisor cancels us mid-execution ---
    #define HANDLE_FAILURE(error_msg) \
        result->success = false; \
        if (goal_handle->is_canceling()) { \
            result->error_message = "Canceled by Supervisor"; \
            goal_handle->canceled(result); \
            RCLCPP_WARN(this->get_logger(), "Task canceled gracefully. Yielding to MTC."); \
        } else { \
            result->error_message = error_msg; \
            goal_handle->abort(result); \
        } \
        return;

    void execute(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ExecuteTask::Feedback>();
        auto result = std::make_shared<ExecuteTask::Result>();

        if (!move_group_) { HANDLE_FAILURE("MoveGroup not initialized!"); }

        // --- PICK ---
        if (goal->task_type == "PICK")
        {
            RCLCPP_INFO(this->get_logger(), "Executing standard PICK sequence");
            
            // 1. Open Gripper (Commented out exactly as in your original file)
            // control_gripper(true);

            // 2. Pre-Grasp Approach
            feedback->current_status = "MOVING_TO_PREGRASP";
            feedback->progress = 0.3;
            goal_handle->publish_feedback(feedback);
            
            geometry_msgs::msg::Pose pregrasp = goal->target_pose;
            pregrasp.position.z = pregrasp.position.z + 0.1;
            
            if (!move_to_pose(pregrasp, goal_handle)) { HANDLE_FAILURE("PICK: Failed to reach pregrasp"); }

            // 3. Move to Target
            feedback->current_status = "MOVING_TO_TARGET";
            feedback->progress = 0.6;
            goal_handle->publish_feedback(feedback);

            if (!move_to_pose(goal->target_pose, goal_handle)) { HANDLE_FAILURE("PICK: Failed to reach pose"); }

            // 4. Close Gripper (Commented out exactly as in your original file)
            // control_gripper(false);
        }
        // --- PLACE ---
        else if (goal->task_type == "PLACE")
        {
            RCLCPP_INFO(this->get_logger(), "Executing standard PLACE sequence");

            // 1. Move to Pre-Place
            feedback->current_status = "MOVING_TO_PRE_PLACE";
            feedback->progress = 0.3;
            goal_handle->publish_feedback(feedback);
            
            geometry_msgs::msg::Pose preplace = goal->target_pose;
            preplace.position.z = preplace.position.z + 0.05;
            
            if (!move_to_pose(preplace, goal_handle)) { HANDLE_FAILURE("PLACE: Failed to reach pre place"); }

            // 2. Move to Place
            feedback->current_status = "MOVING_TO_PLACE";
            feedback->progress = 0.5;
            goal_handle->publish_feedback(feedback);
            
            if (!move_to_pose(goal->target_pose, goal_handle)) { HANDLE_FAILURE("PLACE: Failed to reach pose"); }

            // 3. Release
            feedback->current_status = "RELEASING_OBJECT";
            feedback->progress = 0.8;
            goal_handle->publish_feedback(feedback);
            // control_gripper(true); // OPEN
            
            // 4. Home
            feedback->current_status = "RETURNING_HOME";
            feedback->progress = 0.9;
            goal_handle->publish_feedback(feedback);
            
            if (!move_to_named_target("home", goal_handle)) {
                RCLCPP_WARN(this->get_logger(), "PLACE: Failed to return to HOME");
            }
        }
        else if (goal->task_type == "HOME")
        {
            RCLCPP_INFO(this->get_logger(), "Executing ABB HOME sequence");
            feedback->current_status = "RETURNING_HOME";
            goal_handle->publish_feedback(feedback);

            if (!move_to_named_target("home", goal_handle)) { HANDLE_FAILURE("HOME: Failed to reach home"); }
        }
        else {
            HANDLE_FAILURE("Task " + goal->task_type + " not implemented for ABB");
        }

        // Final cancellation check before claiming success
        if (goal_handle->is_canceling()) { HANDLE_FAILURE("Canceled at finish"); }

        result->success = true;
        result->error_message = "None";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "ABB Task Completed Successfully.");
    }

    bool move_to_pose(const geometry_msgs::msg::Pose & target, std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        if (goal_handle->is_canceling()) return false;

        // 1. Setup MoveIt Goal
        move_group_->setPoseTarget(target);
        
        // 2. Plan (BLOCKING but FAST)
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto error_code = move_group_->plan(plan);

        // CRITICAL: Check if Supervisor canceled us WHILE we were doing the heavy math
        if (goal_handle->is_canceling()) {
            move_group_->clearPoseTargets();
            return false; 
        }

        if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
            // 3. Execute via Driver
            RCLCPP_INFO(this->get_logger(), "Plan successful. Sending to Driver...");
            return execute_trajectory_via_driver(plan.trajectory.joint_trajectory, goal_handle);
        }
        
        RCLCPP_ERROR(this->get_logger(), "Planning Failed!");
        return false;
    }

    bool move_to_named_target(const std::string & name, std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        if (goal_handle->is_canceling()) return false;

        if (move_group_->getNamedTargets().empty()) {
             RCLCPP_WARN(this->get_logger(), "No named targets found.");
             return false;
        }
        
        move_group_->setNamedTarget(name);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
             if (goal_handle->is_canceling()) return false; // CRITICAL CANCEL CHECK
             RCLCPP_INFO(this->get_logger(), "Home Plan successful. Sending to Driver...");
             return execute_trajectory_via_driver(plan.trajectory.joint_trajectory, goal_handle);
        }
        
        RCLCPP_ERROR(this->get_logger(), "Failed to plan to named target: %s", name.c_str());
        return false;
    }

    bool execute_trajectory_via_driver(const trajectory_msgs::msg::JointTrajectory& trajectory, std::shared_ptr<GoalHandleExecuteTask> server_goal_handle)
    {
        if (!arm_driver_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "ABB Driver Action Server not found!");
            return false;
        }

        auto goal_msg = TrajectoryAction::Goal();
        goal_msg.trajectory = trajectory;

        // Send goal asynchronously
        auto goal_handle_future = arm_driver_client_->async_send_goal(goal_msg);
        
        if (goal_handle_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "ABB Driver goal send timed out.");
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "ABB Driver goal rejected.");
            return false;
        }

        // Wait for result
        auto result_future = arm_driver_client_->async_get_result(goal_handle);
        
        // Monitor for C++ Action Cancel while physically moving
        while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if (server_goal_handle->is_canceling()) {
                arm_driver_client_->async_cancel_all_goals();
                return false;
            }
        }

        auto result = result_future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            return true;
        }

        RCLCPP_ERROR(this->get_logger(), "ABB Driver Execution Failed or Timed Out.");
        return false;
    }

    // =========================================================================
    // RESTORED GRIPPER HELPERS
    // =========================================================================

    bool control_gripper(bool open) {
        if (use_sim_) return send_sim_gripper_command(open);
        else return send_real_gripper_command(open);
    }

    bool send_real_gripper_command(bool open)
    {
        if (!real_gripper_client_->wait_for_service(std::chrono::seconds(1))) return false;
        
        auto request = std::make_shared<abb_robot_msgs::srv::SetRAPIDBool::Request>();
        request->path.task = "T_ROB1";
        request->path.module = "egm";
        request->path.symbol = "gripper_close";
        request->value = open; 
        
        auto future = real_gripper_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
            return (future.get()->result_code == abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS);
        }
        return false;
    }

    bool send_sim_gripper_command(bool open)
    {
        if (!sim_gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Sim Gripper Action Server not found!");
            return false;
        }
        
        auto goal_msg = TrajectoryAction::Goal();
        goal_msg.trajectory.joint_names = {
            "gripper_ABB_Gripper_Finger_1_Joint", 
            "gripper_ABB_Gripper_Finger_2_Joint"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        double pos = open ? 0.0120 : 0.005; 
        point.positions = {pos, pos};
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        goal_msg.trajectory.points.push_back(point);

        auto goal_handle_future = sim_gripper_client_->async_send_goal(goal_msg);
        if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) return false;

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) return false;

        auto result_future = sim_gripper_client_->async_get_result(goal_handle);
        return (result_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready);
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