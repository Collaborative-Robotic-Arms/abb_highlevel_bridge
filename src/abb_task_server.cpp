#include <memory>
#include <thread>
#include <string>
#include <future>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"

// MoveIt
#include <moveit/move_group_interface/move_group_interface.hpp>

// Custom Interfaces
#include "dual_arms_msgs/action/execute_task.hpp"
#include "abb_robot_msgs/srv/set_rapid_bool.hpp"
#include "abb_robot_msgs/msg/service_responses.hpp"
#include "std_srvs/srv/set_bool.hpp"

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

        // 3. Initialize Gripper Service for MTC / Supervisor
        this->service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        this->gripper_service_server_ = this->create_service<std_srvs::srv::SetBool>(
            "abb_controller/set_gripper", 
            std::bind(&AbbTaskServer::handle_gripper_service, this, _1, _2),
            rmw_qos_profile_services_default,
            this->service_cb_group_ 
        );

        // 4. Initialize ARM Driver Client 
        std::string arm_controller_name = use_sim_ ? 
            "/irb120_trajectory_controller/follow_joint_trajectory" : 
            "/irb120_controller/follow_joint_trajectory"; 

        this->arm_driver_client_ = rclcpp_action::create_client<TrajectoryAction>(
            this, arm_controller_name
        );

        // 5. Initialize Gripper Clients
        if (use_sim_) {
            RCLCPP_INFO(this->get_logger(), "Mode: SIMULATION. Connecting to: %s", arm_controller_name.c_str());
            this->sim_gripper_client_ = rclcpp_action::create_client<TrajectoryAction>(
                this, "/irb120_gripper_controller/follow_joint_trajectory"
            );
        } else {
            RCLCPP_INFO(this->get_logger(), "Mode: REAL ROBOT. Connecting to: %s and RWS Services", arm_controller_name.c_str());
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
            move_group_->setMaxVelocityScalingFactor(0.4);
            move_group_->setMaxAccelerationScalingFactor(0.4);
            
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface Ready for ABB.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt init failed: %s", e.what());
        }
    }

private:
    rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_service_server_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::Client<abb_robot_msgs::srv::SetRAPIDBool>::SharedPtr real_gripper_client_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp_action::Client<TrajectoryAction>::SharedPtr sim_gripper_client_;
    rclcpp_action::Client<TrajectoryAction>::SharedPtr arm_driver_client_;
    bool use_sim_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------");
        RCLCPP_INFO(this->get_logger(), "📦 ABB NEW ORDER RECEIVED: [%s]", goal->task_type.c_str());
        if (goal->task_type != "HOME" && goal->task_type != "RELEASE") {
            RCLCPP_INFO(this->get_logger(), "📍 Target -> Pos [x: %.3f, y: %.3f, z: %.3f]",
                goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
        }
        RCLCPP_INFO(this->get_logger(), "-----------------------------------------------------");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteTask>)
    {
        RCLCPP_ERROR(this->get_logger(), "🛑 ABB EMERGENCY CANCEL RECEIVED! HALTING ARM!");
        
        if (move_group_) move_group_->stop();

        if (arm_driver_client_) {
            arm_driver_client_->async_cancel_all_goals();
            
            auto stop_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                use_sim_ ? "/irb120_trajectory_controller/joint_trajectory" : "/irb120_controller/joint_trajectory", 10);
            trajectory_msgs::msg::JointTrajectory empty_msg;
            empty_msg.header.stamp = this->get_clock()->now();
            empty_msg.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
            stop_pub->publish(empty_msg);
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_gripper_service(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "MTC/Supervisor requested ABB Gripper: %s", request->data ? "OPEN" : "CLOSE");
        
        bool success = control_gripper(request->data); 
        
        response->success = success;
        response->message = success ? "Gripper moved successfully" : "Gripper failed to move";
        // ---> DEBUG ADDED
        if (!success) RCLCPP_ERROR(this->get_logger(), "Gripper service request failed internally.");
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        std::thread{std::bind(&AbbTaskServer::execute, this, _1), goal_handle}.detach();
    }

    #define HANDLE_FAILURE(error_msg) \
        result->success = false; \
        if (goal_handle->is_canceling()) { \
            result->error_message = "Canceled by Supervisor"; \
            goal_handle->canceled(result); \
        } else { \
            result->error_message = error_msg; \
            goal_handle->abort(result); \
            RCLCPP_ERROR(this->get_logger(), "❌ ABB Task Failed: %s", std::string(error_msg).c_str()); \
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
            if (!control_gripper(true)) { HANDLE_FAILURE("PICK: Failed to open gripper"); }
            
            // ---> DEBUG ADDED
            RCLCPP_INFO(this->get_logger(), "PICK: Gripper opened successfully. Moving to pre-grasp.");

            feedback->current_status = "MOVING_TO_PREGRASP";
            goal_handle->publish_feedback(feedback);
            geometry_msgs::msg::Pose pregrasp = goal->target_pose;
            pregrasp.position.z = pregrasp.position.z + 0.1;
            if (!move_to_pose(pregrasp, goal_handle)) { HANDLE_FAILURE("PICK: Failed to reach pregrasp"); }

            // ---> DEBUG ADDED
            RCLCPP_INFO(this->get_logger(), "PICK: Pre-grasp reached. Moving to target grasp pose.");

            feedback->current_status = "MOVING_TO_TARGET";
            goal_handle->publish_feedback(feedback);
            if (!move_to_pose(goal->target_pose, goal_handle)) { HANDLE_FAILURE("PICK: Failed to reach pose"); }

            if (!control_gripper(false)) { HANDLE_FAILURE("PICK: Failed to grasp object"); }
            
            // ---> DEBUG ADDED
            RCLCPP_INFO(this->get_logger(), "PICK: Gripper closed. Retreating to post-grasp pose.");

            geometry_msgs::msg::Pose postgrasp = goal->target_pose;
            postgrasp.position.z = postgrasp.position.z + 0.1;
            if (!move_to_pose(postgrasp, goal_handle)) { HANDLE_FAILURE("PICK: Failed to reach postgrasp"); }
        }
        // --- PLACE ---
        else if (goal->task_type == "PLACE")
        {
            feedback->current_status = "MOVING_TO_PRE_PLACE";
            goal_handle->publish_feedback(feedback);
            geometry_msgs::msg::Pose preplace = goal->target_pose;
            preplace.position.z = preplace.position.z + 0.05;
            if (!move_to_pose(preplace, goal_handle)) { HANDLE_FAILURE("PLACE: Failed to reach pre place"); }

            feedback->current_status = "MOVING_TO_PLACE";
            goal_handle->publish_feedback(feedback);
            if (!move_to_pose(goal->target_pose, goal_handle)) { HANDLE_FAILURE("PLACE: Failed to reach pose"); }

            feedback->current_status = "RELEASING_OBJECT";
            goal_handle->publish_feedback(feedback);
            if (!control_gripper(true)) { HANDLE_FAILURE("PLACE: Failed to release object"); }
            
            feedback->current_status = "RETURNING_HOME";
            goal_handle->publish_feedback(feedback);
            move_to_named_target("home", goal_handle);
        }
        else if (goal->task_type == "HOME")
        {
            if (!move_to_named_target("home", goal_handle)) { HANDLE_FAILURE("HOME: Failed to reach home"); }
        }
        // --- HANDOVER STATES ---
        else if (goal->task_type == "INTERMEDIATE_GIVE")
        {
            feedback->current_status = "MOVING_TO_HANDOVER_ZONE";
            goal_handle->publish_feedback(feedback);
            if (!move_to_pose(goal->target_pose, goal_handle)) { HANDLE_FAILURE("INTERMEDIATE_GIVE: Failed to reach pose"); }
        }
        else if (goal->task_type == "INTERMEDIATE_TAKE")
        {
            feedback->current_status = "OPENING_GRIPPER_FOR_TAKE";
            goal_handle->publish_feedback(feedback);
            if (!control_gripper(true)) { HANDLE_FAILURE("INTERMEDIATE_TAKE: Failed to open gripper"); }
            
            feedback->current_status = "MOVING_TO_HANDOVER_APPROACH";
            goal_handle->publish_feedback(feedback);
            
            geometry_msgs::msg::Pose pregrasp_handover = goal->target_pose;
            pregrasp_handover.position.z += 0.10; // 10cm Safety Offset
            
            if (!move_to_pose(pregrasp_handover, goal_handle)) { 
                HANDLE_FAILURE("INTERMEDIATE_TAKE: Failed to reach approach pose"); 
            }
            
            feedback->current_status = "MOVING_TO_HANDOVER_GRASP";
            goal_handle->publish_feedback(feedback);
            if (!move_to_pose(goal->target_pose, goal_handle)) { HANDLE_FAILURE("INTERMEDIATE_TAKE: Failed to reach pose"); }

            feedback->current_status = "CLOSING_GRIPPER_TO_TAKE";
            goal_handle->publish_feedback(feedback);
            if (!control_gripper(false)) { HANDLE_FAILURE("INTERMEDIATE_TAKE: Failed to grasp"); }
        }
        else if (goal->task_type == "RELEASE")
        {
            feedback->current_status = "RELEASING_BRICK";
            goal_handle->publish_feedback(feedback);
            if (!control_gripper(true)) { HANDLE_FAILURE("RELEASE: Failed to open gripper"); }
        }
        else {
            HANDLE_FAILURE("Task " + goal->task_type + " not implemented for ABB");
        }

        if (goal_handle->is_canceling()) { HANDLE_FAILURE("Canceled at finish"); }

        result->success = true;
        result->error_message = "None";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "✅ ABB Task Completed Successfully.");
    }

    bool move_to_pose(const geometry_msgs::msg::Pose & target, std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        if (goal_handle->is_canceling()) return false;

        // ---> DEBUG ADDED: Log exact pose request to check for coordinate issues
        RCLCPP_INFO(this->get_logger(), "Planning to Pose Target: Pos [%.3f, %.3f, %.3f], Ori [%.3f, %.3f, %.3f, %.3f]", 
            target.position.x, target.position.y, target.position.z,
            target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);

        geometry_msgs::msg::PoseStamped stamped_target;
        stamped_target.header.frame_id = "abb_table";
        stamped_target.pose = target;

        move_group_->setPoseTarget(stamped_target);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto error_code = move_group_->plan(plan);

        if (goal_handle->is_canceling()) {
            // ---> DEBUG ADDED
            RCLCPP_WARN(this->get_logger(), "Plan was successful, but goal was cancelled by supervisor before execution.");
            move_group_->clearPoseTargets();
            return false; 
        }

        if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
            // ---> DEBUG ADDED
            RCLCPP_INFO(this->get_logger(), "MoveIt Planning SUCCESS. Trajectory points: %zu. Sending to driver...", plan.trajectory.joint_trajectory.points.size());
            
            bool success = execute_trajectory_via_driver(plan.trajectory.joint_trajectory, goal_handle);
            if (!success) {
                // ---> DEBUG ADDED
                RCLCPP_ERROR(this->get_logger(), "MoveIt planned successfully, but execution via hardware/sim driver FAILED.");
            }
            move_group_->clearPoseTargets();
            return success;
        }
        
        // ---> DEBUG ADDED: Explicit error logging when MoveIt cannot find an IK/Path solution
        RCLCPP_ERROR(this->get_logger(), "ABB Planning Failed! MoveIt Error Code: %d (IK failure or out of bounds)", error_code.val);
        move_group_->clearPoseTargets();
        return false;
    }

    bool move_to_named_target(const std::string & name, std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
        if (goal_handle->is_canceling()) return false;

        if (move_group_->getNamedTargets().empty()) {
             RCLCPP_WARN(this->get_logger(), "No named targets found in SRDF!");
             return false;
        }
        
        move_group_->setNamedTarget(name);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        auto error_code = move_group_->plan(plan);
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
             if (goal_handle->is_canceling()) return false; 
             return execute_trajectory_via_driver(plan.trajectory.joint_trajectory, goal_handle);
        }
        
        // ---> DEBUG ADDED
        RCLCPP_ERROR(this->get_logger(), "Failed to plan to named target '%s'. Error code: %d", name.c_str(), error_code.val);
        return false;
    }

    bool execute_trajectory_via_driver(const trajectory_msgs::msg::JointTrajectory& trajectory, std::shared_ptr<GoalHandleExecuteTask> server_goal_handle)
    {
        if (!arm_driver_client_->wait_for_action_server(std::chrono::seconds(2))) {
            // ---> DEBUG ADDED
            RCLCPP_ERROR(this->get_logger(), "Execution Failed: Arm driver action server is not running or unreachable.");
            return false;
        }

        auto goal_msg = TrajectoryAction::Goal();
        goal_msg.trajectory = trajectory;

        auto goal_handle_future = arm_driver_client_->async_send_goal(goal_msg);
        if (goal_handle_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
            // ---> DEBUG ADDED
            RCLCPP_ERROR(this->get_logger(), "Execution Failed: Timed out sending goal to arm driver.");
            return false;
        }

        auto action_goal_handle = goal_handle_future.get();
        if (!action_goal_handle) {
            // ---> DEBUG ADDED
            RCLCPP_ERROR(this->get_logger(), "Execution Failed: Goal was rejected by the arm driver (Controller issue).");
            return false;
        }

        auto result_future = arm_driver_client_->async_get_result(action_goal_handle);
        
        while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if (server_goal_handle->is_canceling()) {
                RCLCPP_WARN(this->get_logger(), "Execution aborted mid-trajectory due to Supervisor cancel.");
                arm_driver_client_->async_cancel_all_goals();
                return false;
            }
        }

        auto result = result_future.get();
        
        // ---> DEBUG ADDED: Check the exact result code from the FollowJointTrajectory action
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "Arm driver execution aborted! Action Server Result Code: %d", static_cast<int>(result.code));
            return false;
        }

        return true;
    }

    // =========================================================================
    // RESTORED GRIPPER HELPERS
    // =========================================================================

    bool control_gripper(bool open) {
        // ---> DEBUG ADDED
        RCLCPP_INFO(this->get_logger(), "Executing Gripper Command: %s", open ? "OPEN" : "CLOSE");
        if (use_sim_) return send_sim_gripper_command(open);
        else return send_real_gripper_command(open);
    }

    bool send_real_gripper_command(bool open)
    {
        if (!real_gripper_client_->wait_for_service(std::chrono::seconds(2))) {
            // ---> DEBUG ADDED
            RCLCPP_ERROR(this->get_logger(), "Real gripper service /rws_client/set_gripper_state timed out.");
            return false;
        }
        
        auto request = std::make_shared<abb_robot_msgs::srv::SetRAPIDBool::Request>();
        request->path.task = "T_ROB1";
        request->path.module = "egm";
        request->path.symbol = "gripper_close";
        request->value = open; 
        
        auto future = real_gripper_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
            auto res = future.get();
            // ---> DEBUG ADDED
            if (res->result_code != abb_robot_msgs::msg::ServiceResponses::RC_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Real Gripper Rejected Command. Result code: %d", res->result_code);
                return false;
            }
            return true;
        }
        
        // ---> DEBUG ADDED
        RCLCPP_ERROR(this->get_logger(), "Real gripper service call timed out while waiting for response.");
        return false;
    }

    bool send_sim_gripper_command(bool open)
    {
        if (!sim_gripper_client_->wait_for_action_server(std::chrono::seconds(2))) {
            // ---> DEBUG ADDED
            RCLCPP_ERROR(this->get_logger(), "Simulated gripper action server unreachable!");
            return false;
        }
        
        auto goal_msg = TrajectoryAction::Goal();
        goal_msg.trajectory.joint_names = {
            "gripper_ABB_Gripper_Finger_1_Joint", 
            "gripper_ABB_Gripper_Finger_2_Joint"
        };

        trajectory_msgs::msg::JointTrajectoryPoint point;
        double pos = open ? 0.0135 : 0.001; 
        point.positions = {pos, pos};
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        goal_msg.trajectory.points.push_back(point);

        auto goal_handle_future = sim_gripper_client_->async_send_goal(goal_msg);
        if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
             // ---> DEBUG ADDED
             RCLCPP_ERROR(this->get_logger(), "Timeout sending goal to sim gripper.");
             return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
             // ---> DEBUG ADDED
             RCLCPP_ERROR(this->get_logger(), "Sim Gripper goal rejected!");
             return false;
        }

        auto result_future = sim_gripper_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto result = result_future.get();
            // ---> DEBUG ADDED
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_ERROR(this->get_logger(), "Sim Gripper action failed with code: %d", static_cast<int>(result.code));
                return false;
            }
            return true;
        }
        
        // ---> DEBUG ADDED
        RCLCPP_ERROR(this->get_logger(), "Sim Gripper action execution timed out.");
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