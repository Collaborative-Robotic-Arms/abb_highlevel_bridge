#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <memory>
#include <thread>
#include <vector>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// --- Configuration ---
// These must match your robot's setup in the SRDF and configuration files
static const std::string ROBOT_GROUP_NAME = "irb120_arm"; 
static const std::string END_EFFECTOR_LINK = "tool0";     
static const std::string POSE_TOPIC_NAME = "/target_pose_abb";
// ---------------------

class MoveItPoseController : public rclcpp::Node
{
public:
    MoveItPoseController() : Node("abb_inverse_control", rclcpp::NodeOptions())
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveItPoseController...");
        
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s for real-time IK control.", POSE_TOPIC_NAME.c_str());
        
    }

    // Initialize MoveGroupInterface AFTER the node is added to executor
    void initialize()
    {
        try
        {
            // Give the planning components time to initialize on the ROS 2 graph
            
            
            rclcpp::sleep_for(std::chrono::seconds(2));

            RCLCPP_INFO(this->get_logger(), "Creating MoveGroupInterface...");

            // Now shared_from_this() will work because node is managed by shared_ptr
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), ROBOT_GROUP_NAME);

            RCLCPP_INFO(this->get_logger(), "Reference Planning Frame: %s", move_group_->getPlanningFrame().c_str());
            RCLCPP_INFO(this->get_logger(), "End Effector Link: %s", move_group_->getEndEffectorLink().c_str());
            // Set the end effector link
            move_group_->setEndEffectorLink(END_EFFECTOR_LINK);

            // --- Increased Solver Robustness ---
            
            // Set position tolerance to 1 mm (0.001 m)
            move_group_->setGoalPositionTolerance(0.01); 
            
            // Set orientation tolerance to ~1 degree (0.017 radians)
            move_group_->setGoalOrientationTolerance(0.017); 
            
            move_group_->setPoseReferenceFrame("base_link");
            // Set planning time
            move_group_->setPlanningTime(10.0); // seconds
            move_group_->setNumPlanningAttempts(10); // Increase attempts

            // -----------------------------------
            // 1. Initialize the subscriber in the constructor
            pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                POSE_TOPIC_NAME,
                rclcpp::QoS(1).transient_local(), // QoS: Keep last message, make it latching for new subscribers
                std::bind(&MoveItPoseController::pose_callback, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for group: %s", ROBOT_GROUP_NAME.c_str());
            RCLCPP_INFO(this->get_logger(), "End Effector Link set to: %s", move_group_->getEndEffectorLink().c_str());

        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
            move_group_.reset();
        }
    }

    /**
     * @brief Plans and executes a motion to a target position AND a specific orientation.
     * @param x, y, z The target position coordinates.
     * @param qw, qx, qy, qz The quaternion orientation components.
     * @return true if planning and execution were successful, false otherwise.
     */
    bool move_to_pose(double x, double y, double z, double qw, double qx, double qy, double qz)
    {
        if (!move_group_)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveGroup is not initialized. Cannot move.");
            return false;
        }
        move_group_->setStartStateToCurrentState();
    
        // Optional: A tiny sleep to allow the callbacks to process the latest /joint_states
        // strictly only if you are hitting this error constantly
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // 1. Define the target pose (Position and Orientation)
        geometry_msgs::msg::Pose target_pose;
        
        target_pose.position.x = x; //x; //y + 0.7; //x;//0.45;//x;
        target_pose.position.y = y; //x - 0.1; //0.23;//y + 0.874; // Note the negative sign for ABB's coordinate system
        target_pose.position.z = 0.30; //z; //;1.035;
        // 0.450, 0.230
        // target_pose.orientation.w = qw;
        target_pose.orientation.x = -0.0;
        // target_pose.orientation.y = 1.0; //qy;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;
        target_pose.orientation.x = qz;
        target_pose.orientation.y = qw; // 1.0; // Note the negative sign
        // target_pose.orientation.z = -0.0;
        // z: 1.035}, orientation: {w: 0.0, x: 0.70711, y: 0.0, z: 0.7071}}}"
        // 2. Set the target pose
        // 1. Get the current joints (Where we definitely ARE)
        // std::vector<double> current_joints = move_group_->getCurrentJointValues();
        
        // RCLCPP_INFO(this->get_logger(), "Current Joints: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]", 
        //             current_joints[0], current_joints[1], current_joints[2], 
        //             current_joints[3], current_joints[4], current_joints[5]);
        // // 2. Modify one joint slightly (e.g., Joint 1 + 5 degrees)
        // current_joints[1] += 0.28; 

        // // 3. Set target
        // move_group_->setJointValueTarget(current_joints);

        // 4. Plan & Execute
        RCLCPP_INFO(this->get_logger(), "Attempting JOINT target...");
        // moveit::planning_interface::MoveGroupInterface::Plan plan;
        // auto const ok = move_group_->plan(plan);
        move_group_->setPoseTarget(target_pose);
        // // move_group_->move_to_position(
        //     target_pose.position.x, target_pose.position.y, target_pose.position.z
        // );
        RCLCPP_INFO(this->get_logger(), "Attempting to move %s to pose: P(%.4f, %.4f, %.4f) | Q(%.2f, %.2f, %.2f, %.2f)", 
                    ROBOT_GROUP_NAME.c_str(), x, y, z, qw, qx, qy, qz);

        // 3. Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); 

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing trajectory...");
            
            // 4. Execute the trajectory
            moveit::core::MoveItErrorCode execute_result = move_group_->execute(plan);
            
            move_group_->clearPoseTargets(); // Clear target after attempt
            
                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_INFO(this->get_logger(), "Execution complete.");
                    return true;
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Execution failed with code: %d", execute_result.val);
                    return false;
                }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Planning failed. Check for unreachable pose or collisions.");
            move_group_->clearPoseTargets(); // Clear target even if planning failed
            return false;
        }
    }

private:
    // 2. Add the subscription member
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    /**
     * @brief Callback executed when a new PoseStamped message is received.
     */
    /**
     * @brief Callback executed when a new PoseStamped message is received.
     */
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "--- NEW COMMAND RECEIVED ---");

        // --- ROBUST STATE CHECK START ---
        moveit::core::RobotStatePtr current_state;
        bool state_received = false;

        // Try up to 5 times (total 0.25 seconds wait) to get the state
        for (int i = 0; i < 5; ++i) {
            current_state = move_group_->getCurrentState();
            if (current_state) {
                state_received = true;
                break;
            }
            // Sleep 50ms to allow /joint_states message to arrive
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (!state_received) {
            RCLCPP_WARN(this->get_logger(), "IGNORING COMMAND: Could not fetch Robot State after retries. (Sim Lag?)");
            return; 
        }
        
        // Double check timestamp (Time 0.0 means empty state)
        if (current_state->getVariableCount() == 0) { 
             RCLCPP_WARN(this->get_logger(), "IGNORING COMMAND: Robot state is empty.");
             return;
        }
        // --- ROBUST STATE CHECK END ---

        RCLCPP_INFO(this->get_logger(), "Target received in frame: %s", msg->header.frame_id.c_str());

        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;

        // Call the move function
        this->move_to_pose(p.x, p.y, p.z, q.w, q.x, q.y, q.z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // 1. Create Node
    auto controller = std::make_shared<MoveItPoseController>(); 

    // 2. Initialize Logic (Create MoveGroup + Subscribers)
    // Doing this BEFORE the executor spins ensures registration is atomic
    controller->initialize();

    // 3. Create Executor
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(controller);

    RCLCPP_INFO(controller->get_logger(), "Controller is ready. Spinning...");

    // 4. Spin (Blocking - no need for a separate thread if we just want to run)
    executor->spin();

    rclcpp::shutdown();
    return 0;
}