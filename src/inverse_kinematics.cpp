#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <memory>
#include <thread>
#include <vector>

// --- Configuration ---
// These must match your robot's setup in the SRDF and configuration files
static const std::string ROBOT_GROUP_NAME = "irb120_arm"; 
static const std::string END_EFFECTOR_LINK = "tool0";     
static const std::string POSE_TOPIC_NAME = "/target_pose";
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


            // Set the end effector link
            move_group_->setEndEffectorLink(END_EFFECTOR_LINK);

            // --- Increased Solver Robustness ---
            
            // Set position tolerance to 1 mm (0.001 m)
            move_group_->setGoalPositionTolerance(0.001); 
            
            // Set orientation tolerance to ~1 degree (0.017 radians)
            move_group_->setGoalOrientationTolerance(0.017); 
            
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

        // 1. Define the target pose (Position and Orientation)
        geometry_msgs::msg::Pose target_pose;
        
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        
        target_pose.orientation.w = qw;
        target_pose.orientation.x = qx;
        target_pose.orientation.y = qy;
        target_pose.orientation.z = qz;

        // 2. Set the target pose
        move_group_->setPoseTarget(target_pose);
        
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
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "--- NEW COMMAND RECEIVED ---");
        RCLCPP_INFO(this->get_logger(), "Target received in frame: %s", msg->header.frame_id.c_str());

        // Extract position and orientation from the received message
        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;

        // Call the move function with the received pose data
        this->move_to_pose(p.x, p.y, p.z, q.w, q.x, q.y, q.z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Use a multi-threaded executor for MoveIt nodes
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    // Create the controller node
    auto controller = std::make_shared<MoveItPoseController>(); 
    
    // Add the node to the executor
    executor->add_node(controller);

    // Spin the executor in a separate thread
    std::thread executor_thread([&]() { executor->spin(); });

    // NOW initialize MoveGroupInterface (after node is in executor)
    controller->initialize();

    RCLCPP_INFO(controller->get_logger(), "Controller is running and waiting for target poses on %s...", POSE_TOPIC_NAME.c_str());
    RCLCPP_INFO(controller->get_logger(), "Send a geometry_msgs::msg::PoseStamped message to command the robot.");
    
    // The main thread waits for the executor thread to finish (which won't happen 
    // until rclcpp::shutdown() is called, usually via Ctrl+C).
    executor_thread.join();
    
    rclcpp::shutdown();
    return 0;
}
