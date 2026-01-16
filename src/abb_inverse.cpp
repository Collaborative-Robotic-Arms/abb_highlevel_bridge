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
// ---------------------

// --- FIXED ORIENTATION ---
// Last known fixed orientation requested (W, X, Y, Z)
static constexpr double TARGET_QW = 0.0; 
static constexpr double TARGET_QX = 0.70711;
static constexpr double TARGET_QY = 0.0;
static constexpr double TARGET_QZ = 0.7071;

// Helper struct to define target positions
struct TargetPosition {
    double x;
    double y;
    double z;
};

class MoveItPoseController : public rclcpp::Node
{
public:
    // FIX: Pass rclcpp::NodeOptions() to the base constructor.
    // This tells the node to use the name assigned by the launch environment (the Node action).
    MoveItPoseController() : Node("abb_inverse_control", rclcpp::NodeOptions())
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MoveItPoseController...");
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
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(),
                ROBOT_GROUP_NAME
            );

            // Set the end effector link
            move_group_->setEndEffectorLink(END_EFFECTOR_LINK);

            // --- INCREASED SOLVER ROBUSTNESS ---
            
            // Set position tolerance to 1 mm (0.001 m)
            move_group_->setGoalPositionTolerance(0.001); 
            
            // Set orientation tolerance to ~1 degree (0.017 radians)
            move_group_->setGoalOrientationTolerance(0.017); 
            
            // Set planning time (optional but recommended)
            move_group_->setPlanningTime(10.0); // seconds
            
            // ---------------------------------------------------

            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for group: %s", ROBOT_GROUP_NAME.c_str());
            RCLCPP_INFO(this->get_logger(), "End Effector Link set to: %s", move_group_->getEndEffectorLink().c_str());
            RCLCPP_INFO(this->get_logger(), "Goal Tolerances set (Pos: 1mm, Orient: 1 deg).");

        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
            move_group_.reset();
        }
    }

    /**
     * @brief Plans and executes a motion to a target position AND a specific orientation. (Pose IK with Tolerance)
     * @param x The target X coordinate.
     * @param y The target Y coordinate.
     * @param z The target Z coordinate.
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
        
        RCLCPP_INFO(this->get_logger(), "Attempting to move %s to pose: x=%.4f, y=%.4f, z=%.4f", 
                    ROBOT_GROUP_NAME.c_str(), x, y, z);

        // 3. Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        // Increase planning attempts
        move_group_->setNumPlanningAttempts(10); 
        
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); 

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing trajectory...");
            
            // 4. Execute the trajectory
            moveit::core::MoveItErrorCode execute_result = move_group_->execute(plan);
            
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "Execution complete.");
                move_group_->clearPoseTargets();
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
            return false;
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Use a multi-threaded executor for MoveIt nodes
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    
    // Create the controller node
    // It now uses rclcpp::NodeOptions() in its constructor to inherit the launch name.
    auto controller = std::make_shared<MoveItPoseController>(); 
    
    // Add the node to the executor FIRST
    executor->add_node(controller);

    // Spin the executor in a separate thread
    std::thread executor_thread([&]() { executor->spin(); });

    // NOW initialize MoveGroupInterface (after node is in executor)
    controller->initialize();

    // --- Motion Commands (15 points, 5 second delay, Pose IK with Tolerance) ---

    // Define the 15 target positions (mm to m conversion applied).
    // Z is set to 1.035m (1035mm) for all points.
    const std::vector<TargetPosition> target_positions = {
        {0.360, 0.000, 1.035},    
        {0.360, 0.100, 1.035},    
        {0.360, -0.170, 1.035},   
        {0.510, -0.170, 1.035},   
        {0.510, -0.130, 1.035},   
        {0.270, 0.230, 1.035},    
        {0.430, 0.250, 1.035},    
        {0.490, -0.090, 1.035},   
        {0.530, -0.020, 1.035},
        {0.430, 0.200, 1.035},    
        {0.500, 0.200, 1.035},    
        {0.430, 0.150, 1.035},    
        {0.450, 0.200, 1.035},    
        {0.450, 0.230, 1.035},
        {0.360, 0.000, 1.035},    
    };

    RCLCPP_INFO(controller->get_logger(), "Starting 15-point motion sequence using POSE IK with tolerances (Z=1.035m).");
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Initial delay

    for (size_t i = 0; i < target_positions.size(); ++i)
    {
        const auto& target = target_positions[i];
        
        RCLCPP_INFO(controller->get_logger(), "--- TARGET %zu/%zu ---", i + 1, target_positions.size());
        
        // Command the robot to move to the position and fixed orientation
        controller->move_to_pose(
            target.x, target.y, target.z,
            TARGET_QW, TARGET_QX, TARGET_QY, TARGET_QZ
        );
        
        // 5-second pause between movements
        RCLCPP_INFO(controller->get_logger(), "Pausing for 5 seconds...");
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    
    RCLCPP_INFO(controller->get_logger(), "Motion sequence complete.");
    
    // --- End of Motion Commands ---

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}
