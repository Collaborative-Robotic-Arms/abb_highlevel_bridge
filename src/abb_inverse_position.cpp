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

// Helper struct to define target positions
struct TargetPosition {
    double x;
    double y;
    double z;
};

class MoveItPoseController : public rclcpp::Node
{
public:
    MoveItPoseController() : Node("abb_inverse_control")
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

            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for group: %s", ROBOT_GROUP_NAME.c_str());
            RCLCPP_INFO(this->get_logger(), "End Effector Link set to: %s", move_group_->getEndEffectorLink().c_str());

            // Set a planning time (optional but recommended)
            move_group_->setPlanningTime(5.0); // seconds

        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
            move_group_.reset();
        }
    }

    /**
     * @brief Plans and executes a motion to a target position, ignoring orientation. (Position Only IK)
     * @param x The target X coordinate.
     * @param y The target Y coordinate.
     * @param z The target Z coordinate.
     * @return true if planning and execution were successful, false otherwise.
     */
    bool move_to_position(double x, double y, double z)
    {
        if (!move_group_)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveGroup is not initialized. Cannot move.");
            return false;
        }

        // 1. Set the target using position only (Ignoring orientation)
        // This is the key change for position-only IK.
        move_group_->setPositionTarget(x, y, z);
        
        RCLCPP_INFO(this->get_logger(), "Attempting to move %s to position only: x=%.4f, y=%.4f, z=%.4f", 
                    ROBOT_GROUP_NAME.c_str(), x, y, z);

        // 2. Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); 

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing trajectory...");
            
            // 3. Execute the trajectory
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
            RCLCPP_WARN(this->get_logger(), "Planning failed. Check for unreachable position or collisions.");
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
    auto controller = std::make_shared<MoveItPoseController>();
    
    // Add the node to the executor FIRST
    executor->add_node(controller);

    // Spin the executor in a separate thread
    std::thread executor_thread([&]() { executor->spin(); });

    // NOW initialize MoveGroupInterface (after node is in executor)
    controller->initialize();

    // --- New Motion Commands (14 points, 5 second delay, Position Only) ---

    // Define the 14 new target positions. 
    // All original (X, Y, Z) values were divided by 1000 (mm to m), and Z had 0.03 (30 mm) added.
    const std::vector<TargetPosition> target_positions = {
        {0.760, 0.000, 1.635},    // (360, 0, 5) -> (0.360, 0.000, 0.005 + 0.03)
        {0.760, 0.100, 1.635},    // (360, 100, 5)
        {0.760, -0.170, 1.635},   // (360, -170, 5)
        {0.910, -0.170, 1.635},   // (510, -170, 5)
        {0.910, -0.130, 1.635},   // (510, -130, 5)
        {0.770, 0.230, 1.635},    // (270, 230, 5)
        {0.830, 0.250, 1.635},    // (430, 250, 5)
        {0.890, -0.090, 1.635},   // (490, -90, 5)
        {0.930, -0.020, 1.635},   // (530, -20, 5)
        {0.830, 0.200, 1.635},    // (430, 200, 5)
        {0.900, 0.200, 1.635},    // (500, 200, 5)
        {0.830, 0.150, 1.635},    // (430, 150, 5)
        {0.850, 0.200, 1.635},    // (450, 200, 5)
        {0.850, 0.230, 1.635}     // (450, 230, 5)
    };

    RCLCPP_INFO(controller->get_logger(), "Starting 14-point motion sequence using POSITION ONLY IK.");
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Initial delay

    for (size_t i = 0; i < target_positions.size(); ++i)
    {
        const auto& target = target_positions[i];
        
        RCLCPP_INFO(controller->get_logger(), "--- TARGET %zu/%zu ---", i + 1, target_positions.size());
        
        // Command the robot to move to the position (X, Y, Z only)
        controller->move_to_position(
            target.x, target.y, target.z
        );
        
        // 5-second pause between movements
        RCLCPP_INFO(controller->get_logger(), "Pausing for 5 seconds...");
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    
    RCLCPP_INFO(controller->get_logger(), "Motion sequence complete.");
    
    // --- End of Motion Commands ---

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}
