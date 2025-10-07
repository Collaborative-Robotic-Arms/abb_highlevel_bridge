#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

// The topic name where the SRDF content must be published
static const std::string TOPIC_NAME = "/robot_description_semantic";
// Number of times to publish to ensure all late-starting subscribers (like MoveGroupInterface) catch it
static const int MAX_PUBLISH_COUNT = 10; 

class SemanticPublisher : public rclcpp::Node
{
public:
    SemanticPublisher() : Node("semantic_publisher"), publish_count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing SemanticPublisher...");

        // 1. Declare and retrieve the path parameter
        // This parameter MUST be provided when launching the node via CLI or Launch file.
        this->declare_parameter<std::string>("srdf_path", "");
        std::string srdf_path = this->get_parameter("srdf_path").as_string();

        if (srdf_path.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'srdf_path' is empty. Please launch the node with a valid absolute path to the .srdf file.");
            is_ready_ = false;
            return;
        }

        // 2. Load the file content
        RCLCPP_INFO(this->get_logger(), "Attempting to load SRDF from: %s", srdf_path.c_str());
        srdf_content_ = load_file(srdf_path);

        if (srdf_content_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load SRDF content from file. Check path and permissions.");
            is_ready_ = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "SRDF content loaded successfully (Size: %zu bytes).", srdf_content_.size());
        is_ready_ = true;

        // 3. Setup Publisher and Timer
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            TOPIC_NAME, 
            rclcpp::QoS(1).transient_local().reliable() // Use reliable QoS for guaranteed delivery
        );
        
        // Publish every 1 second for a limited number of times
        timer_ = this->create_wall_timer(
            1s, 
            std::bind(&SemanticPublisher::publish_semantic_description, this)
        );
    }

private:
    std::string srdf_content_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int publish_count_;
    bool is_ready_ = false;

    // Helper function to read the entire file into a string
    std::string load_file(const std::string& path)
    {
        std::ifstream file(path);
        if (!file.is_open())
        {
            return "";
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    // Timer callback function to publish the SRDF content
    void publish_semantic_description()
    {
        if (!is_ready_ || publish_count_ >= MAX_PUBLISH_COUNT)
        {
            if (publish_count_ >= MAX_PUBLISH_COUNT)
            {
                timer_->cancel();
                RCLCPP_INFO(this->get_logger(), "Finished publishing SRDF string periodically.");
            }
            return;
        }

        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = srdf_content_;
        
        publisher_->publish(std::move(msg));
        RCLCPP_INFO(this->get_logger(), "Publishing SRDF (Attempt %d/%d)", publish_count_ + 1, MAX_PUBLISH_COUNT);
        publish_count_++;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Use a single-threaded executor
    rclcpp::spin(std::make_shared<SemanticPublisher>());
    
    rclcpp::shutdown();
    return 0;
}
