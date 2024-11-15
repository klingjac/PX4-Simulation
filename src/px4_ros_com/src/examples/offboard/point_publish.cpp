#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <std_msgs/msg/bool.hpp>
#include <iostream>
#include <string>
#include <vector>

class SetpointInputNode : public rclcpp::Node
{
public:
    SetpointInputNode() : Node("setpoint_input_node")
    {
        setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/input/setpoints", 10);
        done_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/input/done", 10);
        

        take_input();
    }

    void take_input()
    {
        std::string input;

        
        while (true)
        {
            std::cout << "Enter setpoint (format: x y z yaw) or 'done': ";
            std::getline(std::cin, input);

            if (input == "done")
            {
                std_msgs::msg::Bool done_msg;
                done_msg.data = true;
                done_publisher_->publish(done_msg);
                break;
            }
            else
            {
                px4_msgs::msg::TrajectorySetpoint setpoint = parse_input(input);
                setpoint_publisher_->publish(setpoint);
            }
        }
    }

    px4_msgs::msg::TrajectorySetpoint parse_input(const std::string &input)
    {
        std::istringstream iss(input);
        float x, y, z, yaw;
        iss >> x >> y >> z >> yaw;

        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.position = {x, y, z};
        setpoint.yaw = yaw;
        // You can also set other parameters like yaw here
        return setpoint;
    }

private:
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointInputNode>());
    rclcpp::shutdown();
    return 0;
}
