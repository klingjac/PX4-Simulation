#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <std_msgs/msg/bool.hpp>
#include <iostream>
#include <string>
#include <sstream>
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

        std::cout << "Enter setpoints as a list (format: x1 y1 z1 yaw1, x2 y2 z2 yaw2, ...): ";
        std::getline(std::cin, input);

        if (input == "done")
        {
            std_msgs::msg::Bool done_msg;
            done_msg.data = true;
            done_publisher_->publish(done_msg);
            return;
        }

        std::vector<px4_msgs::msg::TrajectorySetpoint> setpoints = parse_input(input);
        for (const auto &setpoint : setpoints)
        {
            setpoint_publisher_->publish(setpoint);
        }

        // Publish "done" message after parsing and publishing all setpoints
        std_msgs::msg::Bool done_msg;
        done_msg.data = true;
        done_publisher_->publish(done_msg);
    }

    std::vector<px4_msgs::msg::TrajectorySetpoint> parse_input(const std::string &input)
    {
        std::vector<px4_msgs::msg::TrajectorySetpoint> setpoints;
        std::istringstream iss(input);
        std::string point_str;

        while (std::getline(iss, point_str, ','))
        {
            std::istringstream point_stream(point_str);
            float x, y, z, yaw;
            point_stream >> x >> y >> z >> yaw;

            px4_msgs::msg::TrajectorySetpoint setpoint;
            setpoint.position = {x, y, z};
            setpoint.yaw = yaw;
            setpoints.push_back(setpoint);
        }

        return setpoints;
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