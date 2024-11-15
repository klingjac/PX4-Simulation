#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <chrono>
#include <limits>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ActuatorControlNode : public rclcpp::Node
{
public:
    ActuatorControlNode()
        : Node("actuator_control_node")
    {
        motor_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", 10);
        servo_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/in/actuator_servos", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        timer_ = this->create_wall_timer(
            200ms, std::bind(&ActuatorControlNode::publish_actuator_controls, this));

        
        arm();
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

private:
    void publish_actuator_controls()
    {
        // Publish motor controls
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publish_offboard_control_mode();
        auto motor_message = px4_msgs::msg::ActuatorMotors();
        motor_message.timestamp = this->get_clock()->now().nanoseconds() / 1000; // Convert to microseconds
        motor_message.timestamp_sample = motor_message.timestamp;
        motor_message.reversible_flags = 0; // Adjust according to your needs

        // Set motor control values
        for (int i = 0; i < 4; ++i)
        {
            motor_message.control[i] = 0.8; // Replace with actual control values
        }
        // Fill the remaining motor controls with NaN
        for (int i = 4; i < 12; ++i) // Assuming the array length is 12
        {
            motor_message.control[i] = std::numeric_limits<float>::quiet_NaN();
        }

        motor_publisher_->publish(motor_message);
        RCLCPP_INFO(this->get_logger(), "Published actuator motor controls");

        // // Publish servo controls
        // auto servo_message = px4_msgs::msg::ActuatorServos();
        // servo_message.timestamp = motor_message.timestamp; // Use the same timestamp for consistency

        // // Set servo control values
        // for (int i = 0; i < 8; ++i) // Assuming there are 8 servos
        // {
        //     servo_message.control[i] = std::numeric_limits<float>::quiet_NaN(); // Replace with actual control values
        // }

        // //servo_publisher_->publish(servo_message);
        // RCLCPP_INFO(this->get_logger(), "Published actuator servo controls");
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.thrust_and_torque = false;
        msg.direct_actuator = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motor_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr servo_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorControlNode>());
    rclcpp::shutdown();
    return 0;
}
