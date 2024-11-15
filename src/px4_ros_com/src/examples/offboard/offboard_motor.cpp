#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <limits>
#include <vector>
#include <iostream>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ActuatorControlNode : public rclcpp::Node {
public:
    ActuatorControlNode() 
        : Node("actuator_control_node"), is_publishing_(false), start(false), publish_index_(0), repetition_count_(0) {
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        
        motor_publisher_ = this->create_publisher<ActuatorMotors>("/fmu/in/actuator_motors", 10);
        motor_subscriber_ = this->create_subscription<px4_msgs::msg::ActuatorMotors>(
            "/fmu/out/actuator_motors", qos,
            [this](const px4_msgs::msg::ActuatorMotors::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (msg->control[2] > 0.01) {
                    if (!start) {
                        RCLCPP_INFO(this->get_logger(), "Starting.");
                    }
                    start = true;
                } else {
                    start = false;
                }
                if (!is_publishing_ && start) {
                    motor_messages_.push_back(*msg);
                    RCLCPP_INFO(this->get_logger(), "Actuator motor message received and stored.");
                }
            }
        );

        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        update_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "gazebo/update", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "Gazebo simulation update") {
                    this->publish_stored_actuator_controls();
                }
            }
        );
    }

    void start_publishing() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!is_publishing_ && !motor_messages_.empty()) {
            is_publishing_ = true;
            publish_index_ = 0;
            repetition_count_ = 0;
        }
    }

private:
    void publish_stored_actuator_controls() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!is_publishing_ || motor_messages_.empty() || publish_index_ >= motor_messages_.size()) {
            return;
        }

        if (publish_index_ == 0 && repetition_count_ == 0) {
            arm();
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // arm
        }

        const auto& msg = motor_messages_[publish_index_];

        // Publish the original message
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        publish_offboard_control_mode();
        auto msg_copy = msg;
        msg_copy.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        motor_publisher_->publish(msg_copy);
        RCLCPP_INFO(this->get_logger(), "Re-published stored actuator motor control");

        repetition_count_++;
        if (repetition_count_ == 1) {
            publish_index_++;
        }
        if (repetition_count_ == 3) {
            repetition_count_ = 0;
        }

        if (publish_index_ >= motor_messages_.size()) {
            auto& last_msg = motor_messages_.back();
            last_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            motor_publisher_->publish(last_msg);
            RCLCPP_INFO(this->get_logger(), "Re-published last stored actuator motor control");

            is_publishing_ = false;
        }
    }

    void arm() {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
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

    void publish_offboard_control_mode() {
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

    std::vector<ActuatorMotors> motor_messages_;
    rclcpp::Subscription<ActuatorMotors>::SharedPtr motor_subscriber_;
    rclcpp::Publisher<ActuatorMotors>::SharedPtr motor_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr update_subscriber_;
    bool is_publishing_;
    bool start;
    std::mutex mutex_;
    size_t publish_index_;
    int repetition_count_;
};

void terminal_input_listener(std::shared_ptr<ActuatorControlNode> node) {
    std::string input;
    while (std::cin >> input) {
        if (input == "start") {
            node->start_publishing();
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActuatorControlNode>();

    std::thread input_thread(terminal_input_listener, node);

    rclcpp::spin(node);
    rclcpp::shutdown();

    input_thread.join();
    return 0;
}
