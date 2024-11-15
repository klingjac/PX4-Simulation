#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <cmath>
#include <signal.h>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

std::chrono::milliseconds sleep_duration(100); 

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        auto reentrant_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions subscription_options;
        subscription_options.callback_group = reentrant_callback_group;

        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        setpoint_subscriber_ = this->create_subscription<TrajectorySetpoint>(
            "/input/setpoints", 10,
            std::bind(&OffboardControl::setpoint_callback, this, std::placeholders::_1),
            subscription_options);

        done_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/input/done", 10,
            std::bind(&OffboardControl::done_callback, this, std::placeholders::_1),
            subscription_options);

        local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(), std::bind(&OffboardControl::local_position_callback, this, std::placeholders::_1),
            subscription_options);

        RCLCPP_INFO(this->get_logger(), "OffboardControl node has been started.");
    }

    ~OffboardControl()
    {
        // Perform any cleanup before the node is destroyed
        RCLCPP_INFO(this->get_logger(), "Shutting down OffboardControl node...");
    }

    void arm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

    void disarm()
    {
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
        RCLCPP_INFO(this->get_logger(), "Disarm command sent");
    }

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr done_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscriber_;

    std::vector<TrajectorySetpoint> setpoint_vector_;
    float x = 0;
    float y = 0;
    float z = 0;

    const float max_intermediate_distance_ = 0.5;
    const float position_delta_ = 0.3; // Delta value for position accuracy

    void setpoint_callback(const TrajectorySetpoint::SharedPtr msg)
    {
        setpoint_vector_.push_back(*msg);
        RCLCPP_INFO(this->get_logger(), "Received setpoint: x=%f, y=%f, z=%f", 
                msg->position[0], msg->position[1], msg->position[2]);
    }

    std::vector<TrajectorySetpoint> generate_intermediate_waypoints(const TrajectorySetpoint& start, const TrajectorySetpoint& end) {
        std::vector<TrajectorySetpoint> waypoints;

        float dx = end.position[0] - start.position[0];
        float dy = end.position[1] - start.position[1];
        float dz = end.position[2] - start.position[2];
        float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        int num_intermediate_points = std::max(static_cast<int>(distance / max_intermediate_distance_), 1);

        for (int i = 1; i < num_intermediate_points; ++i) {
            float ratio = static_cast<float>(i) / num_intermediate_points;
            TrajectorySetpoint waypoint = start;
            waypoint.position[0] = start.position[0] + ratio * dx;
            waypoint.position[1] = start.position[1] + ratio * dy;
            waypoint.position[2] = start.position[2] + ratio * dz;
            waypoints.push_back(waypoint);
        }

        return waypoints;
    }

    void done_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();

            for (const auto& setpoint : setpoint_vector_) {
                RCLCPP_INFO(this->get_logger(), "List setpoints: x=%f, y=%f, z=%f", 
                setpoint.position[0], setpoint.position[1], setpoint.position[2]);
            }
            TrajectorySetpoint msg{};
            for (int i = 0; i < 80; i++){

                publish_offboard_control_mode();

                msg.yaw = -1.41;
                msg.position[0] = 0;
                msg.position[1] = 0;
                msg.position[2] = -3;
                msg.timestamp = this->get_clock()->now().nanoseconds() / 100;

                trajectory_setpoint_publisher_->publish(msg);
                rclcpp::sleep_for(sleep_duration);
            }

            for (const auto& setpoint : setpoint_vector_) {
                publish_offboard_control_mode();

                RCLCPP_INFO(this->get_logger(), "Going to setpoint: x=%f, y=%f, z=%f", 
                setpoint.position[0], setpoint.position[1], setpoint.position[2]);

                std::vector<TrajectorySetpoint> temp = generate_intermediate_waypoints(msg, setpoint);

                for(auto& temp_set : temp){
                    RCLCPP_INFO(this->get_logger(), "Going to setpoint: x=%f, y=%f, z=%f", 
                        temp_set.position[0], temp_set.position[1], temp_set.position[2]);
                    msg.position = temp_set.position;
                    msg.yaw = temp_set.yaw;
                    msg.timestamp = this->get_clock()->now().nanoseconds() / 100;
                    publish_offboard_control_mode();
                    trajectory_setpoint_publisher_->publish(msg);
                }

                while (!is_position_reached(setpoint)) {
                    publish_offboard_control_mode();

                    msg.position = setpoint.position;
                    msg.yaw = setpoint.yaw;
                    msg.timestamp = this->get_clock()->now().nanoseconds() / 100;

                    trajectory_setpoint_publisher_->publish(msg);
                    
                }
            }
            disarm(); // Or any other post-mission logic
        }
    }

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        x = msg->x;
        y = msg->y;
        z = msg->z;
    }

    bool is_position_reached(const TrajectorySetpoint& setpoint)
    {
        return std::abs(x - setpoint.position[0]) < position_delta_ &&
               std::abs(y - setpoint.position[1]) < position_delta_ &&
               std::abs(z - setpoint.position[2]) < position_delta_;
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OffboardControl>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  // This will automatically handle Ctrl-C to shutdown the node
    rclcpp::shutdown();
    return 0;
}
