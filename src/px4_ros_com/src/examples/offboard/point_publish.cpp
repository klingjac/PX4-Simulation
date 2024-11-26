#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class CsvPointPublisher : public rclcpp::Node {
public:
    CsvPointPublisher(const std::string &csv_file_path)
        : Node("point_publish_csv"), csv_file_path_(csv_file_path) {
        setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/input/setpoints", 10);
        done_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/input/done", 10);

        loadCsvAndPublishPoints();
    }

private:
    std::string csv_file_path_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_publisher_;

    void loadCsvAndPublishPoints() {
        std::ifstream file(csv_file_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the CSV file: %s", csv_file_path_.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream stream(line);
            float x, y, z, yaw;
            char comma;
            if (stream >> x >> comma >> y >> comma >> z >> comma >> yaw) {
                // Publish the trajectory setpoint
                px4_msgs::msg::TrajectorySetpoint setpoint_msg;
                setpoint_msg.position[0] = x;
                setpoint_msg.position[1] = y;
                setpoint_msg.position[2] = z;
                setpoint_msg.yaw = yaw;
                setpoint_publisher_->publish(setpoint_msg);

                // Log the published setpoint
                RCLCPP_INFO(this->get_logger(), "Published setpoint: x=%f, y=%f, z=%f, yaw=%f", x, y, z, yaw);

                // Wait to simulate sequential publishing
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid line in CSV: %s", line.c_str());
            }
        }

        file.close();

        // Send the "done" signal
        std_msgs::msg::Bool done_msg;
        done_msg.data = true;
        done_publisher_->publish(done_msg);

        RCLCPP_INFO(this->get_logger(), "All setpoints published, 'done' message sent.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run px4_ros_com point_publish_csv <csv_file_path>");
        return 1;
    }

    std::string csv_file_path = argv[1];
    auto node = std::make_shared<CsvPointPublisher>(csv_file_path);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
