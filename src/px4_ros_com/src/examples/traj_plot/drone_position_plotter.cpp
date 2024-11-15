#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include <cmath> // For M_PI

class DronePositionPlotter : public rclcpp::Node {
public:
    DronePositionPlotter() : Node("drone_position_plotter") {
        position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(), std::bind(&DronePositionPlotter::position_callback, this, std::placeholders::_1));

        attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(), std::bind(&DronePositionPlotter::attitude_callback, this, std::placeholders::_1));

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("drone_path", 10);

        publish_static_transform();
    }

private:
    void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        current_position_ = *msg;

        geometry_msgs::msg::PoseStamped new_pose;
        new_pose.header.frame_id = "map";
        new_pose.header.stamp = this->get_clock()->now();
        new_pose.pose.position.x = msg->x;
        new_pose.pose.position.y = msg->y;
        new_pose.pose.position.z = -msg->z; // Invert Z-axis for ENU

        flight_path_.poses.push_back(new_pose);
        flight_path_.header.frame_id = "map";
        flight_path_.header.stamp = this->get_clock()->now();
        path_publisher_->publish(flight_path_);
    }

    void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        // Original NED quaternion
        // tf2::Quaternion ned_orientation;
        // ned_orientation.setValue(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

        // // Conversion quaternion (NED to ENU)
        // tf2::Quaternion conversion;
        // conversion.setRPY(M_PI / 2, 0, -M_PI / 2);

        // // Apply conversion
        // tf2::Quaternion enu_orientation = conversion * ned_orientation;
        // enu_orientation.normalize();
        tf2::Quaternion ned_orientation(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

        // Convert NED to ENU
        tf2::Quaternion enu_orientation;
        enu_orientation.setValue(-ned_orientation.z(), ned_orientation.y(), ned_orientation.x(), ned_orientation.w());

        enu_orientation.normalize();

        visualization_msgs::msg::Marker drone_marker;
        drone_marker.header.frame_id = "map";
        drone_marker.header.stamp = this->get_clock()->now();
        drone_marker.ns = "drone_model";
        drone_marker.id = 0;
        drone_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        drone_marker.action = visualization_msgs::msg::Marker::ADD;

        drone_marker.pose.position.x = current_position_.x;
        drone_marker.pose.position.y = current_position_.y;
        drone_marker.pose.position.z = -current_position_.z; // Invert Z-axis for ENU

        drone_marker.pose.orientation = tf2::toMsg(enu_orientation);

        drone_marker.scale.x = 0.001; // Adjust scale as needed
        drone_marker.scale.y = 0.001;
        drone_marker.scale.z = 0.001;
        drone_marker.color.a = 1.0; // Use an alpha value of 1 for full opacity

        drone_marker.mesh_resource = "package://px4_ros_com/examples/models/Drone.stl"; // Path to your 3D model

        marker_publisher_->publish(drone_marker);
    }

    void publish_static_transform() {
        static tf2_ros::StaticTransformBroadcaster static_broadcaster(this);
        geometry_msgs::msg::TransformStamped static_transform_stamped;

        static_transform_stamped.header.stamp = this->get_clock()->now();
        static_transform_stamped.header.frame_id = "map";
        static_transform_stamped.child_frame_id = "base_link"; // Replace with the drone's frame
        static_transform_stamped.transform.translation.x = 0.0;
        static_transform_stamped.transform.translation.y = 0.0;
        static_transform_stamped.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0); // No rotation
        static_transform_stamped.transform.rotation.x = q.x();
        static_transform_stamped.transform.rotation.y = q.y();
        static_transform_stamped.transform.rotation.z = q.z();
        static_transform_stamped.transform.rotation.w = q.w();

        static_broadcaster.sendTransform(static_transform_stamped);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    px4_msgs::msg::VehicleLocalPosition current_position_;
    nav_msgs::msg::Path flight_path_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DronePositionPlotter>());
    rclcpp::shutdown();
    return 0;
}
