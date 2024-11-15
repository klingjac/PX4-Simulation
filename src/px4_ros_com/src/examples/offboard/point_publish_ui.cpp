#include <QApplication>
#include <QWidget>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <iostream>
#include <vector>

class RosQtWidget : public QWidget {
public:
    RosQtWidget(rclcpp::Node::SharedPtr node) : node_(node) {
        this->setupUi();
        this->connectSignals();
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_publisher_;

    QLineEdit *xLineEdit, *yLineEdit, *zLineEdit, *yawLineEdit;
    QPushButton *publishButton, *doneButton, *previewButton;

    std::vector<geometry_msgs::msg::Point> points_;

    void setupUi() {
        auto *layout = new QVBoxLayout(this);

        xLineEdit = new QLineEdit(this);
        xLineEdit->setPlaceholderText("X");
        layout->addWidget(xLineEdit);

        yLineEdit = new QLineEdit(this);
        yLineEdit->setPlaceholderText("Y");
        layout->addWidget(yLineEdit);

        zLineEdit = new QLineEdit(this);
        zLineEdit->setPlaceholderText("Z");
        layout->addWidget(zLineEdit);

        yawLineEdit = new QLineEdit(this);
        yawLineEdit->setPlaceholderText("Yaw");
        layout->addWidget(yawLineEdit);

        previewButton = new QPushButton("Preview", this);
        layout->addWidget(previewButton);

        publishButton = new QPushButton("Publish Position", this);
        layout->addWidget(publishButton);

        doneButton = new QPushButton("Execute Flight", this);
        layout->addWidget(doneButton);

        setpoint_publisher_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/input/setpoints", 10);
        done_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/input/done", 10);
        marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        line_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("visualization_marker_line", 10);

        this->setLayout(layout);

        
    }

    void connectSignals() {
        connect(previewButton, &QPushButton::clicked, this, &RosQtWidget::onPreviewClicked);
        connect(publishButton, &QPushButton::clicked, this, &RosQtWidget::onPublishClicked);
        connect(doneButton, &QPushButton::clicked, this, &RosQtWidget::onDoneClicked);
    }

    void onPublishClicked() {

        visualization_msgs::msg::Marker delete_preview_marker;
        delete_preview_marker.header.frame_id = "map";
        delete_preview_marker.header.stamp = node_->get_clock()->now();
        delete_preview_marker.ns = "preview_point";
        delete_preview_marker.id = 0;
        delete_preview_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_publisher_->publish(delete_preview_marker);

        px4_msgs::msg::TrajectorySetpoint setpoint_msg;
        setpoint_msg.position[0] = xLineEdit->text().toFloat();
        setpoint_msg.position[1] = yLineEdit->text().toFloat();
        setpoint_msg.position[2] = zLineEdit->text().toFloat();
        setpoint_msg.yaw = yawLineEdit->text().toFloat();
        setpoint_publisher_->publish(setpoint_msg);

        visualization_msgs::msg::Marker point_marker;
        point_marker.header.frame_id = "map";
        point_marker.header.stamp = node_->get_clock()->now();
        point_marker.ns = "setpoint_markers";
        point_marker.id = points_.size();
        point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker.action = visualization_msgs::msg::Marker::ADD;
        point_marker.pose.position.x = setpoint_msg.position[0];
        point_marker.pose.position.y = setpoint_msg.position[1];
        point_marker.pose.position.z = -1*setpoint_msg.position[2];
        point_marker.pose.orientation.w = 1.0;
        point_marker.scale.x = 0.2;
        point_marker.scale.y = 0.2;
        point_marker.scale.z = 0.2;
        point_marker.color.a = 1.0; // Don't forget to set the alpha!
        point_marker.color.r = 1.0;
        point_marker.color.g = 0.0;
        point_marker.color.b = 0.0;
        marker_publisher_->publish(point_marker);

        geometry_msgs::msg::Point p;
        p.x = setpoint_msg.position[0];
        p.y = setpoint_msg.position[1];
        p.z = -1*setpoint_msg.position[2];
        points_.push_back(p);

        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = node_->get_clock()->now();
        line_marker.ns = "setpoint_lines";
        line_marker.id = 1;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.05;
        line_marker.color.a = 1.0; // Don't forget to set the alpha!
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        line_marker.points = points_;
        line_publisher_->publish(line_marker);
    }

    void onDoneClicked() {
        std_msgs::msg::Bool msg;
        msg.data = true;
        done_publisher_->publish(msg);
    }

    void onPreviewClicked() {
        visualization_msgs::msg::Marker preview_marker;
        preview_marker.header.frame_id = "map";
        preview_marker.header.stamp = node_->get_clock()->now();
        preview_marker.ns = "preview_point";
        preview_marker.id = 0; // Unique ID for the preview marker
        preview_marker.type = visualization_msgs::msg::Marker::SPHERE;
        preview_marker.action = visualization_msgs::msg::Marker::ADD;
        preview_marker.pose.position.x = xLineEdit->text().toDouble();
        preview_marker.pose.position.y = yLineEdit->text().toDouble();
        preview_marker.pose.position.z = -1*zLineEdit->text().toDouble();
        preview_marker.pose.orientation.w = 1.0;
        preview_marker.scale.x = 0.2; // Size of the preview marker
        preview_marker.scale.y = 0.2;
        preview_marker.scale.z = 0.2;
        preview_marker.color.a = 1.0; // Make sure the marker is visible
        preview_marker.color.r = 0.0;
        preview_marker.color.g = 0.0;
        preview_marker.color.b = 1.0; // Blue color for preview

        marker_publisher_->publish(preview_marker);
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("qt_ros_node");

    QApplication app(argc, argv);
    RosQtWidget widget(node);
    widget.show();

    auto result = app.exec();

    rclcpp::shutdown();
    return result;
}