#include <rclcpp/rclcpp.hpp>
#include "setpoint_input_node.hpp" // Adjust the include path as necessary

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_submitButton_clicked();
    void on_doneButton_clicked();

private:
    Ui::MainWindow *ui;
    std::shared_ptr<SetpointInputNode> setpointNode;
};


MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    rclcpp::init(0, nullptr); // Initialize ROS 2, adjust as necessary
    setpointNode = std::make_shared<SetpointInputNode>();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_submitButton_clicked()
{
    // Collect input from UI, convert to setpoint, and publish
    float x = ui->lineEditX->text().toFloat();
    float y = ui->lineEditY->text().toFloat();
    float z = ui->lineEditZ->text().toFloat();
    float yaw = ui->lineEditYaw->text().toFloat();

    px4_msgs::msg::TrajectorySetpoint setpoint;
    setpoint.position = {x, y, z};
    setpoint.yaw = yaw;

    setpointNode->publishSetpoint(setpoint); // Implement this method in your ROS 2 node class
}

void MainWindow::on_doneButton_clicked()
{
    std_msgs::msg::Bool done_msg;
    done_msg.data = true;
    setpointNode->publishDone(done_msg); // Implement this method in your ROS 2 node class
}
