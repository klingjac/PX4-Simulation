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
