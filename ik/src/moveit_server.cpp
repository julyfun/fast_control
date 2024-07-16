#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>

using moveit::planning_interface::MoveGroupInterface;

class Test: public rclcpp::Node {
public:
    explicit Test(const rclcpp::NodeOptions& options):
        Node(
            "ik_hello",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        ) {
        this->that();
    }

private:
    void that() {
        auto const node = std::make_shared<rclcpp::Node>("hello_moveit");

        // rclcpp::executors::SingleThreadedExecutor executor;
        // executor.add_node(node);
        // auto spinner = std::thread([&executor]() { executor.spin(); });

        auto thread = std::thread([&]() { rclcpp::spin(node); });
        // spinner.join();
        auto move_group_interface = MoveGroupInterface(node, "classic_six");
        {
            moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
        }
        RCLCPP_INFO(node->get_logger(), "Everything done!");
        thread.join();
    }
};

namespace ik::hello {

using moveit::planning_interface::MoveGroupInterface;

class Ik: public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_moveit_plan;
    std::optional<MoveGroupInterface> move_group_interface;

public:
    explicit Ik(const rclcpp::NodeOptions& options):
        Node(
            "ik_test",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        ) {
        {
            this->timer_moveit_plan = this->create_wall_timer(std::chrono::seconds(1), [this] {
                this->callback_moveit_plan();
            });
            RCLCPP_INFO(this->get_logger(), "#1");
            auto const node = std::make_shared<rclcpp::Node>("hello_moveit");

            // rclcpp::executors::SingleThreadedExecutor executor;
            // executor.add_node(node);
            // auto spinner = std::thread([&executor]() { executor.spin(); });

            RCLCPP_INFO(this->get_logger(), "#2");
            auto thread = std::thread([&]() { rclcpp::spin(node); });
            // spinner.join();

            RCLCPP_INFO(this->get_logger(), "#3");
            this->move_group_interface = MoveGroupInterface(node, "classic_six");
            {
                RCLCPP_INFO(this->get_logger(), "#4");
            }
            thread.detach();
        }
    }

private:
    void callback_moveit_plan() {
        // moveit::core::RobotStatePtr current_state = this->move_group_interface->getCurrentState();
        moveit::core::RobotStatePtr current_state = this->move_group_interface->getCurrentState();
        RCLCPP_INFO(this->get_logger(), "Current state got");
    }
};
} // namespace ik::hello

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ik::hello::Ik>(
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::shutdown();
    return 0;
}
