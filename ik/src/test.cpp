#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <serviceinterface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp>

namespace ik::test {
using moveit::planning_interface::MoveGroupInterface;

class Ik: public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_moveit_plan;

    std::shared_ptr<rclcpp::Node> node_for_movegroup;
    MoveGroupInterface move_group_interface;

public:
    explicit Ik(const rclcpp::NodeOptions& options):
        Node(
            "ik_test",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        ),
        node_for_movegroup(std::make_shared<rclcpp::Node>("node_for_movegroup")),
        move_group_interface(node_for_movegroup, "classic_six") {
        {
            this->timer_moveit_plan = this->create_wall_timer(

                std::chrono::duration<double>(1.0 / 5.0),
                [this] { this->callback_moveit_plan(); }
            );
            std::thread([&]() { rclcpp::spin(node_for_movegroup); }).detach();
        }
    }

private:
    void callback_moveit_plan() {
        // moveit::core::RobotStatePtr current_state = this->move_group_interface->getCurrentState();
        moveit::core::RobotStatePtr current_state = this->move_group_interface.getCurrentState();
        RCLCPP_INFO(this->get_logger(), "Current state got");
    }
};
} // namespace ik::test
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ik::test::Ik)
