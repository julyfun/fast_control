#include <atomic>
#include <cstdint>
#include <mutex>
#include <queue>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <serviceinterface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp>

#include "AuboRobotMetaType.h"
#include "ik/filter.hpp"

namespace ik::login_test {

class LoginTest: public rclcpp::Node {
private:
    const char* HOST = "192.168.38.128";
    const int PORT = 8899;

public:
private:
};

} // namespace ik::login_test

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ik::login_test::LoginTest)
