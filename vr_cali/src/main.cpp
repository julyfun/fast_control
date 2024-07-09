// #include "tf_test/tf_test.hpp"

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// #include <serviceinterface.h> // local
// #include <Poco/Net/ServerSocket.h> // system

namespace vr_cali {

tf2::Vector3 axis_under(const tf2::Vector3& a, const tf2::Vector3& b, const tf2::Transform& trans) {
    auto new_a = trans * a;
    auto new_b = trans * b;
    return (new_b - new_a).normalized();
}

void vec_info(const tf2::Vector3& vec) {
    RCLCPP_INFO(rclcpp::get_logger("vr_cali"), "x: %f, y: %f, z: %f", vec.x(), vec.y(), vec.z());
}

class VrCali: public rclcpp::Node {
private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::StaticTransformBroadcaster
        tf_ref_custom; // custom representation => ref representation
    tf2_ros::StaticTransformBroadcaster tf_random_upright;

    std::optional<tf2::Vector3> p0;
    std::optional<tf2::Vector3> px;
    std::optional<tf2::Vector3> py;
    // need a quaternion

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr p0_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr px_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr py_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cali_fixed_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr tracker_upright_sub;
    // tf buffer

public:
    explicit VrCali(const rclcpp::NodeOptions& options):
        Node("vr_cali", options),
        tf_buffer(this->get_clock()),
        tf_listener(this->tf_buffer),
        tf_ref_custom(this),
        tf_random_upright(this) {
        RCLCPP_INFO(this->get_logger(), "Node has been started.");

        this->p0_sub = this->create_subscription<std_msgs::msg::Empty>(
            "/cali/p0",
            10,
            [this](std_msgs::msg::Empty::SharedPtr msg) { this->record_p0(msg); }
        );
        this->px_sub = this->create_subscription<std_msgs::msg::Empty>(
            "/cali/px",
            10,
            [this](std_msgs::msg::Empty::SharedPtr msg) { this->record_px(msg); }
        );
        this->py_sub = this->create_subscription<std_msgs::msg::Empty>(
            "/cali/py",
            10,
            [this](std_msgs::msg::Empty::SharedPtr msg) { this->record_py(msg); }
        );
        this->cali_fixed_sub = this->create_subscription<std_msgs::msg::Empty>(
            "/cali/cali_fixed",
            10,
            [this](std_msgs::msg::Empty::SharedPtr msg) { this->cali_fixed(msg); }
        );
        this->tracker_upright_sub = this->create_subscription<std_msgs::msg::Empty>(
            "/cali/upright",
            10,
            [this](std_msgs::msg::Empty::SharedPtr msg) { this->tracker_upright(msg); }
        );
    };

    ~VrCali() override = default;

private:
    void tracker_upright(const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Try to calibrate upright.");
        auto rotation_matrix = this->get_rotation("tracker_random", "custom");
        const auto translation_vector = tf2::Vector3(0.0, 0, 0);
        const auto transform = tf2::Transform(rotation_matrix, translation_vector);
        tf2::Quaternion quaternion = transform.getRotation();
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "tracker_random"; // target
        transform_stamped.child_frame_id = "tracker_upright"; // source
        transform_stamped.transform.translation = tf2::toMsg(transform.getOrigin());
        transform_stamped.transform.rotation = tf2::toMsg(quaternion);
        this->tf_random_upright.sendTransform(transform_stamped); // 可能重新标定, so not static
    }

    tf2::Matrix3x3 get_rotation(const std::string& target_frame, const std::string& source_frame) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped =
                this->tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
            throw std::runtime_error("Transform lookup failed");
        }

        tf2::Quaternion quaternion;
        tf2::fromMsg(transform_stamped.transform.rotation, quaternion);
        return tf2::Matrix3x3(quaternion);
    }

    tf2::Vector3 get_position(const std::string& target_frame, const std::string& source_frame) {
        // target_frame 是 frame, source_frame 是 child_frame
        // transform M should be the description of source_frame in target_frame
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped =
                this->tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
            throw std::runtime_error("Transform lookup failed");
        }

        tf2::Vector3 translation;
        tf2::fromMsg(transform_stamped.transform.translation, translation);
        return translation;
    }

    void record_p0(const std_msgs::msg::Empty::SharedPtr msg) {
        // 同一点的表述 b => 表述 a 的平移量就是 a 表述下 b 的原点位置
        this->p0 = this->get_position("ref", "tracker_random");
        RCLCPP_INFO(this->get_logger(), "p0 has been set to %f %f %f", p0->x(), p0->y(), p0->z());
    }
    // 避免数据竞争
    void record_px(const std_msgs::msg::Empty::SharedPtr msg) {
        this->px = this->get_position("ref", "tracker_random");
        RCLCPP_INFO(this->get_logger(), "px has been set to %f %f %f", px->x(), px->y(), px->z());
    }
    void record_py(const std_msgs::msg::Empty::SharedPtr msg) {
        this->py = this->get_position("ref", "tracker_random");
        RCLCPP_INFO(this->get_logger(), "py has been set to %f %f %f", py->x(), py->y(), py->z());
    }
    void cali_fixed(const std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Try to calibrate.");
        if (!this->p0.has_value()) {
            RCLCPP_ERROR(this->get_logger(), "p0 is not set, cannot calibrate");
            return;
        }
        if (!this->px.has_value()) {
            RCLCPP_ERROR(this->get_logger(), "px is not set, cannot calibrate");
            return;
        }
        if (!this->py.has_value()) {
            RCLCPP_ERROR(this->get_logger(), "py is not set, cannot calibrate");
            return;
        }

        const auto x_axis = (this->px.value() - this->p0.value()).normalized();
        auto y_axis = (this->py.value() - this->p0.value()).normalized();
        const auto z_axis = x_axis.cross(y_axis).normalized();
        y_axis = z_axis.cross(x_axis).normalized();
        const tf2::Matrix3x3 rotation_matrix(
            x_axis.x(),
            y_axis.x(),
            z_axis.x(),
            x_axis.y(),
            y_axis.y(),
            z_axis.y(),
            x_axis.z(),
            y_axis.z(),
            z_axis.z()
        );
        const auto translation_vector = this->p0.value();
        const auto transform = tf2::Transform(rotation_matrix, translation_vector);
        tf2::Quaternion quaternion = transform.getRotation();
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "ref"; // target
        transform_stamped.child_frame_id = "custom"; // source
        transform_stamped.transform.translation = tf2::toMsg(transform.getOrigin());
        transform_stamped.transform.rotation = tf2::toMsg(quaternion);
        this->tf_ref_custom.sendTransform(transform_stamped); // 可能重新标定, so not static

        vec_info(axis_under(p0.value(), px.value(), transform.inverse()));
        vec_info(axis_under(p0.value(), py.value(), transform.inverse()));
        vec_info(axis_under(p0.value(), p0.value() + z_axis, transform.inverse()));
    }
};

} // namespace vr_cali

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(vr_cali::VrCali)
