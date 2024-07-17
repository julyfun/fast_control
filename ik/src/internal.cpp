#include <atomic>
#include <cstdint>
#include <moveit/collision_detection/collision_common.h>
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

namespace ik::internal {

using Points = trajectory_msgs::msg::JointTrajectory::_points_type;
using geometry_msgs::msg::Pose;
using ik::PositionFilter;
using ik::PositionPredictorInterface;
using ::moveit::planning_interface::MoveGroupInterface;

const double JOINTS_ANGLE_MAX_VEL[6] = {
    2.596177, 2.596177, 2.596177, 3.110177, 3.110177, 3.110177
};

class Ik: public rclcpp::Node {
private:
    // [tf]
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    // [timer]
    rclcpp::TimerBase::SharedPtr timer_moveit_plan;
    rclcpp::TimerBase::SharedPtr timer_aubo_query;
    // [moveit]
    std::shared_ptr<rclcpp::Node> node_for_movegroup;
    MoveGroupInterface move_group_interface;
    // [aubo]
    ServiceInterface robot_service;
    double fps_aubo_consume;
    int size_expected_mac_points;
    // [latency]
    double latency_hand_to_before_plan;
    double latency_after_plan_others;
    // [plan state]
    std::array<double, 6> last_sent_aubo_point;
    uint64_t last_sent_aubo_point_id;
    std::mutex last_sent_aubo_point_mutex;

    std::deque<std::array<double, 6>> moveit_point_queue;
    uint64_t moveit_points_parent_id;
    std::mutex moveit_points_mutex;
    // [hand filter]
    PositionFilter<2, 1> pos_filter;
    double q_x;
    double q_v;
    double r_x;
    // [vm]
    bool vm;

public:
    explicit Ik(const rclcpp::NodeOptions& options):
        Node(
            "ik_moveit",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        ),
        tf_buffer(this->get_clock()),
        tf_listener(this->tf_buffer),
        // [moveit]
        node_for_movegroup(std::make_shared<rclcpp::Node>("node_for_movegroup")),
        move_group_interface(node_for_movegroup, "classic_six")
    //
    {
        RCLCPP_INFO(this->get_logger(), "Moveit ik started.");
        // [timer]
        {
            // const double fps_moveit_plan = this->get_parameter("fps_moveit_plan").as_double();
            const double fps_moveit_plan = 2.0;
            RCLCPP_INFO(this->get_logger(), "fps_moveit_plan: %f", fps_moveit_plan);
            this->timer_moveit_plan = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / fps_moveit_plan),
                [this]() { this->callback_moveit_plan(); }
            );
            const double fps_aubo_query = this->get_parameter("fps_aubo_query").as_double();
            // const double fps_aubo_query = 1.0;
            this->timer_aubo_query = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / fps_aubo_query),
                [this]() { this->callback_aubo_query(); }
            );
        }
        // [node for movegroup]
        std::thread([&]() { rclcpp::spin(node_for_movegroup); }).detach();

        // [constants]
        this->fps_aubo_consume = this->get_parameter("fps_aubo_consume").as_double();
        this->latency_hand_to_before_plan =
            this->get_parameter("latency_hand_to_before_plan").as_double();
        this->latency_after_plan_others =
            this->get_parameter("latency_after_plan_others").as_double();
        this->size_expected_mac_points = this->get_parameter("size_expected_mac_points").as_int();
        this->vm = this->get_parameter("vm").as_bool();

        // [robot init]
        {
            const char* host = "192.168.38.128";
            const int port = 8899;
            const int ret = this->robot_service.robotServiceLogin(host, port, "aubo", "123456");
            if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
                RCLCPP_INFO(this->get_logger(), "login success.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "login failed");
                return;
            }
            this->robot_service.robotServiceEnterTcp2CanbusMode();
            this->robot_service.robotServiceInitGlobalMoveProfile();
        }
        // [aubo state]
        {
            // only needed in init time (later we'll keep updateing it)
            aubo_robot_namespace::JointStatus joint_status[6];
            const int ret = this->robot_service.robotServiceGetRobotJointStatus(joint_status, 6);
            if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
                for (size_t i = 0; i < 6; ++i) {
                    this->last_sent_aubo_point[i] = joint_status[i].jointPosJ; // ok this
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "get joint status failed");
                return;
            }
        }

        // [moveit 添加桌面]
        {
            auto const collision_object = [frame_id =
                                               this->move_group_interface.getPlanningFrame()] {
                moveit_msgs::msg::CollisionObject collision_object;
                collision_object.header.frame_id = frame_id;
                collision_object.id = "desk";
                shape_msgs::msg::SolidPrimitive primitive;

                // Define the size of the box in meters
                primitive.type = primitive.BOX;
                primitive.dimensions.resize(3);
                primitive.dimensions[primitive.BOX_X] = 2.0;
                primitive.dimensions[primitive.BOX_Y] = 2.0;
                primitive.dimensions[primitive.BOX_Z] = 0.2;

                // Define the pose of the box (relative to the frame_id)
                geometry_msgs::msg::Pose box_pose;
                box_pose.orientation.w =
                    1.0; // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
                box_pose.position.x = 0.0;
                box_pose.position.y = 0.0;
                box_pose.position.z = -0.1;

                collision_object.primitives.push_back(primitive);
                collision_object.primitive_poses.push_back(box_pose);
                collision_object.operation = collision_object.ADD;

                return collision_object;
            }();
            ::moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            planning_scene_interface.applyCollisionObject(collision_object);
        }

        { // kalman init
            // const auto [pos, ori] = this->get_pose();
            // const auto ts_estimated_hand =
            //     this->now().seconds() - this->latency_hand_to_before_plan;
            // this->pos_filter.set_state(
            //     { std::make_tuple(Eigen::Matrix<double, 2, 1> { pos.x(), 0.0 }, ts_estimated_hand),
            //       std::make_tuple(Eigen::Matrix<double, 2, 1> { pos.y(), 0.0 }, ts_estimated_hand),
            //       std::make_tuple(Eigen::Matrix<double, 2, 1> { pos.z(), 0.0 }, ts_estimated_hand) }
            // );

            this->q_x = this->get_parameter("pos_filter_q_x").as_double();
            this->q_v = this->get_parameter("pos_filter_q_v").as_double();
            this->r_x = this->get_parameter("pos_filter_r_x").as_double();
        }
    }

private:
    void callback_moveit_plan() {
        RCLCPP_INFO(this->get_logger(), "callback_moveit_plan");
        // count time
        const auto t_start = std::chrono::high_resolution_clock::now();
        auto print_t = [&t_start](const std::string& name) {
            const auto t_end = std::chrono::high_resolution_clock::now();
            const auto duration =
                std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
            RCLCPP_INFO(
                rclcpp::get_logger("rclcpp"),
                "Time taken by %s: %ld mis",
                name.c_str(),
                duration.count()
            );
        };

        const auto [pos, ori] = this->get_pose();
        const double ts_before_plan = this->now().seconds();
        const auto ts_estimated_hand = ts_before_plan - this->latency_hand_to_before_plan;
        this->pos_filter.update(
            { pos.x(), pos.y(), pos.z() },
            ts_estimated_hand,
            { this->q_x, this->q_v },
            { this->r_x }
        );

        print_t("#1");

        // get mac size
        const auto mac_size = this->get_aubo_mac_size();
        const auto points_size = mac_size / 6 + (mac_size % 6 > 0 ? 1 : 0);
        const double ts_estimated_effector_arraival = ts_before_plan
            + double(points_size) / this->fps_aubo_consume + this->latency_after_plan_others;
        const auto pos_estimated_effector =
            this->pos_filter.predict_pos(ts_estimated_effector_arraival);

        print_t("#2");

        struct Out {
            double base;
            double min;
            double max;
        };
        const auto out_pos = [](const Eigen::Vector3<double>& position,
                                const Out& outx,
                                const Out& outy,
                                const Out& outz,
                                const double ratio) {
            return tf2::Vector3(
                std::clamp(outx.base + position.x() * ratio, outx.min, outx.max),
                std::clamp(outy.base + position.y() * ratio, outy.min, outy.max),
                std::clamp(outz.base + position.z() * ratio, outz.min, outz.max)
            );
        };

        const auto out_esti_pos = out_pos(
            pos_estimated_effector,
            { 0.4, 0.4, 1.8 },
            { 0, -1.8, 1.8 },
            { 0.1, 0.05, 1.8 },
            1.0
        );

        {
            for (size_t i = 0; i < 3; ++i) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "pos[%d]: %f -> %f",
                    int(i),
                    pos[i],
                    pos_estimated_effector[i]
                );
            }
        }

        print_t("#3");

        const auto [this_plan_parent_id, from] = [&]() {
            std::lock_guard<std::mutex> lock(this->last_sent_aubo_point_mutex);
            const auto parent_id = this->last_sent_aubo_point_id;
            const auto from = this->last_sent_aubo_point;
            return std::make_tuple(parent_id, from);
        }();
        const auto ori_here = ori;
        const auto to = [&] {
            geometry_msgs::msg::Pose msg;
            msg.position.x = out_esti_pos[0];
            msg.position.y = out_esti_pos[1];
            msg.position.z = out_esti_pos[2];
            msg.orientation = tf2::toMsg(ori_here);
            return msg;
        }();
        // show from
        RCLCPP_INFO(
            this->get_logger(),
            "from: %f %f %f %f %f %f",
            from[0],
            from[1],
            from[2],
            from[3],
            from[4],
            from[5]
        );
        // show to
        RCLCPP_INFO(
            this->get_logger(),
            "to: %f %f %f %f %f %f %f",
            to.position.x,
            to.position.y,
            to.position.z,
            to.orientation.w,
            to.orientation.x,
            to.orientation.y,
            to.orientation.z
        );

        print_t("#4");

        const auto trajectory = this->get_moveit_trajectory(from, to);

        print_t("#5");

        if (trajectory.has_value()) {
            std::lock_guard<std::mutex> lock(this->moveit_points_mutex);
            if (this->moveit_points_parent_id != this_plan_parent_id) {
                while (!this->moveit_point_queue.empty()) {
                    this->moveit_point_queue.pop_front();
                }
            }
            const auto& points = trajectory.value();
            for (size_t i = 0; i < points.size(); ++i) {
                const std::array<double, 6> point {
                    points[i].positions[0], points[i].positions[1], points[i].positions[2],
                    points[i].positions[3], points[i].positions[4], points[i].positions[5]
                };
                // print angles
                // RCLCPP_INFO(
                //     this->get_logger(),
                //     "Point %zu: %f %f %f %f %f %f",
                //     i,
                //     point[0],
                //     point[1],
                //     point[2],
                //     point[3],
                //     point[4],
                //     point[5]
                // );
                this->moveit_point_queue.push_back(point);
            }
            for (size_t i = 0; i < 6; ++i) {
                this->last_sent_aubo_point[i] = points.back().positions[i];
            }
            // this->moveit_points_queue.push(poin);
            this->moveit_points_parent_id = this->last_sent_aubo_point_id;
            RCLCPP_INFO(this->get_logger(), "plan %d points", int(points.size()));
        }

        print_t("#6");
    }

    void callback_aubo_query() {
        RCLCPP_INFO(this->get_logger(), "########### callback_aubo_query");
        const auto mac_size = this->get_aubo_mac_size();
        const auto points_size = mac_size / 6 + (mac_size % 6 > 0 ? 1 : 0);
        auto arr_to_waypoint = [](const std::array<double, 6>& arr) {
            aubo_robot_namespace::wayPoint_S waypoint;
            std::memcpy(waypoint.jointpos, arr.data(), 6 * sizeof(double));
            return waypoint;
        };
        auto waypoint_to_arr = [](const aubo_robot_namespace::wayPoint_S& waypoint) {
            std::array<double, 6> arr;
            std::memcpy(arr.data(), waypoint.jointpos, 6 * sizeof(double));
            return arr;
        };
        const auto waypoint_vector = [&]() {
            std::lock_guard<std::mutex> lock_plan(this->moveit_points_mutex);
            if (this->moveit_points_parent_id != this->last_sent_aubo_point_id) {
                while (!this->moveit_point_queue.empty()) {
                    this->moveit_point_queue.pop_front();
                } // 必须防止饥饿，就算数据没用都要送过去一堆老的
            }
            // [todo] check if "lock_guard not at the beginning of the block" is ok.
            // 必须保证 moveit_points_queue 始终有效
            std::vector<aubo_robot_namespace::wayPoint_S> waypoint_vector;
            std::lock_guard<std::mutex> lock_sent(this->last_sent_aubo_point_mutex);
            auto pre_point = this->last_sent_aubo_point;
            int points_needed = this->size_expected_mac_points - points_size;
            while (points_needed > 0) {
                if (this->moveit_point_queue.empty()) {
                    waypoint_vector.push_back(arr_to_waypoint(pre_point));
                    points_needed--;
                    continue;
                }
                const auto nxt_point = this->moveit_point_queue.front();
                const int interpolation_num = [&]() {
                    int itpltn = 1;
                    for (size_t i = 0; i < 6; ++i) {
                        // 假设电机就会这样执行
                        const double vel =
                            std::abs(nxt_point[i] - pre_point[i]) / (1.0 / this->fps_aubo_consume);
                        itpltn = std::max(itpltn, int(ceil(vel / JOINTS_ANGLE_MAX_VEL[i])));
                    }
                    return itpltn;
                }();
                for (int i = 0; i < interpolation_num && points_needed > 0; i++) {
                    std::array<double, 6> point;
                    for (size_t j = 0; j < 6; ++j) {
                        // don't push pre_joint
                        point[j] = pre_point[j]
                            + (nxt_point[j] - pre_point[j]) * (i + 1) / interpolation_num;
                    }
                    waypoint_vector.push_back(arr_to_waypoint(point));
                    if (i == interpolation_num - 1) {
                        pre_point = nxt_point;
                        this->moveit_point_queue.pop_front();
                    }
                    points_needed--;
                }
            }
            return waypoint_vector;
        }();
        RCLCPP_INFO(
            this->get_logger(),
            "points_size: %d, points_to_send: %d",
            int(points_size),
            int(waypoint_vector.size())
        );
        if (waypoint_vector.empty()) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock_sent(this->last_sent_aubo_point_mutex);
            this->robot_service.robotServiceSetRobotPosData2Canbus(waypoint_vector);
            this->last_sent_aubo_point = waypoint_to_arr(waypoint_vector.back());
            this->last_sent_aubo_point_id++;
            // show size
            RCLCPP_INFO(
                this->get_logger(),
                "sent %d points, %d points in plan queue",
                int(waypoint_vector.size()),
                int(this->moveit_point_queue.size())
            );
        }
    }

    uint16_t get_aubo_mac_size() {
        // count time here
        using namespace std::chrono;

        if (this->vm) {
            return uint16_t(0);
        }

        auto start = high_resolution_clock::now();
        aubo_robot_namespace::RobotDiagnosis robot_diagnosis_info;
        if (this->robot_service.robotServiceGetRobotDiagnosisInfo(robot_diagnosis_info)
            != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to get RobotDiagnosisInfo");
            return uint16_t(0);
        };
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        RCLCPP_INFO(
            this->get_logger(),
            "Time taken by function: %ld microseconds",
            duration.count()
        );
        return robot_diagnosis_info.macTargetPosDataSize;
    }

    tf2::Transform get_transform(const std::string& target_frame, const std::string& source_frame) {
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

        tf2::Transform transform;
        tf2::fromMsg(transform_stamped.transform, transform);
        return transform;
    }

    std::tuple<tf2::Vector3, tf2::Quaternion> get_pose() {
        const auto transform = this->get_transform("custom", "tracker_upright");
        return std::make_tuple(transform.getOrigin(), transform.getRotation());
    }

    bool collide(const std::array<double, 6>& point) {
        using namespace collision_detection;
        auto current_state = this->move_group_interface.getCurrentState();
        ::moveit::core::RobotState assume_state(*current_state); // copy the state
        std::vector<double> joint_group_positions;
        assume_state.setJointGroupPositions(
            current_state->getJointModelGroup("classic_six"),
            joint_group_positions
        );
        CollisionRequest collision_request;
        CollisionResult collision_result;
        // collision_request.contacts = true;
        // collision_request.max_contacts = 1000;

        return false;
    }

    // suppose urdf is loaded and links are specified
    std::optional<Points> get_moveit_trajectory(const std::array<double, 6>& from, const Pose& to) {
        RCLCPP_INFO(this->get_logger(), "set start state");
        const auto start_t = std::chrono::high_resolution_clock::now();
        auto print_t = [&start_t](const std::string& name) {
            const auto end_t = std::chrono::high_resolution_clock::now();
            const auto duration =
                std::chrono::duration_cast<std::chrono::microseconds>(end_t - start_t);
            RCLCPP_INFO(
                rclcpp::get_logger("rclcpp"),
                "Time taken by %s: %ld mis",
                name.c_str(),
                duration.count()
            );
        };
        { // [set start state]

            ::moveit::core::RobotStatePtr current_state =
                this->move_group_interface.getCurrentState();

            print_t("##1");

            ::moveit::core::RobotState start_state(*current_state);
            std::vector<double> joint_group_positions;
            start_state.copyJointGroupPositions(
                current_state->getJointModelGroup("classic_six"),
                joint_group_positions
            );
            print_t("##2");
            for (size_t i = 0; i < 6; ++i) {
                joint_group_positions[i] = from[i];
            }
            start_state.setJointGroupPositions(
                current_state->getJointModelGroup("classic_six"),
                joint_group_positions
            );
            print_t("##3");
            this->move_group_interface.setStartState(start_state);
            print_t("##4");
        }
        // [Set a target Pose]
        this->move_group_interface.setPoseTarget(to);
        print_t("##5");
        auto const [success, plan] = [this] {
            ::moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(this->move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();
        print_t("##6");
        if (success) {
            return plan.trajectory_.joint_trajectory.points;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            return std::nullopt;
        }
    }
};

//

} // namespace ik::internal

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ik::internal::Ik)
