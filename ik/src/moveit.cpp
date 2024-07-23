#include <atomic>
#include <cstdint>
#include <mutex>
#include <queue>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/executor.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serviceinterface.h>
#include <std_msgs/msg/int16.hpp>
#include <sys/types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp>

#include "AuboRobotMetaType.h"
#include "ik/filter.hpp"

namespace ik::moveit {

using Points = trajectory_msgs::msg::JointTrajectory::_points_type;
using geometry_msgs::msg::Pose;
using ik::PositionFilter;
using ik::PositionPredictorInterface;
using ::moveit::planning_interface::MoveGroupInterface;

// const double JOINTS_ANGLE_MAX_VEL[6] = {
//     2.596177, 2.596177, 2.596177, 3.110177, 3.110177, 3.110177
// };
// / 15 / pi * 180
// const double JOINTS_ANGLE_MAX_VEL[6] = { 20.0, 20.0, 20.0, 22.0, 22.0, 22.0 };
// mul 2
const double JOINTS_ANGLE_MAX_VEL[6] = { 40.0, 40.0, 40.0, 44.0, 44.0, 44.0 };
// div 4
// const double JOINTS_ANGLE_MAX_VEL[6] = { 5.0, 5.0, 5.0, 5.5, 5.5, 5.5 };
const double POS_MIN_DIFF_TO_SEND = 0.005;
const double ANGLE_MIN_DIFF_TO_SEND = 1.0;

double qua_theta(const tf2::Quaternion& q1, const tf2::Quaternion& q2) {
    // ref: https://math.stackexchange.com/questions/90081/quaternion-distance
    const double dot = q1.dot(q2);
    return std::acos(2 * dot * dot - 1);
}

double reduced_angle(const double angle) {
    // between -pi and pi
    return angle - 2 * M_PI * std::floor((angle + M_PI) / (2 * M_PI));
}

double angle_interpolation(const double left, const double right, const int itpltn, const int n) {
    // left maybe 179, right maybe -179, there differenct is 2
    const double diff = reduced_angle(right - left);
    return left + diff * itpltn / n;
}

double closest_angle(const double from, const double target) {
    return target + reduced_angle(from - target);
}

const double ANGLE_LIMIT[6][2] = { { -360, 360 }, { -175, 175 }, { -175, 175 },
                                   { -175, 175 }, { -175, 175 }, { -360, 360 } };

const double deg2rad(const double deg) {
    return deg / 180.0 * M_PI;
}

const double rad2deg(const double rad) {
    return rad / M_PI * 180.0;
}

std::array<double, 6> limited_closest_joint_angles(
    const std::array<double, 6>& joint_angles,
    const std::array<double, 6>& target_angles
) {
    std::array<double, 6> result;
    for (size_t i = 0; i < 6; ++i) {
        result[i] = closest_angle(joint_angles[i], target_angles[i]);
        result[i] = std::clamp(result[i], deg2rad(ANGLE_LIMIT[i][0]), deg2rad(ANGLE_LIMIT[i][1]));
    }
    return result;
}

class Ik: public rclcpp::Node {
private:
    // [tf]
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    // [timer]
    rclcpp::TimerBase::SharedPtr timer_moveit_plan;
    rclcpp::TimerBase::SharedPtr timer_aubo_query;
    // rclcpp::TimerBase::SharedPtr timer_mac_size_update;
    // [moveit]
    std::shared_ptr<rclcpp::Node> node_for_move_group;
    MoveGroupInterface move_group_interface;
    // [planning scene]
    // const ::moveit::core::RobotModelPtr robot_model;
    const ::moveit::core::JointModelGroup* joint_model_group;
    // [load]
    // planning_scene::PlanningScene virtual_planning_scene;
    // [t.service]
    std::shared_ptr<rclcpp::Node> node_for_srv;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr ps_srv;
    // [t.monitor]
    std::shared_ptr<rclcpp::Node> node_for_monitor;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm;
    // [aubo]
    ServiceInterface robot_service;
    double fps_aubo_consume;
    int size_expected_mac_points;
    // [aubo.libtopic]
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mac_size_sub;
    uint16_t sub_mac_size = 0; // most recent macsize. Must initialize
    std::mutex sub_mac_size_mutex;
    // [latency]
    double latency_hand_to_before_plan;
    double latency_after_plan_others;
    // [plan state]
    std::array<double, 6> last_sent_aubo_point;
    uint64_t last_sent_aubo_point_id;
    std::mutex last_sent_aubo_point_mutex;
    // [plan points]
    std::deque<std::array<double, 6>> plan_point_queue;
    uint64_t plan_points_parent_id;
    geometry_msgs::msg::Pose last_planned_pose;
    std::mutex plan_points_mutex;
    // [macsize estimate]
    uint16_t last_estimated_point_size;
    double last_estimated_point_size_time;
    // [hand filter]
    PositionFilter<2, 1> pos_filter;
    double q_x;
    double q_v;
    double r_x;
    // [hungry]
    bool fill_to_avoid_hunger;
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
        node_for_move_group(std::make_shared<rclcpp::Node>("node_for_move_group")),
        move_group_interface(node_for_move_group, "classic_six"),
        joint_model_group([&]() {
            return this->move_group_interface.getRobotModel()->getJointModelGroup("classic_six");
        }()),
        // [t.like inter]
        node_for_srv(std::make_shared<rclcpp::Node>("node_for_srv")),
        // [psm]
        node_for_monitor(std::make_shared<rclcpp::Node>("node_for_monitor")),
        psm(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            this->node_for_move_group,
            "robot_description"
        )),
        // [aubo.libtopic]
        mac_size_sub(this->create_subscription<std_msgs::msg::Int16>(
            "/mac_target_pos_data_size",
            10,
            [this](const std_msgs::msg::Int16::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->sub_mac_size_mutex);
                this->sub_mac_size = msg->data;
            }
        ))
    // [t.service]
    // 太抽象
    //[load]
    // virtual_planning_scene([&]() {
    //     robot_model_loader::RobotModelLoader robot_model_loader(
    //         this->node_for_move_group,
    //         "robot_description"
    //     );
    //     const ::moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    //     return planning_scene::PlanningScene(kinematic_model);
    // }())
    //
    {
        // planning_scene([&]() { return planning_scene::PlanningScene(this->robot_model); }()) {
        // planning_scene_monitor::PlanningSceneMonitor asd(node_for_move_group, "");
        // planning_scene::PlanningScene(this->move_group_interface.getRobotModel());
        RCLCPP_INFO(this->get_logger(), "Moveit ik started.");

        // [psm]
        // if (!psm->getPlanningScene()) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to get planning scene");
        //     return;
        // }
        this->psm->startSceneMonitor("/monitored_planning_scene");
        this->psm->startStateMonitor("/joint_states");
        {
            bool success = this->psm->requestPlanningSceneState("/get_planning_scene");
            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get planning scene");
            } else {
                RCLCPP_INFO(this->get_logger(), "Got planning scene");
            }
        }
        // psm->startWorldGeometryMonitor();
        // psm->startStateMonitor();
        // {
        //     using namespace std::chrono_literals;
        //     rclcpp::sleep_for(2s);
        // }

        // [timer]
        {
            const double fps_moveit_plan = this->get_parameter("fps_moveit_plan").as_double();
            RCLCPP_INFO(this->get_logger(), "fps_moveit_plan: %f", fps_moveit_plan);
            this->timer_moveit_plan = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / fps_moveit_plan),
                [this]() { this->callback_internal_ik(); }
            );
            const double fps_aubo_query = this->get_parameter("fps_aubo_query").as_double();
            // const double fps_aubo_query = 1.0;
            this->timer_aubo_query = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / fps_aubo_query),
                [this]() { this->callback_aubo_query(); }
            );
        }
        // [node for movegroup]
        std::thread([&]() { rclcpp::spin(node_for_move_group); }).detach();

        // [constants]
        this->fps_aubo_consume = this->get_parameter("fps_aubo_consume").as_double();
        this->latency_hand_to_before_plan =
            this->get_parameter("latency_hand_to_before_plan").as_double();
        this->latency_after_plan_others =
            this->get_parameter("latency_after_plan_others").as_double();
        this->size_expected_mac_points = this->get_parameter("size_expected_mac_points").as_int();
        this->vm = this->get_parameter("vm").as_bool();
        this->fill_to_avoid_hunger = this->get_parameter("fill_to_avoid_hunger").as_bool();

        // [robot init]
        {
            const char* host = "192.168.1.7";
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
            this->robot_service.robotServiceSetRobotCollisionClass(9);
        }
        // [aubo state]
        {
            // only needed in init time (later we'll keep updateing it)
            aubo_robot_namespace::JointStatus joint_status[6];
            const int ret = this->robot_service.robotServiceGetRobotJointStatus(joint_status, 6);
            if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
                std::string init_joints;
                for (size_t i = 0; i < 6; ++i) {
                    this->last_sent_aubo_point[i] = joint_status[i].jointPosJ; // ok this
                    init_joints +=
                        std::to_string(this->last_sent_aubo_point[i] / M_PI * 180.0) + " ";
                }
                RCLCPP_INFO(this->get_logger(), "init joints %s", init_joints.c_str());
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

            // [load]

            // auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node_for_move_group);
            // moveit_cpp->getPlanningSceneMonitor()->providePlanningSceneService();
            // ::moveit::planning_interface::PlanningSceneInterface planning_scene_interface(moveit_cpp
            // );
            // [old]
            ::moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            planning_scene_interface.applyCollisionObject(collision_object);
            planning_scene_interface.getObjects();
            for (const auto& object: planning_scene_interface.getObjects()) {
                RCLCPP_INFO(this->get_logger(), "Object: %s", object.first.c_str());
            }

            // [t.monitor]
            // auto current_state = move_group_interface.getCurrentState();
            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "desk type %s",
            //     planning_scene_monitor::LockedPlanningSceneRO(this->psm)
            //         ->getObjectType("desk")
            //         .db.c_str()
            // );

            // [t.monitor]
            // RCLCPP_INFO(this->get_logger(), "<<<");
            // rclcpp::sleep_for(std::chrono::seconds(1));
            // this->psm->updateSceneWithCurrentState();
            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "%s",
            //     psm->getPlanningScene()->getObjectType("desk").db.c_str()
            // );
            // RCLCPP_INFO(this->get_logger(), ">>>");

            // [t.local]
            // ::moveit::PlanningScene planning_scene(this->move_group_interface.getRobotModel());

            // [t.like inter]
            {
                RCLCPP_INFO(this->get_logger(), "[t.like inter]");
                this->ps_srv =
                    this->node_for_srv->create_client<moveit_msgs::srv::GetPlanningScene>(
                        move_group::GET_PLANNING_SCENE_SERVICE_NAME
                    );
                auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
                moveit_msgs::srv::GetPlanningScene::Response::SharedPtr response;
                std::vector<std::string> result;
                request->components.components = request->components.WORLD_OBJECT_NAMES;

                // auto planning_scene = planning_scene::PlanningScene();
                auto res = this->ps_srv->async_send_request(request);
                if (rclcpp::spin_until_future_complete(this->node_for_srv, res)
                    != rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(this->get_logger(), "[t.like inter] Failed to get planning scene");
                }
                response = res.get();
                for (const moveit_msgs::msg::CollisionObject& collision_object:
                     response->scene.world.collision_objects)
                {
                    result.push_back(collision_object.id);
                }
                for (const std::string& object_id: result) {
                    RCLCPP_INFO(this->get_logger(), "Object: %s", object_id.c_str());
                }
            }
        }

        {

        } // [t.r0]
        {
            planning_scene_monitor::LockedPlanningSceneRO planning_scene(psm);
            const auto& collision_objects = planning_scene->getWorld()->getObjectIds();
            // print their names
            RCLCPP_INFO(this->get_logger(), "num: %d", int(collision_objects.size())); // 0

            int box_count = 0;
            for (const auto& object_id: collision_objects) {
                // 获取每个碰撞对象
                auto object = planning_scene->getWorld()->getObject(object_id);

                // 检查对象是否为盒子（box）类型
                for (const auto& shape: object->shapes_) {
                    if (shape->type == shapes::ShapeType::BOX || true) {
                        ++box_count;
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), "There are %d boxes in the scene", box_count);
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

        {
            // estimate
            this->last_estimated_point_size = 0;
            this->last_estimated_point_size_time = this->now().seconds();
        }
    }

private:
    void callback_internal_ik() {
        //
        const auto [pos, ori] = this->get_pose();
        const double ts_before_plan = this->now().seconds();
        const auto ts_estimated_hand = ts_before_plan - this->latency_hand_to_before_plan;
        this->pos_filter.update(
            { pos.x(), pos.y(), pos.z() },
            ts_estimated_hand,
            { this->q_x, this->q_v },
            { this->r_x }
        );

        const auto mac_size = this->get_aubo_mac_size_by_another_tcp();
        const auto points_size = mac_size / 6 + (mac_size % 6 > 0 ? 1 : 0);
        const double ts_estimated_effector_arraival = ts_before_plan
            + double(points_size) / this->fps_aubo_consume + this->latency_after_plan_others;
        const auto pos_estimated_effector =
            this->pos_filter.predict_pos(ts_estimated_effector_arraival);

        // print_t("#2");

        struct V1Out {
            double base;
            double min;
            double max;
        };
        const auto v1_out_pos = [](const Eigen::Vector3<double>& position,
                                   const V1Out& outx,
                                   const V1Out& outy,
                                   const V1Out& outz,
                                   const double ratio) {
            return tf2::Vector3(
                std::clamp(outx.base + position.x() * ratio, outx.min, outx.max),
                std::clamp(outy.base + position.y() * ratio, outy.min, outy.max),
                std::clamp(outz.base + position.z() * ratio, outz.min, outz.max)
            );
        };
        struct Out {
            double base;
            double min;
            double max;
            double ratio;
        };
        const auto out_pos = [](const Eigen::Vector3<double>& position,
                                const Out& outx,
                                const Out& outy,
                                const Out& outz) {
            return tf2::Vector3(
                std::clamp(outx.base + position.x() * outx.ratio, outx.min, outx.max),
                std::clamp(outy.base + position.y() * outy.ratio, outy.min, outy.max),
                std::clamp(outz.base + position.z() * outz.ratio, outz.min, outz.max)
            );
        };

        const double ratio = 1.0;
        const auto out_esti_pos = out_pos(
            pos_estimated_effector,
            { +0.4, -0.7, 0.7, ratio },
            { 0, -1.8, 1.8, ratio },
            { 0.35, 0.15, 1.0, ratio }
        );

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

        const auto point = this->get_internal_ik(from, to);

        if (point.has_value()) {
            if (this->collide(point.value())) {
                return;
            }
            std::lock_guard<std::mutex> lock(this->plan_points_mutex);
            // check trans and angular dis
            const double hyp = std::hypot(
                to.position.x - this->last_planned_pose.position.x,
                to.position.y - this->last_planned_pose.position.y,
                to.position.z - this->last_planned_pose.position.z
            );
            const double the = qua_theta(
                tf2::Quaternion(
                    to.orientation.x,
                    to.orientation.y,
                    to.orientation.z,
                    to.orientation.w
                ),
                tf2::Quaternion(
                    this->last_planned_pose.orientation.x,
                    this->last_planned_pose.orientation.y,
                    this->last_planned_pose.orientation.z,
                    this->last_planned_pose.orientation.w
                )
            );
            // if (hyp < POS_MIN_DIFF_TO_SEND && the < deg2rad(ANGLE_MIN_DIFF_TO_SEND)) {
            //     RCLCPP_INFO(
            //         this->get_logger(),
            //         "diff too small, cancel plan. pos is %f, angle is %f",
            //         hyp,
            //         the
            //     );
            //     return;
            // }

            this->last_planned_pose = to;
            if (this->plan_points_parent_id != this_plan_parent_id) {
                while (!this->plan_point_queue.empty()) {
                    this->plan_point_queue.pop_front();
                }
            }
            this->plan_point_queue.push_back(point.value());
            // this->last_sent_aubo_point = point;
            // this->moveit_points_queue.push(poin);
            this->plan_points_parent_id = this->last_sent_aubo_point_id;
        }
    }

    bool collide(const std::array<double, 6>& joint_angles) {
        using namespace collision_detection;
        CollisionRequest request;
        CollisionResult result;
        request.contacts = true;
        request.max_contacts = 100;

        planning_scene_monitor::LockedPlanningSceneRW planning_scene(psm);
        // ::moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        auto robot_state = planning_scene->getCurrentStateNonConst();
        robot_state.setJointGroupPositions(this->joint_model_group, joint_angles.data());
        planning_scene->checkSelfCollision(request, result, robot_state);
        if (result.collision) {
            RCLCPP_ERROR(this->get_logger(), "Self Collision!");
            return true;
        }
        planning_scene->checkCollision(request, result, robot_state);
        if (result.collision) {
            RCLCPP_ERROR(this->get_logger(), "Out Collision!");
            return true;
        }
        return false;
    }

    void callback_aubo_query() {
        // RCLCPP_INFO(this->get_logger(), "########### callback_aubo_query");
        const auto mac_size = this->get_aubo_mac_size_by_another_tcp();
        {
            // [last sent]
            const auto time_interval = this->now().seconds() - this->last_estimated_point_size_time;
            const int consumed = std::floor(time_interval * this->fps_aubo_consume);
            this->last_estimated_point_size =
                uint16_t(std::max(0, int(this->last_estimated_point_size) - consumed));
            this->last_estimated_point_size_time += double(consumed) / this->fps_aubo_consume;
            // RCLCPP_INFO(
            //     this->get_logger(),
            //     "time_interval: %f, consumed: %d, last_estimated_point_size: %d",
            //     time_interval,
            //     consumed,
            //     int(this->last_estimated_point_size)
            // );
        }
        const auto points_size = std::max(
            int(this->last_estimated_point_size),
            mac_size / 6 + (mac_size % 6 > 0 ? 1 : 0)
        );
        // const auto points_size = mac_size / 6 + (mac_size % 6 > 0 ? 1 : 0);

        {
            // print this->last_estimated_point_size,
            // mac_size / 6 + (mac_size % 6 > 0 ? 1 : 0)
            //
            RCLCPP_INFO(
                this->get_logger(),
                "est: %d, mac / 6: %d",
                int(this->last_estimated_point_size),
                mac_size / 6 + (mac_size % 6 > 0 ? 1 : 0)
            );
        }
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
            std::lock_guard<std::mutex> lock_plan(this->plan_points_mutex);
            if (this->plan_points_parent_id != this->last_sent_aubo_point_id) {
                while (!this->plan_point_queue.empty()) {
                    this->plan_point_queue.pop_front();
                } // 必须防止饥饿，就算数据没用都要送过去一堆老的
            }
            // ? 必须保证 moveit_points_queue 始终有效
            std::vector<aubo_robot_namespace::wayPoint_S> waypoint_vector;
            std::lock_guard<std::mutex> lock_sent(this->last_sent_aubo_point_mutex);
            auto pre_point = this->last_sent_aubo_point;

            // [v1: always fill]
            int points_needed = this->size_expected_mac_points - points_size;
            RCLCPP_INFO(this->get_logger(), "points_needed: %d", points_needed);

            // [v2]
            // int points_needed = [&]() {
            //     if (this->fill_to_avoid_hunger) {
            //         return this->size_expected_mac_points - points_size;
            //     }
            //     if (this->plan_point_queue.empty()) {
            //         return 0;
            //     }
            //     return std::min(this->size_expected_mac_points - points_size, 7);
            // }();
            // RCLCPP_INFO(this->get_logger(), "points_needed: %d", points_needed);

            while (points_needed > 0) {
                if (this->plan_point_queue.empty()) {
                    waypoint_vector.push_back(arr_to_waypoint(pre_point));
                    points_needed--;
                    continue;
                }
                // make it close to pre_point
                const auto nxt_point =
                    limited_closest_joint_angles(this->plan_point_queue.front(), pre_point);
                const int interpolation_num = [&]() {
                    int itpltn = 1;
                    for (size_t i = 0; i < 6; ++i) {
                        // 假设电机就会这样执行
                        const double vel = std::abs(reduced_angle(nxt_point[i] - pre_point[i]))
                            / (1.0 / this->fps_aubo_consume);
                        itpltn =
                            std::max(itpltn, int(ceil(vel / deg2rad(JOINTS_ANGLE_MAX_VEL[i]))));
                    }
                    return itpltn;
                }();
                RCLCPP_INFO(this->get_logger(), "interpolation_num: %d", interpolation_num);
                for (int i = 0; i < interpolation_num && points_needed > 0; i++) {
                    std::array<double, 6> point;
                    for (size_t j = 0; j < 6; ++j) {
                        // don't push pre_joint
                        // point[j] = pre_point[j]
                        //     + (nxt_point[j] - pre_point[j]) * (i + 1) / interpolation_num;
                        point[j] = angle_interpolation(
                            pre_point[j],
                            nxt_point[j],
                            i + 1,
                            interpolation_num
                        );
                        // if (j == 0)
                        //     RCLCPP_INFO(
                        //         this->get_logger(),
                        //         "joint %d: %f -> %f",
                        //         int(j),
                        //         pre_point[j],
                        //         point[j]
                        //     );
                    }
                    waypoint_vector.push_back(arr_to_waypoint(point));
                    if (i == interpolation_num - 1) {
                        pre_point = nxt_point;
                        this->plan_point_queue.pop_front();
                    }
                    points_needed--;
                }
            }
            return waypoint_vector;
        }();
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "in %d send %d",
        //     int(points_size),
        //     int(waypoint_vector.size())
        // );
        if (waypoint_vector.empty()) {
            RCLCPP_INFO(this->get_logger(), "no points to send");
            return;
        }
        {
            std::lock_guard<std::mutex> lock_sent(this->last_sent_aubo_point_mutex);
            this->robot_service.robotServiceSetRobotPosData2Canbus(waypoint_vector);
            this->last_estimated_point_size += uint16_t(waypoint_vector.size());
            this->last_sent_aubo_point = waypoint_to_arr(waypoint_vector.back());
            this->last_sent_aubo_point_id++;
            // show size
            RCLCPP_INFO(
                this->get_logger(),
                "sent %d points, %d points in plan queue",
                int(waypoint_vector.size()),
                int(this->plan_point_queue.size())
            );
            // this time sent joint[0] are:
            // for (auto i: waypoint_vector) {
            //     RCLCPP_INFO(this->get_logger(), "joint[0]: %f", i.jointpos[0] / M_PI * 180.0);
            // }
            for (auto i: waypoint_vector) {
                std::string joints;
                for (size_t j = 0; j < 6; ++j) {
                    joints += std::to_string(i.jointpos[j] / M_PI * 180.0).substr(0, 7) + " ";
                }
                RCLCPP_INFO(this->get_logger(), "joints: %s", joints.c_str());
            }
        }
    }

    uint16_t get_aubo_mac_size_by_another_tcp() {
        if (this->vm) {
            return uint16_t(this->size_expected_mac_points - 5) * 6;
        }
        std::lock_guard<std::mutex> lock(this->sub_mac_size_mutex);
        return this->sub_mac_size;
    }

    uint16_t get_aubo_mac_size_by_diagnosis() {
        // count time here
        using namespace std::chrono;

        if (this->vm) {
            return uint16_t(this->size_expected_mac_points - 5) * 6;
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
        RCLCPP_INFO(
            this->get_logger(),
            "macTargetPosDataSize: %d",
            robot_diagnosis_info.macTargetPosDataSize
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

    std::optional<std::array<double, 6>>
    get_internal_ik(const std::array<double, 6>& init, const Pose& to) {
        const double initial_positions_arr[] = { init[0], init[1], init[2],
                                                 init[3], init[4], init[5] };
        aubo_robot_namespace::wayPoint_S internal_sol;
        int ret = this->robot_service.robotServiceRobotIk(
            initial_positions_arr,
            { to.position.x, to.position.y, to.position.z },
            { to.orientation.w, to.orientation.x, to.orientation.y, to.orientation.z },
            internal_sol
        );
        if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
            RCLCPP_ERROR(this->get_logger(), "ik failed");
            return std::nullopt;
        }
        return std::array<double, 6> { internal_sol.jointpos[0], internal_sol.jointpos[1],
                                       internal_sol.jointpos[2], internal_sol.jointpos[3],
                                       internal_sol.jointpos[4], internal_sol.jointpos[5] };
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

            // print_t("##1");

            ::moveit::core::RobotState start_state(*current_state);
            std::vector<double> joint_group_positions;
            start_state.copyJointGroupPositions(
                current_state->getJointModelGroup("classic_six"),
                joint_group_positions
            );
            // print_t("##2");
            for (size_t i = 0; i < 6; ++i) {
                joint_group_positions[i] = from[i];
            }
            start_state.setJointGroupPositions(
                current_state->getJointModelGroup("classic_six"),
                joint_group_positions
            );
            // print_t("##3");
            this->move_group_interface.setStartState(start_state);
            // print_t("##4");
        }
        // [Set a target Pose]
        this->move_group_interface.setPoseTarget(to);
        // print_t("##5");
        auto const [success, plan] = [this] {
            ::moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(this->move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();
        // print_t("##6");
        if (success) {
            return plan.trajectory_.joint_trajectory.points;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            return std::nullopt;
        }
    }
};

//

} // namespace ik::moveit

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ik::moveit::Ik)
