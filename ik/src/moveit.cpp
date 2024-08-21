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
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serviceinterface.h>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16.hpp>
#include <sys/types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <trajectory_msgs/msg/detail/joint_trajectory_point__struct.hpp>

#include "AuboRobotMetaType.h"
#include "ik/filter.hpp"

namespace ik::moveit {

using std::deque;
using std::mutex;
using std::string;
using std::to_string;
using std::tuple;
using std::vector;

using Points = trajectory_msgs::msg::JointTrajectory::_points_type;
using geometry_msgs::msg::Pose;
using ik::PositionFilter;
using ik::PositionPredictorInterface;
using ::moveit::planning_interface::MoveGroupInterface;

// const double JOINTS_ANGLE_MAX_VEL[6] = {
//     2.596177, 2.596177, 2.596177, 3.110177, 3.110177, 3.110177
// };
// / 15 / pi * 180
// const double JOINTS_ANGLE_MAX_VEL[6] = { 400.0, 400.0, 400.0, 800.0, 800.0, 800.0 };
// const double JOINTS_ANGLE_MAX_VEL[6] = { 100.0, 100.0, 100.0, 200.0, 200.0, 200.0 };
// const double JOINTS_ANGLE_MAX_ACC[6] = {
//     17.30878, 17.30878, 17.30878, 20.73676, 20.73676, 20.73676
// };
// div 3.14 * 180
// const double JOINTS_ANGLE_MAX_ACC[6] = { 5.5, 5.5, 5.5, 6.6, 6.6, 6.6 };
// * 180
// const double JOINTS_ANGLE_MAX_ACC_DEG[6] = { 400.0, 400.0, 400.0, 420.0, 420.0, 422.0 };

// mul 2
// const double JOINTS_ANGLE_MAX_VEL[6] = { 40.0, 40.0, 40.0, 44.0, 44.0, 44.0 };
// div 4
// const double JOINTS_ANGLE_MAX_VEL[6] = { 5.0, 5.0, 5.0, 5.5, 5.5, 5.5 };
// const double POS_MIN_DIFF_TO_SEND = 0.005;
// const double ANGLE_MIN_DIFF_TO_SEND = 1.0;

std::string vecd_to_string(const vector<double>& vec) {
    std::string res;
    for (const auto& v: vec) {
        res += std::to_string(v) + " ";
    }
    return res;
}

std::string array6_to_string(const std::array<double, 6>& arr) {
    std::string res;
    for (size_t i = 0; i < 6; ++i) {
        res += std::to_string(arr[i] / M_PI * 180.0).substr(0, 7) + " ";
    }
    return res;
}

template<typename T, size_t N>
std::array<T, N> minus(const std::array<T, N>& a, const std::array<T, N>& b) {
    std::array<T, N> result;
    for (size_t i = 0; i < N; ++i) {
        result[i] = a[i] - b[i];
    }
    return result;
}

template<typename T, size_t N>
std::array<T, N> mul(const std::array<T, N>& a, const T b) {
    std::array<T, N> result;
    for (size_t i = 0; i < N; ++i) {
        result[i] = a[i] * b;
    }
    return result;
}

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

double real_mod(const double x, const double d) {
    return x - std::floor(x / d) * d;
}

const double CHECK_ANGLE_LIMIT[6][2] = { { -360, 360 }, { -175, 175 }, { -175, 175 },
                                         { -175, 175 }, { -175, 175 }, { -360, 360 } };
// 0.5 degree stricter
const double SENT_ANGLE_LIMIT[6][2] = { { -359.5, 359.5 }, { -174.5, 174.5 }, { -174.5, 174.5 },
                                        { -174.5, 174.5 }, { -174.5, 174.5 }, { -359.5, 359.5 } };

const double deg2rad(const double deg) {
    return deg / 180.0 * M_PI;
}

const double rad2deg(const double rad) {
    return rad / M_PI * 180.0;
}

std::array<double, 6> limited_closest_joint_angles(
    const std::array<double, 6>& pre_angles_thats_reduced,
    const std::array<double, 6>& target_angles
) {
    std::array<double, 6> result;
    for (size_t i = 0; i < 6; ++i) {
        result[i] = closest_angle(pre_angles_thats_reduced[i], target_angles[i]);
        result[i] =
            std::clamp(result[i], deg2rad(SENT_ANGLE_LIMIT[i][0]), deg2rad(SENT_ANGLE_LIMIT[i][1]));
    }
    return result;
}

class Ik: public rclcpp::Node {
private:
    // [meta]
    std::string group_name;
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
    // [control]
    double latency_hand_to_before_plan;
    double latency_after_plan_others;
    // [plan state]
    std::array<double, 6> last_sent_aubo_point;
    std::array<double, 6> last_sent_aubo_point_vel;
    uint64_t last_sent_aubo_point_id;
    // std::mutex last_sent_aubo_point_mutex;
    // [plan points]
    std::deque<std::array<double, 6>> plan_point_queue;
    uint64_t in_plan_points_ancestor_id;
    std::array<double, 6> last_planned_point; // maybe sent in plan_point_queue
    geometry_msgs::msg::Pose last_planned_pose;
    std::mutex plan_points_mutex;
    // [macsize estimate]
    std::mutex last_estimated_point_size_mutex;
    uint16_t last_estimated_point_size;
    double last_estimated_point_size_time;
    // [hand filter]
    PositionFilter<2, 1> hand_pos_filter;
    double q_x;
    double q_v;
    double r_x;
    // [vm]
    bool check_joint_broadcaster;
    // [joints max vel and max acc]
    std::array<double, 6> joints_deg_max_vel;
    std::array<double, 6> joints_deg_max_acc;
    // [hand]
    double hand_pos_min_diff_to_plan;
    double hand_deg_min_diff_to_plan;
    // [透传时间监视]
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr print_canbus_sub;
    std::deque<double> canbus_timeq;
    std::mutex canbus_timeq_mutex;
    // [always-sent-start mode] don't plan
    // [joint cmd test]
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_cmd_pub;
    // [plan mode.linear]
    double begin_time;
    // [member: points]
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr print_debugq_sub;
    std::mutex debug_vr_pose_mutex;
    std::deque<std::vector<double>> debug_vr_poses;
    std::mutex debug_sent_fk_mutex;
    deque<aubo_robot_namespace::wayPoint_S> debug_sent_fk;

public:
    explicit Ik(const rclcpp::NodeOptions& options):
        Node(
            "fake_ik_moveit",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        ),
        group_name(this->get_parameter("group_name").as_string()),
        tf_buffer(this->get_clock()),
        tf_listener(this->tf_buffer),
        // [moveit]
        node_for_move_group(std::make_shared<rclcpp::Node>(
            "node_for_move_group",
            rclcpp::NodeOptions().allow_undeclared_parameters(true)
        )),
        move_group_interface(node_for_move_group, this->group_name),
        joint_model_group([&]() {
            return this->move_group_interface.getRobotModel()->getJointModelGroup(this->group_name);
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
            "/" + this->group_name + "_mac_target_pos_data_size",
            10,
            [this](const std_msgs::msg::Int16::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(this->sub_mac_size_mutex);
                this->sub_mac_size = msg->data;
            }
        )),
        joint_cmd_pub(this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/" + this->group_name + "_controller/joint_trajectory",
            10
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
        // auto kin_model = this->move_group_interface.getRobotModel();
        // const ::moveit::core::JointModelGroup * joint_model_group = kin_model->
        // auto kinematic_state = this->move_group_interface.getCurrentState();
        // kinematic_state->setToDefaultValues();

        // [constants]
        this->fps_aubo_consume = this->get_parameter("fps_aubo_consume").as_double();
        this->latency_hand_to_before_plan =
            this->get_parameter("latency.hand_to_before_plan").as_double();
        this->latency_after_plan_others =
            this->get_parameter("latency.after_plan_others").as_double();
        this->size_expected_mac_points = this->get_parameter("size_expected_mac_points").as_int();
        this->check_joint_broadcaster = this->get_parameter("check_joint_broadcaster").as_bool();
        this->joints_deg_max_vel = [&]() {
            const auto param = this->get_parameter("joints_deg_max_vel").as_double_array();
            return std::array<double, 6> { param[0], param[1], param[2],
                                           param[3], param[4], param[5] };
        }();
        this->joints_deg_max_acc = [&]() {
            const auto param = this->get_parameter("joints_deg_max_acc").as_double_array();
            return std::array<double, 6> { param[0], param[1], param[2],
                                           param[3], param[4], param[5] };
        }();
        this->hand_pos_min_diff_to_plan =
            this->get_parameter("hand_pos_min_diff_to_plan").as_double();
        this->hand_deg_min_diff_to_plan =
            this->get_parameter("hand_deg_min_diff_to_plan").as_double();

        // [check topic]
        {
            if (this->check_joint_broadcaster) {
                int has_message =
                    this->count_publishers("/" + this->group_name + "_mac_target_pos_data_size");
                if (has_message == 0) {
                    RCLCPP_ERROR(this->get_logger(), "No publisher for /mac_target_pos_data_size");
                    return;
                }
            }
        }

        {
            using namespace std::chrono_literals;
            auto parameters_client =
                std::make_shared<rclcpp::SyncParametersClient>(node_for_move_group, "/move_group");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(
                        this->node_for_move_group->get_logger(),
                        "Interrupted while waiting for the service. Exiting."
                    );
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(
                    this->node_for_move_group->get_logger(),
                    "service not available, waiting again..."
                );
            }
            rcl_interfaces::msg::ListParametersResult parameter_list =
                parameters_client->list_parameters({ "robot_description_kinematics" }, 10);
            auto parameters = parameters_client->get_parameters(parameter_list.names);

            // set the parameters to our node
            this->node_for_move_group->set_parameters(parameters);
        }

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

        // [timer]
        {
            const double fps_moveit_plan = this->get_parameter("fps_moveit_plan").as_double();
            RCLCPP_INFO(this->get_logger(), "fps_moveit_plan: %f", fps_moveit_plan);
            this->timer_moveit_plan = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / fps_moveit_plan),
                [this]() { this->callback_ik(); }
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

        // [robot init]
        {
            const auto host = this->get_parameter("aubo_ip").as_string();
            const int port = this->get_parameter("aubo_service_port").as_int();
            const int ret =
                this->robot_service.robotServiceLogin(host.c_str(), port, "aubo", "123456");
            if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
                RCLCPP_INFO(this->get_logger(), "login success.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "login failed");
                return;
            }
            this->robot_service.robotServiceEnterTcp2CanbusMode();
            this->robot_service.robotServiceInitGlobalMoveProfile();
            this->robot_service.robotServiceSetRobotCollisionClass(8);
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
                    this->last_sent_aubo_point_vel[i] = 0.0;
                    init_joints +=
                        std::to_string(this->last_sent_aubo_point[i] / M_PI * 180.0) + " ";
                }
                this->last_planned_point = this->last_sent_aubo_point;
                this->in_plan_points_ancestor_id = 0;
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
                primitive.dimensions[primitive.BOX_X] = 4.0;
                primitive.dimensions[primitive.BOX_Y] = 4.0;
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
            // [old]
            ::moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            planning_scene_interface.applyCollisionObject(collision_object);
            planning_scene_interface.getObjects();
            for (const auto& object: planning_scene_interface.getObjects()) {
                RCLCPP_INFO(this->get_logger(), "Object: %s", object.first.c_str());
            }
            // [t.like inter]
            {
                // 这部分模仿 PlanningInterface 实现物体查询
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

            this->q_x = this->get_parameter("pos_filter.q.x").as_double();
            this->q_v = this->get_parameter("pos_filter.q.v").as_double();
            this->r_x = this->get_parameter("pos_filter.r.x").as_double();
        }

        {
            // estimate
            this->last_estimated_point_size = 0;
            this->last_estimated_point_size_time = this->now().seconds();
        }

        {
            this->print_canbus_sub = this->create_subscription<std_msgs::msg::Empty>(
                "/ik/pct",
                10,
                [this](std_msgs::msg::Empty::SharedPtr msg) { this->print_that(msg); }
            );
            this->print_debugq_sub = this->create_subscription<std_msgs::msg::String>(
                "/ik/pq" + to_string(this->get_parameter("vr_idx").as_int()),
                10,
                [this](std_msgs::msg::String::SharedPtr msg) { this->sub_print_one_q(msg); }
            );
        }
        this->begin_time = this->now().seconds();
    }

private:
    void print_that(std_msgs::msg::Empty::SharedPtr) {
        this->canbus_timeq_mutex.lock();
        std::string res;
        for (const auto& time: this->canbus_timeq) {
            res += std::to_string(time * 1000) + " ";
        }
        this->canbus_timeq_mutex.unlock();
        std::ofstream file(std::string(std::getenv("HOME")) + "/.teleop/data.txt", std::ios::out);
        file << res;
        file.close();
    }
    void print_one_q(mutex& mut, deque<vector<double>>& q, string filename) {
        std::lock_guard<std::mutex> lock(mut);
        std::string res;
        for (const auto& v: q) {
            res += vecd_to_string(v) + "\n";
        }
        std::ofstream file(string(std::getenv("HOME")) + "/.teleop/" + filename, std::ios::out);
        file << res;
        file.close();
    }
    void sub_print_one_q(std_msgs::msg::String::SharedPtr qname) {
        if (qname->data == "vr") {
            this->print_one_q(
                this->debug_vr_pose_mutex,
                this->debug_vr_poses,
                this->get_parameter("record_vr_file").as_string()
            );
        } else if (qname->data == "fk") {
            deque<vector<double>> fkq;
            {
                std::lock_guard lock(this->debug_sent_fk_mutex);
                for (const auto& wp: this->debug_sent_fk) {
                    vector<double> fk;
                    aubo_robot_namespace::wayPoint_S wp_fk;
                    this->robot_service.robotServiceRobotFk(wp.jointpos, 6, wp_fk);
                    fk.push_back(wp_fk.cartPos.position.x);
                    fk.push_back(wp_fk.cartPos.position.y);
                    fk.push_back(wp_fk.cartPos.position.z);
                    fk.push_back(wp_fk.orientation.x);
                    fk.push_back(wp_fk.orientation.y);
                    fk.push_back(wp_fk.orientation.z);
                    fk.push_back(wp_fk.orientation.w);
                    fkq.push_back(fk);
                }
            }
            this->print_one_q(this->debug_sent_fk_mutex, fkq, "fk.txt");
        }
    }

    void callback_ik() {
        if (this->get_parameter("plan_mode").as_string() == "no") {
            return;
        }
        if (this->get_parameter("plan_mode").as_string() == "init_point") {
            std::lock_guard<std::mutex> lock(this->plan_points_mutex);
            while (!this->plan_point_queue.empty()) {
                this->plan_point_queue.pop_front();
            }
            this->plan_point_queue.push_back(this->last_sent_aubo_point);
            return;
        }

        const auto [pos, ori] = this->get_hand_pose();
        const double ts_before_plan = this->now().seconds();
        const auto ts_estimated_hand = ts_before_plan - this->latency_hand_to_before_plan;
        this->hand_pos_filter.update(
            { pos.x(), pos.y(), pos.z() },
            ts_estimated_hand,
            { this->q_x, this->q_v },
            { this->r_x }
        );

        const auto points_size = this->get_esti_limited_point_size();
        const double ts_estimated_effector_arraival = ts_before_plan
            + double(points_size) / this->fps_aubo_consume + this->latency_after_plan_others;
        const auto pos_predicted_when_effector_arrives =
            this->hand_pos_filter.predict_pos(ts_estimated_effector_arraival);
        const auto pos_no_predict = this->hand_pos_filter.predict_pos(ts_estimated_hand);

        struct Out {
            double base;
            double min;
            double max;
            double ratio;
        };
        const auto scaled_pos_fn = [](const Eigen::Vector3<double>& position,
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
        const auto so_you_want = [&]() {
            if (!this->get_parameter("pos_filter.enabled").as_bool()) {
                return Eigen::Vector3d { pos.x(), pos.y(), pos.z() };
            }
            if (this->get_parameter("pos_filter.prediction").as_bool()) {
                return pos_predicted_when_effector_arrives;
            }
            return pos_no_predict;
        }();

        {
            std::lock_guard lock(this->debug_vr_pose_mutex);
            if (this->debug_vr_poses.size()
                >= this->get_parameter("launchtime_debug_queue_size").as_int()) {
                this->debug_vr_poses.pop_front();
            }
            std::vector<double> pose;
            pose.push_back(so_you_want.x());
            pose.push_back(so_you_want.y());
            pose.push_back(so_you_want.z());
            pose.push_back(ori.x());
            pose.push_back(ori.y());
            pose.push_back(ori.z());
            pose.push_back(ori.w());
            this->debug_vr_poses.push_back(pose);
        }

        const auto scaled_pos = scaled_pos_fn(
            so_you_want,
            { +0.4, -0.7, 0.7, ratio },
            { 0, -1.8, 1.8, ratio },
            { 0.2, 0.15, 1.0, ratio }
        );

        // 左手 hand y = 0.4 时，应该解算 x = 0.0
        const auto right_left_pos = this->get_parameter("mirror_the_hand").as_bool()
            ? tf2::Vector3(-scaled_pos.y(), -scaled_pos.x(), scaled_pos.z())
            : tf2::Vector3(scaled_pos.y(), -scaled_pos.x(), scaled_pos.z());

        const auto [this_plan_ancestor_id, from] = [&]() {
            // std::lock_guard<std::mutex> lock(this->last_sent_aubo_point_mutex); // 不需要锁 sent aubo point
            const auto ancestor_id = this->in_plan_points_ancestor_id;
            const auto from = this->last_planned_point;
            return std::make_tuple(ancestor_id, from);
        }();
        const auto ori_here = ori;
        const auto to = [&] {
            if (this->get_parameter("plan_mode").as_string() == "linear") {
                const double half_t = this->get_parameter("plan_mode_linear.half_t").as_double();
                // [0 ~ 2.0]
                const double prop1 =
                    real_mod(this->now().seconds() - this->begin_time, 2 * half_t) / half_t;
                const double prop2 = std::min(prop1, 2.0 - prop1);
                auto param_vec = [this](const string& name) {
                    const auto vec = this->get_parameter(name).as_double_array();
                    assert(vec.size() == 3);
                    return Eigen::Vector3d(vec[0], vec[1], vec[2]);
                };
                const auto a = param_vec("plan_mode_linear.A");
                const auto b = param_vec("plan_mode_linear.B");
                const auto c = a + (b - a) * prop2;
                geometry_msgs::msg::Pose msg;
                msg.position.x = c[0];
                msg.position.y = c[1];
                msg.position.z = c[2];
                msg.orientation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
                return msg;
            }
            geometry_msgs::msg::Pose msg;
            msg.position.x = right_left_pos[0];
            msg.position.y = right_left_pos[1];
            msg.position.z = right_left_pos[2];
            msg.orientation = this->get_parameter("plan_mode_normal.q_up").as_bool()
                ? tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0))
                : tf2::toMsg(ori_here);
            return msg;
        }();

        const auto point = this->get_parameter("ik_method").as_string() == "internal"
            ? this->get_internal_ik(from, to)
            : this->get_kdl_ik(to);

        if (point.has_value()) {
            auto start = std::chrono::high_resolution_clock::now();
            if (this->collide_or_outofrange(point.value())) {
                return;
            }
            RCLCPP_INFO(
                this->get_logger(),
                "collide check time: %f",
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start)
                    .count()
            );
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
            // this is a very nice idea!
            if (hyp < this->hand_pos_min_diff_to_plan
                && the < deg2rad(this->hand_deg_min_diff_to_plan)) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "diff too small, cancel plan. pos is %f, deg is %f",
                    hyp,
                    rad2deg(the)
                );
                RCLCPP_INFO(
                    this->get_logger(),
                    "plan.size(): %d",
                    int(this->plan_point_queue.size())
                );
                return;
            }
            // 仅用于判断是否应该 plan，并不参与 plan 初值。居然割裂了 planned_pose 和 sent_pose，好吧这样也还可以
            // 毕竟最近一次 plan 是绝对的优先级最高，要想尽办法快速到达
            this->last_planned_pose = to;
            // if (this->plan_points_parent_id != this_plan_parent_id) {
            //     while (!this->plan_point_queue.empty()) {
            //         this->plan_point_queue.pop_front();
            //     }
            // }
            while (!this->plan_point_queue.empty()) {
                this->plan_point_queue.pop_front();
            }
            this->plan_point_queue.push_back(point.value());
            // this->last_sent_aubo_point = point;
            // this->moveit_points_queue.push(poin);
            this->in_plan_points_ancestor_id = this_plan_ancestor_id; // no use at all
        }
    }

    bool collide_or_outofrange(const std::array<double, 6>& joint_angles) {
        for (size_t i = 0; i < 6; ++i) {
            if (joint_angles[i] < deg2rad(CHECK_ANGLE_LIMIT[i][0])
                || joint_angles[i] > deg2rad(CHECK_ANGLE_LIMIT[i][1]))
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Joint %d out of range: %f",
                    int(i),
                    rad2deg(joint_angles[i])
                );
                return true;
            }
        }
        using namespace collision_detection;
        CollisionRequest request;
        CollisionResult result;
        request.contacts = true;
        request.max_contacts = 100;

        planning_scene_monitor::LockedPlanningSceneRW planning_scene(psm);
        // ::moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        auto robot_state = planning_scene->getCurrentStateNonConst();
        robot_state.setJointGroupPositions(this->joint_model_group, joint_angles.data());
        // how to broadcast it to planning scene?
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
        const auto points_size = this->get_esti_limited_point_size();
        const auto arr_to_waypoint = [](const std::array<double, 6>& arr) {
            aubo_robot_namespace::wayPoint_S waypoint;
            std::memcpy(waypoint.jointpos, arr.data(), 6 * sizeof(double));
            return waypoint;
        };
        // auto waypoint_to_arr = [](const aubo_robot_namespace::wayPoint_S& waypoint) {
        //     std::array<double, 6> arr;
        //     std::memcpy(arr.data(), waypoint.jointpos, 6 * sizeof(double));
        //     return arr;
        // };
        const auto [waypoint_vector, pre_sent_point, pre_sent_vel] = [&]() {
            // 这里要锁住，plan 线程会修改相关变量
            std::lock_guard<std::mutex> lock_plan(this->plan_points_mutex);
            // [这里之前用的 moveit plan，规划起点必须是上一个规划终点。现在换成直接 ik 了以后就没这必要了]
            // if (this->plan_points_parent_id != this->last_sent_aubo_point_id) { // should be something like current_plan ancestor id
            //     while (!this->plan_point_queue.empty()) {
            //         this->plan_point_queue.pop_front();
            //     } // 必须防止饥饿，就算数据没用都要送过去一堆老的
            // }
            std::vector<aubo_robot_namespace::wayPoint_S> waypoint_vector;
            // [todo] 此处只读，需要锁吗？只有本线程会修改和读取它，所以不用锁！
            // std::lock_guard<std::mutex> lock_sent(this->last_sent_aubo_point_mutex);
            auto pre_sent_point = this->last_sent_aubo_point;
            auto pre_sent_vel = this->last_sent_aubo_point_vel;

            int points_needed = this->size_expected_mac_points - points_size;
            RCLCPP_INFO(this->get_logger(), "points_needed: %d", points_needed);

            // [Try to consume a plan in each loop]
            bool ok_no_collide = true;
            while (ok_no_collide && points_needed > 0) {
                if (this->plan_point_queue.empty()) {
                    // this->plan_point_queue.push_back(pre_sent_point);
                    waypoint_vector.push_back(arr_to_waypoint(this->last_sent_aubo_point));
                    pre_sent_point = this->last_sent_aubo_point;
                    // all 0
                    pre_sent_vel = {};
                    points_needed--;
                    continue;
                }
                // make it close to pre_point
                const auto nxt_plan_point =
                    limited_closest_joint_angles(this->plan_point_queue.front(), pre_sent_point);
                RCLCPP_INFO(
                    this->get_logger(),
                    "nxt_plan_point: %s",
                    std::accumulate(
                        nxt_plan_point.begin(),
                        nxt_plan_point.end(),
                        std::string(),
                        [](std::string& a, double b) {
                            return a + std::to_string(b / M_PI * 180.0).substr(0, 7) + " ";
                        }
                    ).c_str()
                );
                const double joints_limit_rate = [&]() {
                    for (size_t i = 0; i < 6; ++i) {
                        if (std::abs(nxt_plan_point[i] - pre_sent_point[i])
                            > deg2rad(this->get_parameter("joints_jump_deg").as_double()))
                        {
                            RCLCPP_INFO(
                                this->get_logger(),
                                "Joint Jump! %d %f %f",
                                int(i),
                                rad2deg(nxt_plan_point[i]),
                                rad2deg(pre_sent_point[i])
                            );
                            return this->get_parameter("joints_jump_limit_rate").as_double();
                        }
                    }
                    return 1.0;
                }();
                // const auto nxt_plan_vel =
                //     mul(minus(nxt_plan_point, pre_plan_point), this->fps_aubo_consume);
                std::array<bool, 6> done;
                int need_done = 6;
                std::fill(done.begin(), done.end(), false);

                // [generate a sent point]
                while (ok_no_collide && points_needed > 0 && need_done > 0) {
                    const std::array<double, 6> joints_acc = [&]() {
                        std::array<double, 6> joints_acc = {};
                        auto sign = [](const double some) { return some > 0 ? 1 : -1; };
                        auto limited_by = [](const double some, const double abs_lim) {
                            return std::clamp(some, -abs_lim, abs_lim);
                        };
                        for (int i = 0; i < 6; i++) {
                            const double dis = nxt_plan_point[i] - pre_sent_point[i];
                            const double dis_at_next_control_if_same_v = nxt_plan_point[i]
                                + pre_sent_vel[i] / this->fps_aubo_consume - pre_sent_point[i];
                            const double half_dis = (dis + dis_at_next_control_if_same_v) / 2.0;
                            // 因突变 限制速度时，不要缩减 a_max，会停不下来
                            const double a_max = deg2rad(this->joints_deg_max_acc[i]);
                            const double v_max =
                                deg2rad(this->joints_deg_max_vel[i]) * joints_limit_rate;
                            const double best_v_at_half_dis = limited_by(
                                sign(half_dis) * std::sqrt(std::abs(half_dis) * a_max),
                                v_max
                            );
                            // 有可能 halfdis 和 dis 直接反了
                            // this is target_v
                            const double no_inverse_v =
                                std::signbit(best_v_at_half_dis) != std::signbit(dis)
                                ? (dis * this->fps_aubo_consume)
                                : best_v_at_half_dis;
                            const double acc_if_goto_best_v =
                                (no_inverse_v - pre_sent_vel[i]) * this->fps_aubo_consume;
                            const double limited_acc = limited_by(acc_if_goto_best_v, a_max);
                            joints_acc[i] = limited_acc;
                        }
                        return joints_acc;
                    }();
                    const std::array<double, 6> so_v_is = [&]() {
                        std::array<double, 6> so_v_is = {};
                        for (int i = 0; i < 6; i++) {
                            so_v_is[i] = pre_sent_vel[i] + joints_acc[i] / this->fps_aubo_consume;
                        }
                        return so_v_is;
                    }();
                    pre_sent_vel = so_v_is;
                    const std::array<double, 6> so_point_is = [&]() {
                        std::array<double, 6> so_point_is = {};
                        for (int i = 0; i < 6; i++) {
                            so_point_is[i] =
                                pre_sent_point[i] + so_v_is[i] / this->fps_aubo_consume;
                        }
                        return so_point_is;
                    }();
                    const double control_deg_eps =
                        this->get_parameter("control_deg_eps").as_double();
                    for (int i = 0; i < 6; i++) {
                        if (std::abs(so_point_is[i] - nxt_plan_point[i]) < deg2rad(control_deg_eps)
                            && !done[i]) {
                            done[i] = true;
                            need_done--;
                        }
                    }
                    // acc, use array6_to_string
                    RCLCPP_INFO(
                        this->get_logger(),
                        "joints_acc: %s",
                        array6_to_string(joints_acc).c_str()
                    );
                    RCLCPP_INFO(
                        this->get_logger(),
                        "so_v_is: %s",
                        array6_to_string(so_v_is).c_str()
                    );
                    RCLCPP_INFO(
                        this->get_logger(),
                        "so_point_is: %s",
                        array6_to_string(so_point_is).c_str()
                    );
                    if (this->collide_or_outofrange(so_point_is)) {
                        ok_no_collide = false;
                        break;
                    }
                    waypoint_vector.push_back(arr_to_waypoint(so_point_is));
                    // show sent point
                    points_needed--;
                    pre_sent_point = so_point_is;
                }
                if (need_done == 0) {
                    this->plan_point_queue.pop_front();
                }
            }
            return std::make_tuple(waypoint_vector, pre_sent_point, pre_sent_vel);
        }();
        if (waypoint_vector.empty()) {
            RCLCPP_INFO(this->get_logger(), "no points to send");
            return;
        }
        {
            // std::lock_guard<std::mutex> lock_sent(this->last_sent_aubo_point_mutex); // 只有本线程修改和读取它，不锁

            const auto now = this->now().seconds();

            this->robot_service.robotServiceSetRobotPosData2Canbus(waypoint_vector);
            {
                std::lock_guard lock(this->debug_sent_fk_mutex);
                if (this->debug_sent_fk.size()
                    >= this->get_parameter("launchtime_debug_queue_size").as_int()) {
                    this->debug_sent_fk.pop_front();
                }
                for (const auto& s: waypoint_vector) {
                    this->debug_sent_fk.push_back(s);
                }
            }

            const auto p = this->now().seconds() - now;

            this->canbus_timeq_mutex.lock();
            if (this->canbus_timeq.size() >= 10000) {
                this->canbus_timeq.pop_front();
            }
            this->canbus_timeq.push_back(p);
            this->canbus_timeq_mutex.unlock();

            std::lock_guard<std::mutex> lock_size(this->last_estimated_point_size_mutex);
            this->last_estimated_point_size += uint16_t(waypoint_vector.size());
            this->last_sent_aubo_point = pre_sent_point;
            this->last_sent_aubo_point_vel = pre_sent_vel;
            this->last_sent_aubo_point_id++; // of no use at all

            // {
            //     // [only for test]
            //     auto current_state = this->move_group_interface.getCurrentState();
            //     std::vector<double> last_reduced;
            //     for (size_t i = 0; i < 6; ++i) {
            //         last_reduced.push_back(reduced_angle(this->last_sent_aubo_point[i]));
            //     }
            //     current_state->setJointGroupPositions(this->joint_model_group, last_reduced);
            //     this->move_group_interface.setJointValueTarget(*current_state);
            //     this->move_group_interface.move();
            // }

            {
                // [publish joint states to moveit2 (for collision detecting)]
                trajectory_msgs::msg::JointTrajectory traj_msg;
                traj_msg.joint_names = this->joint_model_group->getVariableNames();
                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions = std::vector<double>(
                    this->last_sent_aubo_point.begin(),
                    this->last_sent_aubo_point.end()
                );
                point.time_from_start = rclcpp::Duration::from_seconds(0.01);
                traj_msg.points.push_back(point);
                this->joint_cmd_pub->publish(traj_msg);
            }

            // show size
            for (auto i: waypoint_vector) {
                std::string joints;
                for (size_t j = 0; j < 6; ++j) {
                    joints += std::to_string(i.jointpos[j] / M_PI * 180.0).substr(0, 7) + " ";
                }
            }
        }
    }

    uint16_t get_esti_limited_point_size() {
        if (this->check_joint_broadcaster) {
            return uint16_t(this->size_expected_mac_points - 5) * 6;
        }
        const uint16_t tcp_size = this->get_aubo_mac_size_by_another_tcp();
        const uint16_t tcp_point = tcp_size / 6 + int(tcp_size % 6 != 0);
        const uint16_t esti_point = this->get_aubo_point_size_by_estimate();
        return std::max(esti_point, tcp_point);
    }

    uint16_t get_aubo_point_size_by_estimate() {
        // [last sent]
        std::lock_guard<std::mutex> lock(this->last_estimated_point_size_mutex);
        const auto time_interval = this->now().seconds() - this->last_estimated_point_size_time;
        const int consumed = std::floor(time_interval * this->fps_aubo_consume);
        this->last_estimated_point_size =
            uint16_t(std::max(0, int(this->last_estimated_point_size) - consumed));
        this->last_estimated_point_size_time += double(consumed) / this->fps_aubo_consume;
        return this->last_estimated_point_size;
    }

    uint16_t get_aubo_mac_size_by_another_tcp() {
        std::lock_guard<std::mutex> lock(this->sub_mac_size_mutex);
        return this->sub_mac_size;
    }

    uint16_t get_aubo_mac_size_by_diagnosis() {
        // count time here
        using namespace std::chrono;

        if (this->check_joint_broadcaster) {
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

    std::tuple<tf2::Vector3, tf2::Quaternion> get_hand_pose() {
        const auto transform = this->get_transform(
            "custom",
            "tracker_upright" + to_string(this->get_parameter("vr_idx").as_int())
        );
        const tf2::Vector3 pos = { transform.getOrigin().x(),
                                   transform.getOrigin().y()
                                       + this->get_parameter("hand_y_offset").as_double(), // 左右手
                                   transform.getOrigin().z() };

        return std::make_tuple(pos, transform.getRotation());
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

    std::optional<std::array<double, 6>> get_kdl_ik(const Pose& to) {
        RCLCPP_INFO(this->get_logger(), "calling kdl ik");
        auto kinematic_state = *(this->move_group_interface.getCurrentState());
        bool found_ik = kinematic_state.setFromIK(this->joint_model_group, to, 10);
        if (!found_ik) {
            RCLCPP_ERROR(this->get_logger(), "kdl ik failed");
            return std::nullopt;
        }
        std::vector<double> joint_values;
        kinematic_state.copyJointGroupPositions(this->joint_model_group, joint_values);
        assert(joint_values.size() == 6);
        return std::array<double, 6> { joint_values[0], joint_values[1], joint_values[2],
                                       joint_values[3], joint_values[4], joint_values[5] };
    }

    // suppose urdf is loaded and links are specified
    std::optional<Points> get_moveit_trajectory(const std::array<double, 6>& from, const Pose& to) {
        { // [set start state]
            ::moveit::core::RobotStatePtr current_state =
                this->move_group_interface.getCurrentState();

            // print_t("##1");

            ::moveit::core::RobotState start_state(*current_state);
            std::vector<double> joint_group_positions;
            start_state.copyJointGroupPositions(
                current_state->getJointModelGroup(this->group_name),
                joint_group_positions
            );
            // print_t("##2");
            for (size_t i = 0; i < 6; ++i) {
                joint_group_positions[i] = from[i];
            }
            start_state.setJointGroupPositions(
                current_state->getJointModelGroup(this->group_name),
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
