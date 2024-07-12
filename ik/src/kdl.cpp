// #include "tf_test/tf_test.hpp"
// C++ system
#include <cmath>
#include <memory>
#include <string>

#include <AuboRobotMetaType.h>
#include <geometry_msgs/msg/pose.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serviceinterface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
// #include <serviceinterface.h> // local
// #include <Poco/Net/ServerSocket.h> // system

namespace ik::kdl {
using std::string;

constexpr float FPS = 50;
const char* HOST = "192.168.38.128";
const int PORT = 8899;

void print_joint_status(const aubo_robot_namespace::JointStatus* jointStatus, int len) {
    std::cout << std::endl << "start----------关节状态信息-------" << std::endl;

    for (int i = 0; i < len; i++) {
        std::cout << "关节ID:" << i << "  ";
        std::cout << "电流:" << jointStatus[i].jointCurrentI << " ";
        std::cout << "速度:" << jointStatus[i].jointSpeedMoto << " ";
        std::cout << "关节角:" << jointStatus[i].jointPosJ << " ";
        std::cout << "电压   :" << jointStatus[i].jointCurVol << " ";
        std::cout << "温度   :" << jointStatus[i].jointCurTemp << " ";
        std::cout << "目标电流:" << jointStatus[i].jointTagCurrentI << " ";
        std::cout << "目标电机速度:" << jointStatus[i].jointTagSpeedMoto << " ";
        std::cout << "目标关节角 :" << jointStatus[i].jointTagPosJ << " ";
        std::cout << "关节错误   :" << jointStatus[i].jointErrorNum << std::endl;
    }
    std::cout << std::endl;
}

std::array<double, 6> get_jnt_ang_from_status(const aubo_robot_namespace::JointStatus* jointStatus
) {
    std::array<double, 6> jnt_ang;
    for (int i = 0; i < 6; i++) {
        jnt_ang[i] = jointStatus[i].jointPosJ;
    }
    return jnt_ang;
}

KDL::JntArray jnt_arr_from_arr(const std::array<double, 6>& arr) {
    KDL::JntArray jnt_arr(6);
    for (int i = 0; i < 6; i++) {
        jnt_arr(i) = arr[i];
    }
    return jnt_arr;
}

class Ik: public rclcpp::Node {
private:
    urdf::Model urdf_model;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;

    // rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    ServiceInterface robot_service;
    rclcpp::TimerBase::SharedPtr timer;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

public:
    explicit Ik(const rclcpp::NodeOptions& options):
        Node("ik_kdl", options),
        timer(this->create_wall_timer(
            std::chrono::duration<float>(1 / FPS),
            [this] { this->fast_control(); }
        )),
        tf_buffer(this->get_clock()),
        tf_listener(this->tf_buffer) // 别缩到这
    {
        RCLCPP_INFO(this->get_logger(), "This is Ik");

        // [robot init]
        int ret = this->robot_service.robotServiceLogin(HOST, PORT, "aubo", "123456");
        if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
            RCLCPP_INFO(this->get_logger(), "login success.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "login failed");
        }

        this->robot_service.robotServiceEnterTcp2CanbusMode();
        this->robot_service.robotServiceInitGlobalMoveProfile();

        // [kdl init]
        std::string urdf_file = "/home/julyfun/ros_ws/src/aubo/urdf/aubo_i5.urdf";
        this->urdf_model.initFile(urdf_file);
        if (!kdl_parser::treeFromUrdfModel(this->urdf_model, kdl_tree_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
            return;
        }

        // Extract the chain from the KDL tree
        std::string base_link = "base_link";
        std::string end_effector_link = "wrist3_Link";
        if (!kdl_tree_.getChain(base_link, end_effector_link, kdl_chain)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain");
            return;
        }

        // Initialize solvers
        fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain);
        ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(kdl_chain);
        // const double qmin[] = { -3.14, -3.14, -3.14, -3.14, -3.14, -3.14 };
        const auto qmin = jnt_arr_from_arr({ -3.14, -3.14, -3.14, -3.14, -3.14, -3.14 });
        const auto qmax = jnt_arr_from_arr({ 3.14, 3.14, 3.14, 3.14, 3.14, 3.14 });
        ik_pos_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(
            kdl_chain,
            qmin,
            qmax,
            *fk_solver_,
            *ik_vel_solver_
        );
    };
    ~Ik() override {
        this->robot_service.robotServiceLeaveTcp2CanbusMode();
    };

private:
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

    void fast_control() {
        // 坐标系的转换和点的转换是反的
        tf2::Transform transform;
        try {
            transform = this->get_transform("custom", "tracker_upright");
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", e.what());
            return;
        }

        aubo_robot_namespace::JointStatus joint_status[6];
        int ret = this->robot_service.robotServiceGetRobotJointStatus(joint_status, 6);
        if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
            // print_joint_status(joint_status, 6);
        } else {
            std::cerr << "ERROR:获取关节状态失败."
                      << "ret:" << ret << std::endl;
        }
        const auto jnt_ang = get_jnt_ang_from_status(joint_status);

        // this->robot_service.robotServiceInitGlobalMoveProfile();

        // /** 接口调用: 设置关节型运动的最大加速度 ***/
        // aubo_robot_namespace::JointVelcAccParam joint_max_acc;
        // joint_max_acc.jointPara[0] = 50.0 / 180.0 * M_PI;
        // joint_max_acc.jointPara[1] = 50.0 / 180.0 * M_PI;
        // joint_max_acc.jointPara[2] = 50.0 / 180.0 * M_PI;
        // joint_max_acc.jointPara[3] = 50.0 / 180.0 * M_PI;
        // joint_max_acc.jointPara[4] = 50.0 / 180.0 * M_PI;
        // joint_max_acc.jointPara[5] = 50.0 / 180.0 * M_PI; //接口要求单位是弧度
        // this->robot_service.robotServiceSetGlobalMoveJointMaxAcc(joint_max_acc);

        // /** 接口调用: 设置关节型运动的最大速度 ***/
        // aubo_robot_namespace::JointVelcAccParam joint_max_velc;
        // joint_max_velc.jointPara[0] = 50.0 / 180.0 * M_PI;
        // joint_max_velc.jointPara[1] = 50.0 / 180.0 * M_PI;
        // joint_max_velc.jointPara[2] = 50.0 / 180.0 * M_PI;
        // joint_max_velc.jointPara[3] = 50.0 / 180.0 * M_PI;
        // joint_max_velc.jointPara[4] = 50.0 / 180.0 * M_PI;
        // joint_max_velc.jointPara[5] = 50.0 / 180.0 * M_PI; //接口要求单位是弧度
        // this->robot_service.robotServiceSetGlobalMoveJointMaxVelc(joint_max_velc);

        // /** 接口调用: 初始化运动属性 ***/
        // this->robot_service.robotServiceInitGlobalMoveProfile();

        // /** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
        // double end_move_max_acc;
        // end_move_max_acc = 0.2; //单位米每秒
        // robot_service.robotServiceSetGlobalMoveEndMaxLineAcc(end_move_max_acc);
        // robot_service.robotServiceSetGlobalMoveEndMaxAngleAcc(end_move_max_acc);

        // /** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动***/
        // double end_move_max_velc;
        // end_move_max_velc = 0.2; //单位米每秒
        // robot_service.robotServiceSetGlobalMoveEndMaxLineVelc(end_move_max_velc);
        // robot_service.robotServiceSetGlobalMoveEndMaxAngleVelc(end_move_max_velc);

        // robot_service.robotServiceSetGlobalCircularLoopTimes(1); //圆的圈数

        // [kdl solve]
        auto position = transform.getOrigin();
        auto orientation = transform.getRotation();

        struct Out {
            double base;
            double min;
            double max;
        };
        const auto out_pos = [](const tf2::Vector3& position,
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
        const auto out_position =
            out_pos(position, { 0.4, 0.4, 1.8 }, { 0, -1.8, 1.8 }, { 0.1, 0.05, 1.8 }, 1.0);

        KDL::Frame target_frame;
        target_frame.p = KDL::Vector(out_position.x(), out_position.y(), out_position.z());

        target_frame.M = KDL::Rotation::Quaternion(
            orientation.x(),
            orientation.y(),
            orientation.z(),
            orientation.w()
        );

        // [kdl ik]
        // #qm should that be deg or rad?
        KDL::JntArray sol_joints(kdl_chain.getNrOfJoints());
        // = {-3.607505, 0.341427, -2.684385, 0.115326, 1.106423, 1.570813};
        KDL::JntArray init_joints(kdl_chain.getNrOfJoints()); // Initial guess for the solver
        for (int i = 0; i < 6; i++) {
            init_joints(i) = jnt_ang[i];
        }

        for (int i = 0; i < 3; i++) {
            RCLCPP_INFO(this->get_logger(), "target_frame.p[%d]: %f", i, target_frame.p(i));
        }
        // {
        //     double x, y, z, w;
        //     target_frame.M.GetQuaternion(x, y, z, w);
        //     RCLCPP_INFO(this->get_logger(), "target_frame.q: %f %f %f %f", x, y, z, w);
        // }
        int ik_ret = ik_pos_solver_->CartToJnt(init_joints, target_frame, sol_joints);
        if (ik_ret >= 0) {
        } else {
            RCLCPP_ERROR(this->get_logger(), "IK solver failed");
        }

        // [internal ik]
        const double initial_positions_arr[] = { -3.607505, 0.341427, -2.684385,
                                                 0.115326,  1.106423, 1.570813 };
        aubo_robot_namespace::wayPoint_S internal_sol;
        this->robot_service.robotServiceRobotIk(
            initial_positions_arr,
            { out_position.x(), out_position.y(), out_position.z() },
            { orientation.w(), orientation.x(), orientation.y(), orientation.z() },
            internal_sol
        );
        for (int i = 0; i < 6; i++) {
            RCLCPP_INFO(
                this->get_logger(),
                "internal_sol.jointpos[%d]: %f",
                i,
                internal_sol.jointpos[i] / M_PI * 180.0
            );
        }

        // [show forward kin, is it correct?]
        {
            decltype(target_frame) out_frame;
            // internal?
            KDL::JntArray sol_joints_internal(kdl_chain.getNrOfJoints());
            for (int i = 0; i < 6; i++) {
                sol_joints_internal(i) = internal_sol.jointpos[i];
            }
            this->fk_solver_->JntToCart(sol_joints_internal, out_frame);
            for (int i = 0; i < 3; i++) {
                RCLCPP_INFO(this->get_logger(), "out_frame.p[%d]: %f", i, out_frame.p(i));
            }

            double x, y, z, w;
            out_frame.M.GetQuaternion(x, y, z, w);
            RCLCPP_INFO(this->get_logger(), "out_frame.q: %f %f %f %f", x, y, z, w);
        }
        // [send to canbus]
        ret = this->robot_service.robotServiceSetRobotPosData2Canbus(sol_joints.data.data());

        {
            for (int i = 0; i < 6; i++) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "sol_joints[%d]: %f",
                    i,
                    sol_joints(i) / M_PI * 180.0
                );
            }
        }
        if (ret != aubo_robot_namespace::InterfaceCallSuccCode) {
            const auto err_desc =
                this->robot_service.getErrDescByCode(aubo_robot_namespace::RobotErrorCode(ret));
            RCLCPP_ERROR(
                this->get_logger(),
                "robotServiceSetRobotPosData2Canbus failed: %s",
                err_desc.c_str()
            );
        }
    }

    void try_some() {
        // [begin]
        // tf2::Vector3 position = { 0.5, 0.3, 0.2 };
        // tf2::Quaternion orientation(0, 0, 0, 1);

        // [end]

        // Subscriber for target pose
        // target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        //     "target_pose",
        //     10,
        //     std::bind(&Ik::target_pose_callback, this, std::placeholders::_1)
        // );

        // // Publisher for joint states
        // joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }
    // void target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    //     KDL::Frame target_frame;
    //     target_frame.p = KDL::Vector(msg->position.x, msg->position.y, msg->position.z);
    //     target_frame.M = KDL::Rotation::Quaternion(
    //         msg->orientation.x,
    //         msg->orientation.y,
    //         msg->orientation.z,
    //         msg->orientation.w
    //     );

    //     KDL::JntArray joint_positions(kdl_chain_.getNrOfJoints());
    //     KDL::JntArray initial_positions(kdl_chain_.getNrOfJoints()); // Initial guess for the solver

    //     // Solve IK
    //     int ret = ik_pos_solver_->CartToJnt(initial_positions, target_frame, joint_positions);
    //     if (ret >= 0) {
    //         publishJointStates(joint_positions);
    //     } else {
    //         RCLCPP_ERROR(this->get_logger(), "IK solver failed");
    //     }
    // }

    // void publishJointStates(const KDL::JntArray& joint_positions) {
    //     auto joint_state_msg = sensor_msgs::msg::JointState();
    //     joint_state_msg.header.stamp = this->now();
    //     joint_state_msg.name = { "joint1", "joint2", "joint3", "joint4", "joint5", "joint6" };
    //     joint_state_msg.position.resize(joint_positions.rows());
    //     for (unsigned int i = 0; i < joint_positions.rows(); ++i) {
    //         joint_state_msg.position[i] = joint_positions(i);
    //     }
    //     joint_state_pub_->publish(joint_state_msg);
    // }
};

} // namespace ik::kdl

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ik::kdl::Ik)
