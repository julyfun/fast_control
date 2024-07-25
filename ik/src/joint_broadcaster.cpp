/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// #include "joint_broadcaster/joint_listener.h"
// #include <joint_broadcaster/MacTargetPosDataSize.h>
// #include </JointState.h>
#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <std_msgs/msg/detail/int16__struct.hpp>
#include <std_msgs/msg/int16.hpp>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

namespace ik::joint_broadcaster {

std::string tcp_ip = "192.168.1.7";
int tcp_port = 8891;

class JointBroadcaster: public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr mac_target_pos_data_size_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
    int sock;

public:
    explicit JointBroadcaster(const rclcpp::NodeOptions& options):
        Node("aubo_state_broadcaster", options) {
        RCLCPP_INFO(this->get_logger(), "hello");
        std::thread(&JointBroadcaster::call_me_once, this).detach();
    };

    ~JointBroadcaster() override {};

private:
    void call_me_once() {
        this->mac_target_pos_data_size_pub =
            this->create_publisher<std_msgs::msg::Int16>("/mac_target_pos_data_size", 10);
        // this->joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
        this->joint_pub =
            this->create_publisher<sensor_msgs::msg::JointState>("/aubo_joint_states", 10);

        int sock = this->connectToServer(tcp_ip, tcp_port);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to server.");
            return;
        }

        char buffer[1550] = { 0 };
        std::string accumulated_data;
        while (rclcpp::ok()) {
            int valread = read(sock, buffer, sizeof(buffer));
            bool success = (valread > 0);
            if (success) {
                accumulated_data.append(buffer, valread);

                size_t start_pos = accumulated_data.find("<PACK_BEGIN");
                size_t end_pos = accumulated_data.find("PACK_END>");

                while (start_pos != std::string::npos && end_pos != std::string::npos) {
                    if (end_pos + 8 > accumulated_data.size()) {
                        break;
                    }

                    std::string complete_data =
                        accumulated_data.substr(start_pos, end_pos - start_pos + 9);
                    accumulated_data.erase(0, end_pos + 8);

                    std::vector<std::string> joint_names;
                    std::vector<double> joint_positions;
                    int mac_target_pos_data_size = 0;

                    // RCLCPP_INFO(this->get_logger(), "Received data: %s", complete_data.c_str());
                    this->parseJointData(
                        complete_data,
                        joint_names,
                        joint_positions,
                        mac_target_pos_data_size
                    );

                    if (!joint_names.empty() && !joint_positions.empty()) {
                        sensor_msgs::msg::JointState joint_state;
                        joint_state.header.stamp = this->now();
                        joint_state.name = joint_names;
                        joint_state.position = joint_positions;
                        this->joint_pub->publish(joint_state);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "No joint data parsed from received data.");
                    }

                    std_msgs::msg::Int16 mac_target_pos_data_size_msg;
                    mac_target_pos_data_size_msg.data = mac_target_pos_data_size;
                    this->mac_target_pos_data_size_pub->publish(mac_target_pos_data_size_msg);

                    RCLCPP_INFO(
                        this->get_logger(),
                        "Published macTargetPosDataSize: %d, success: %s",
                        mac_target_pos_data_size,
                        success ? "true" : "false"
                    );
                    start_pos = accumulated_data.find("<PACK_BEGIN");
                    end_pos = accumulated_data.find("PACK_END>");
                }
            } else {
                std_msgs::msg::Int16 mac_target_pos_data_size_msg;
                mac_target_pos_data_size_msg.data = 0;
                this->mac_target_pos_data_size_pub->publish(mac_target_pos_data_size_msg);

                RCLCPP_WARN(this->get_logger(), "Failed to read data from socket. Reconnecting...");
                close(sock);
                sock = connectToServer(tcp_ip, tcp_port);
                while (sock < 0 && rclcpp::ok()) {
                    rclcpp::sleep_for(std::chrono::seconds(1));
                    sock = connectToServer(tcp_ip, tcp_port);
                }
            }
            memset(buffer, 0, sizeof(buffer));
        }
        close(this->sock);
    }

    void parseJointData(
        const std::string& data,
        std::vector<std::string>& joint_names,
        std::vector<double>& joint_positions,
        int& macTargetPosDataSize
    ) {
        //RCLCPP_INFO(this->get_logger(), "Received raw data: %s", data.c_str());

        const std::string PACK_BEGIN = "<PACK_BEGIN";
        const std::string PACK_END = "PACK_END>";

        size_t start_pos = data.find(PACK_BEGIN);
        size_t end_pos = data.find(PACK_END);

        if (start_pos == std::string::npos || end_pos == std::string::npos || start_pos >= end_pos)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid data format. start_pos: %zu, end_pos: %zu",
                start_pos,
                end_pos
            );
            return;
        }

        start_pos += PACK_BEGIN.length();
        size_t length_pos = start_pos;
        start_pos += 8;

        std::string length_str = data.substr(length_pos, 8);
        size_t data_length = std::stoul(length_str);
        if (end_pos - start_pos != data_length) {
            RCLCPP_WARN(
                this->get_logger(),
                "Data length mismatch. Expected: %zu, Actual: %zu",
                data_length,
                end_pos - start_pos
            );
            return;
        }

        std::string json_str = data.substr(start_pos, data_length);
        //RCLCPP_INFO(this->get_logger(), "Extracted JSON string: %s", json_str.c_str());

        Json::Reader reader;
        Json::Value root;

        if (reader.parse(json_str, root)) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Parsed JSON successfully.");

            const Json::Value jointPos = root["RobotJointStatus"]["jointPos"];
            if (!jointPos.isNull() && jointPos.isArray()) {
                std::vector<std::string> joint_names_temp = { "shoulder_joint", "upperArm_joint",
                                                              "foreArm_joint",  "wrist1_joint",
                                                              "wrist2_joint",   "wrist3_joint" };
                for (Json::Value::ArrayIndex i = 0; i < jointPos.size(); ++i) {
                    joint_positions.push_back(jointPos[i].asDouble());
                    joint_names.push_back(joint_names_temp[i]);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to find jointPos in JSON data.");
            }

            if (root["RobotDiagnosis"].isMember("macTargetPosDataSize")) {
                macTargetPosDataSize = root["RobotDiagnosis"]["macTargetPosDataSize"].asInt();
                RCLCPP_INFO_ONCE(
                    this->get_logger(),
                    "macTargetPosDataSize: %d",
                    macTargetPosDataSize
                );
            } else {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Failed to find macTargetPosDataSize in JSON data."
                );
            }
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to parse JSON data. Error: %s",
                reader.getFormattedErrorMessages().c_str()
            );
        }
    }

    int connectToServer(const std::string& ip, int port) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation error");
            return -1;
        }

        struct sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);

        if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid address/ Address not supported");
            close(sock);
            return -1;
        }

        if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection Failed");
            close(sock);
            return -1;
        }

        RCLCPP_INFO_ONCE(this->get_logger(), "Connected to server at %s:%d", ip.c_str(), port);
        return sock;
    }
};

} // namespace ik::joint_broadcaster
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ik::joint_broadcaster::JointBroadcaster)
