#pragma once

#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "comm.h"
#include "unitree/idl/go2/LowState_.hpp"
#include "unitree/idl/go2/LowCmd_.hpp"
#include "conversion.hpp"

namespace unitree::common
{

    constexpr double PosStopF = (2.146E+9f);
    constexpr double VelStopF = (16000.0f);

    // Go2 real-robot joint order (unitree lowstate motor order).
    // Policy ONNX metadata joint_names (sim order) are remapped onto this
    // order by name in ObservationBuilder.
    inline std::vector<std::string> go2_joint_names()
    {
        return {
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    }

    class Go2RobotInterface
    {
    public:
        explicit Go2RobotInterface(const std::vector<std::string> &joint_names)
            : joint_names_(joint_names)
        {
            size_t n = joint_names_.size();
            jpos.assign(n, 0.f);
            jvel.assign(n, 0.f);
            tau.assign(n, 0.f);
            jpos_des.assign(n, 0.f);
            jvel_des.assign(n, 0.f);
            kp.assign(n, 0.f);
            kd.assign(n, 0.f);
            tau_ff.assign(n, 0.f);
        }
        /**
         * @brief 将接收到的底层状态保存到robot_interface中
         * @param state 底层状态
         * @note 底层状态中包含了机器人当前的姿态、速度、力矩等信息
        */
        void SetState(unitree_go::msg::dds_::LowState_ &state)
        {
            // imu
            const unitree_go::msg::dds_::IMUState_ &imu = state.imu_state();
            quat = imu.quaternion();
            rpy = imu.rpy();
            gyro = imu.gyroscope();
            acc = imu.accelerometer();
            UpdateProjectedGravity();

            // motor
            const std::array<unitree_go::msg::dds_::MotorState_, 20> &motor = state.motor_state();
            size_t n = std::min<size_t>(12, jpos.size());
            for (size_t i = 0; i < n; ++i)
            {
                const unitree_go::msg::dds_::MotorState_ &m = motor.at(i);
                jpos.at(i) = m.q();
                jvel.at(i) = m.dq();
                tau.at(i) = m.tau_est();
            }
        }

        const std::vector<std::string> &joint_names() const { return joint_names_; }

        // observed state (real robot order)
        std::vector<float> jpos, jvel, tau;
        std::array<float, 4> quat;
        std::array<float, 3> rpy, gyro, projected_gravity, acc;

        // desired state (real robot order)
        std::vector<float> jpos_des, jvel_des, kp, kd, tau_ff;

    private:
        std::vector<std::string> joint_names_;

        inline void UpdateProjectedGravity()
        {
            // inverse quat
            float w = quat.at(0);
            float x = -quat.at(1);
            float y = -quat.at(2);
            float z = -quat.at(3);

            float x2 = x * x;
            float y2 = y * y;
            float z2 = z * z;
            float w2 = w * w;
            float xy = x * y;
            float xz = x * z;
            float yz = y * z;
            float wx = w * x;
            float wy = w * y;
            float wz = w * z;

            projected_gravity.at(0) = -2 * (xz + wy);
            projected_gravity.at(1) = -2 * (yz - wx);
            projected_gravity.at(2) = -(w2 - x2 - y2 + z2);
        }
    };
} // namespace unitree::common
