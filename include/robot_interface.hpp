#pragma once

#include <array>
#include <iostream>
#include "common/mathTools.h"

#include "comm.h"
#include "unitree/idl/go2/LowState_.hpp"
#include "unitree/idl/go2/LowCmd_.hpp"
#include "conversion.hpp"

namespace unitree::common
{

    constexpr double PosStopF = (2.146E+9f);
    constexpr double VelStopF = (16000.0f);

    class BasicRobotInterface
    {
    public:
        BasicRobotInterface()
        {
            jpos_des.fill(0.);
            jvel_des.fill(0.);
            kp.fill(0.);
            kd.fill(0.);
            tau_ff.fill(0.);
            projected_gravity.fill(0.);
            projected_gravity.at(2) = -1.0;
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
            for (size_t i = 0; i < 12; ++i)
            {
                const unitree_go::msg::dds_::MotorState_ &m = motor.at(i);
                jpos.at(i) = m.q();
                jvel.at(i) = m.dq();
                tau.at(i) = m.tau_est();
            }
        }

        std::array<float, 12> jpos, jvel, tau;
        std::array<float, 4> quat;
        std::array<float, 3> rpy, gyro, projected_gravity, acc;
        std::array<float, 12> jpos_des, jvel_des, kp, kd, tau_ff;

    private:
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