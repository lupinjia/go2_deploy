#pragma once

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <filesystem>
#include <mutex>
#include <fstream>
#include <future>
#include <algorithm>
#include <sstream>

#include "unitree/idl/go2/LowState_.hpp"
#include "unitree/idl/go2/LowCmd_.hpp"
#include "unitree/common/thread/thread.hpp"

#include "unitree/robot/channel/channel_publisher.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/common/time/time_tool.hpp"
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

#include "state_machine.hpp"
#include "gamepad.hpp"
#include "robot_interface.hpp"
#include "user_controller.hpp"
#include "depth_image_subscriber.hpp"
#include "parkour_controller.hpp"

#include <opencv2/opencv.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;
namespace fs = std::filesystem;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

// RobotController为一个框架, 将有限状态机整体的逻辑具象化为一个类
class RobotController
{
public:
    RobotController(BasicUserController* ctrl) : ctrl(ctrl) {}

    RobotController(fs::path &log_file_name, BasicUserController* ctrl) : ctrl(ctrl)
    {
        // set log file
        log_file = std::ofstream(log_file_name);
        std::stringstream header;
        header << "state,";
        header << "cmd_lin_x,cmd_lin_y,cmd_ang_z,";
        header << std::endl;
        log_file << header.str();
    }

    void loadParam()
    {
        config_path_ = (fs::current_path() / "../params/go2_config.yaml").string();
        ctrl->loadParam();
    }

    void loadPolicy()
    {
        ctrl->loadPolicy();
    }

    void initRobotStateClient()
    {
        rsc.SetTimeout(10.0f); 
        rsc.Init();
    }

    void activateService(const std::string& serviceName, int activate, int& serviceStatus)
    {
        rsc.ServiceSwitch(serviceName, activate, serviceStatus);
    }

    void InitDdsModel()
    {
        // init dds
        lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));

        lowcmd_publisher->InitChannel();
        lowstate_subscriber->InitChannel(std::bind(&RobotController::LowStateMessageHandler, this, std::placeholders::_1), 1);

        // depth image subscription (parkour); domain/interface come from config
        // (defaults: sim2sim domain 1 / "lo"; real robot domain 0 / eth0)
        int depth_domain = 1;
        std::string depth_interface = "lo";
        bool depth_enabled = true;
        try
        {
            auto cfg = YAML::LoadFile(config_path_);
            depth_enabled = cfg["depth_enabled"].as<bool>(depth_enabled);
            depth_domain = cfg["depth_domain"].as<int>(depth_domain);
            depth_interface = cfg["depth_interface"].as<std::string>(depth_interface);
            depth_visualize_ = cfg["depth_visualize"].as<bool>(false);
        }
        catch (...) {}
        if (depth_enabled)
        {
            depth_subscriber_.reset(new DepthImageSubscriber(depth_domain, depth_interface));
        }
    }

    void StartControl()
    {
        // waiting for gamepad command to start the control thread
        std::chrono::milliseconds duration(100); // 100ms
        // init low-level command
        init_low_cmd();
        // listen to gamepad command
        std::cout << "Press START button to start!" << std::endl;
        while (true)
        {
            std::this_thread::sleep_for(duration);

            InteprateGamePad();
            if (gamepad.start.on_press)
            {
                break;
            }
        }

        // prepare for start
        std::cout << "Start!" << std::endl;
        Damping();
        ctrl_dt_micro_sec = static_cast<uint64_t>(ctrl->dt * 1000000); // 0.02s, 50Hz

        // Start the control thread
        control_thread_ptr = CreateRecurrentThreadEx("ctrl", UT_CPU_ID_NONE, ctrl_dt_micro_sec, &RobotController::ControlStep, this);

        // Start the lowlevel command thread
        std::this_thread::sleep_for(duration);
        StartSendCmd();
        std::cout << "Start Send Cmd!" << std::endl;

        // keep the main thread alive
        while (true)
        {
            std::this_thread::sleep_for(duration);
        }
    }

protected:
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ThreadPtr low_cmd_write_thread_ptr, control_thread_ptr;
    unitree_go::msg::dds_::LowCmd_ cmd;
    unitree_go::msg::dds_::LowState_ state;

    Gamepad gamepad;
    REMOTE_DATA_RX rx;

    RLStateMachine state_machine;
    BasicUserController* ctrl;

    std::mutex state_mutex, cmd_mutex;

    std::ofstream log_file;

    uint64_t ctrl_dt_micro_sec = 2000;

    std::vector<float> compute_time;

    RobotStateClient rsc;

    std::string config_path_;
    std::unique_ptr<DepthImageSubscriber> depth_subscriber_;
    bool depth_visualize_ = false;

private:
        
    // 500Hz
    void LowStateMessageHandler(const void *message)
    {
        state = *(unitree_go::msg::dds_::LowState_ *)message;
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            ctrl->robot().SetState(state);
        }
    }

    void InteprateGamePad()
    {
        // update gamepad
        memcpy(rx.buff, &state.wireless_remote()[0], 40);
        gamepad.update(rx.RF_RX);
    }

    // 500Hz
    void LowCmdwriteHandler()
    {
        // write low-level command
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            lowcmd_publisher->Write(cmd);
        }
        
    }

    void init_low_cmd()
    {
        cmd.head()[0] = 0xFE;
        cmd.head()[1] = 0xEF;
        cmd.level_flag() = 0xFF;
        cmd.gpio() = 0;

        for(int i=0; i<20; i++)
        {
            cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
            cmd.motor_cmd()[i].q() = (PosStopF);
            cmd.motor_cmd()[i].kp() = (0);
            cmd.motor_cmd()[i].dq() = (VelStopF);
            cmd.motor_cmd()[i].kd() = (0);
            cmd.motor_cmd()[i].tau() = (0);
        }
    }

    void set_cmd()
    {
        auto &robot = ctrl->robot();
        size_t n = robot.jpos_des.size();
        for(size_t i = 0; i < n; i++)
        {
            cmd.motor_cmd()[i].q() = robot.jpos_des.at(i);
            cmd.motor_cmd()[i].dq() = robot.jvel_des.at(i);
            cmd.motor_cmd()[i].kp() = robot.kp.at(i);
            cmd.motor_cmd()[i].kd() = robot.kd.at(i);
            cmd.motor_cmd()[i].tau() = robot.tau_ff.at(i);
        }
        cmd.crc() = crc32_core((uint32_t *)&cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    }

    void StartSendCmd()
    {
        low_cmd_write_thread_ptr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &RobotController::LowCmdwriteHandler, this);
    }

   void UpdateStateMachine()
    {
        // L1 + R1 -> Sit
        // L1 + R2 -> Stand
        // L1 + A -> Ctrl
        // L1 + Y -> Stop
        if(gamepad.R1.on_press && gamepad.L1.pressed)
        {
            if(state_machine.Sit())
            {
                SitCallback();
            }
        }
        if (gamepad.R2.on_press && gamepad.L1.pressed)
        {
            if (state_machine.Stand()) // 进入站立状态
            {
                StandCallback();
            }
        }
        if (gamepad.A.on_press && gamepad.L1.pressed)
        {
            if (state_machine.Ctrl())
            {
                CtrlCallback();
            }
        }
        if (gamepad.Y.pressed && gamepad.L1.pressed)
        {
            state_machine.Stop();
        }
        if (gamepad.X.on_press && gamepad.L1.pressed)
        {
            if (state_machine.Parkour())
            {
                ParkourCallback();
            }
        }
    }

    // 50Hz
    void ControlStep()
    {
        // main loop

        // update state
        InteprateGamePad(); // 接收数据是500Hz频率, 但是我只有运行主控制线程时才需要处理好的gamepad数据
                            // 所以可以在主控制程序中处理gamepad数据
        UpdateStateMachine();

        // select control modes according to the state machine
        auto start = std::chrono::high_resolution_clock::now();
        // 读取状态信息
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            ctrl->GetInput(ctrl->robot(), gamepad);
        }
        // 根据状态判断具体执行哪个状态下的处理函数
        if(state_machine.state == STATES::SIT)
        {
            Sitting();
        }
        if (state_machine.state == STATES::STAND)
        {
            Standing();
        }
        if (state_machine.state == STATES::DAMPING)
        {
            Damping();
        }
        if (state_machine.state == STATES::CTRL)
        {
            UserControlStep(true);
            if (CheckTermination())
            {
                state_machine.Stop();
                Damping();
            }
        }
        if (state_machine.state == STATES::PARKOUR)
        {
            std::vector<float> depth;
            uint16_t dw = 0, dh = 0;
            if (depth_subscriber_ && depth_subscriber_->latest_frame(depth, dw, dh))
            {
                ctrl->set_depth_frame(depth.data(), depth.size());
                if (depth_visualize_ && dw > 0 && dh > 0)
                {
                    cv::Mat depth_raw(dh, dw, CV_32FC1, depth.data());
                    cv::Mat depth_vis;
                    depth_raw.convertTo(depth_vis, CV_8UC1, 255.0);
                    cv::resize(depth_vis, depth_vis, cv::Size(dw * 10, dh * 10), 0, 0, cv::INTER_NEAREST);
                    cv::imshow("parkour depth", depth_vis);
                    cv::waitKey(1);
                }
            }
            UserControlStep(true);
            if (CheckTermination())
            {
                state_machine.Stop();
                Damping();
            }
        }

        // update low-level command
        {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            set_cmd();
        }

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        compute_time.push_back(duration.count() / 1000.); // us->ms

        // write log
        WriteLog();

        if (compute_time.size() == 50)
        {
            float sum = 0;
            for (auto &t : compute_time)
            {
                sum += t;
            }
            std::cout << "Performance: mean: " << sum / 50
                      << " ms; max: " << *std::max_element(compute_time.begin(), compute_time.end())
                      << " ms; min: " << *std::min_element(compute_time.begin(), compute_time.end())
                      << "ms." << std::endl;

            compute_time.clear();

            std::cout << "Current State: " << static_cast<size_t>(state_machine.state) << std::endl;
        }
    }

    void WriteLog()
    {
        if (log_file.is_open())
        {
            log_file << static_cast<size_t>(state_machine.state) << ",";
            auto log = ctrl->GetLog();
            for (const auto &v : log)
            {
                log_file << v << ",";
            }
            log_file << std::endl;
        }
    }

    void SitCallback()
    {
        // 保存初始关节位置
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            ctrl->save_jpos(ctrl->robot());
        }
        auto &robot = ctrl->robot();
        robot.jpos_des = ctrl->start_pos; // 由阻尼状态的初始位置开始
        std::fill(robot.jvel_des.begin(), robot.jvel_des.end(), 0.f);
        std::fill(robot.kp.begin(), robot.kp.end(), ctrl->stand_kp());
        std::fill(robot.kd.begin(), robot.kd.end(), ctrl->stand_kd());
        std::fill(robot.tau_ff.begin(), robot.tau_ff.end(), 0.f);
    }
    /**
     * @brief 站立状态回调函数
     * @note  进入状态时调用一次, 用于进入站立状态的初始化工作
    */
    void StandCallback()
    {
        // 保存初始关节位置
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            ctrl->save_jpos(ctrl->robot());
        }
        auto &robot = ctrl->robot();
        robot.jpos_des = ctrl->start_pos; // 由阻尼状态的初始位置开始
        std::fill(robot.jvel_des.begin(), robot.jvel_des.end(), 0.f);
        std::fill(robot.kp.begin(), robot.kp.end(), ctrl->stand_kp());
        std::fill(robot.kd.begin(), robot.kd.end(), ctrl->stand_kd());
        std::fill(robot.tau_ff.begin(), robot.tau_ff.end(), 0.f);
    }

    void CtrlCallback()
    {
        // 重置状态
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            ctrl->reset(ctrl->robot(), gamepad);
        }

        auto &robot = ctrl->robot();
        robot.jpos_des = ctrl->stand_pos_real();
        std::fill(robot.jvel_des.begin(), robot.jvel_des.end(), 0.f);
        robot.kp = ctrl->kp();  // metadata per-joint gains
        robot.kd = ctrl->kd();
        std::fill(robot.tau_ff.begin(), robot.tau_ff.end(), 0.f);
        std::cout << "Ctrl enabled." << std::endl;
    }

    void ParkourCallback()
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            ctrl->reset(ctrl->robot(), gamepad);
        }

        auto &robot = ctrl->robot();
        robot.jpos_des = ctrl->stand_pos_real();
        std::fill(robot.jvel_des.begin(), robot.jvel_des.end(), 0.f);
        robot.kp = ctrl->kp();
        robot.kd = ctrl->kd();
        std::fill(robot.tau_ff.begin(), robot.tau_ff.end(), 0.f);
        std::cout << "Parkour enabled." << std::endl;
    }

    void Damping(float kd = 3.0)
    {
        auto &robot = ctrl->robot();
        std::fill(robot.jpos_des.begin(), robot.jpos_des.end(), 0.f);
        std::fill(robot.jvel_des.begin(), robot.jvel_des.end(), 0.f);
        std::fill(robot.kp.begin(), robot.kp.end(), 0.f);
        std::fill(robot.kd.begin(), robot.kd.end(), kd);
        std::fill(robot.tau_ff.begin(), robot.tau_ff.end(), 0.f);
    }

    void Sitting(float kp = 60.0, float kd = 5.0)
    {
        state_machine.Sitting(*ctrl);

        ctrl->robot().jpos_des = ctrl->jpos_des;
    }

    void Standing(float kp = 60.0, float kd = 5.0)
    {
        ctrl->DummyCalculate(); // warmup the neural network

        state_machine.Standing(*ctrl);

        ctrl->robot().jpos_des = ctrl->jpos_des;
    }

    void UserControlStep(bool send_cmd = false)
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex);
            ctrl->Calculate();
        }

        if (send_cmd)
        {
            ctrl->robot().jpos_des = ctrl->jpos_des;
        }
        
    }

    bool CheckTermination()
    {
        // if (robot_interface.rpy.at(0) > M_PI/3)
        // {
        //     return true;
        // }
        // if(max_abs(ctrl->jpos_des) > 3*M_PI/2)
        // {
        //     return true;
        // }
        return false;
    }
};
