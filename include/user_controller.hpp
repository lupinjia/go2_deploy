#pragma once

#include <array>
#include <vector>
#include <filesystem>
#include <fstream>
#include <string>
#include <deque>

#include "torch/script.h"
#include "robot_interface.hpp"
#include "gamepad.hpp"
#include "param/cfg.hpp"

namespace fs = std::filesystem;

namespace unitree::common
{
    class BasicUserController
    {
    public:
        BasicUserController() {}

        virtual void loadParam() = 0;

        virtual void loadPolicy() = 0;

        virtual void reset(BasicRobotInterface &robot_interface, Gamepad &gamepad) = 0;

        virtual void GetInput(BasicRobotInterface &robot_interface, Gamepad &gamepad) = 0;

        virtual void DummyCalculate() = 0;

        virtual void Calculate() = 0;

        virtual std::vector<float> GetLog() = 0;

        void save_jpos(BasicRobotInterface &robot_interface)
        {
            std::copy(robot_interface.jpos.begin(), robot_interface.jpos.end(), start_pos.begin());
        }

        float dt;
        float stand_kp;
        float stand_kd;
        float ctrl_kp;
        float ctrl_kd;
        std::array<float, 12> start_pos; // 阻尼状态的位置
        std::array<float, 12> stand_pos; // 站立状态最终位置
        std::array<float, 12> jpos_des;
        std::array<float, 12> sit_pos; // sit状态最终位置
    };


class SimpleRLController : public BasicUserController
{
public:
    SimpleRLController(const std::string& cfg_file)
    {
        // observation init to 0
        base_ang_vel.fill(0.0);
        projected_gravity.fill(0.);
        projected_gravity.at(2) = -1.0;
        cmd.fill(0.0);
        jpos_processed.fill(0.0);
        jvel.fill(0.0);
        actions.fill(0.0);
        config_file_name = cfg_file;
    }
    void loadParam()
    {
        SimpleRLCfg cfg(config_file_name);
        dt = cfg.dt;
        stand_kp = cfg.stand_kp;
        stand_kd = cfg.stand_kd;
        action_scale = cfg.action_scale;
        lin_vel_scale = cfg.lin_vel_scale;
        ang_vel_scale = cfg.ang_vel_scale;
        dof_pos_scale = cfg.dof_pos_scale;
        dof_vel_scale = cfg.dof_vel_scale;
        policy_name = cfg.policy_name;
        ctrl_kp = cfg.ctrl_kp;
        ctrl_kd = cfg.ctrl_kd;
        num_obs = cfg.num_obs;
        obs.resize(num_obs);
        for (int i = 0; i < 12; ++i)
        {
            stand_pos.at(i) = cfg.stand_pos.at(i);
            sit_pos.at(i) = cfg.sit_pos.at(i);
        }
    }
    void loadPolicy()
    {
        fs::path model_path = fs::current_path() / "../models";
        policy = torch::jit::load(model_path / policy_name);
        std::cout << "Load policy from: " << model_path / policy_name << std::endl;
    }
    void reset(BasicRobotInterface &robot_interface, Gamepad &gamepad)
    {
        // do nothing
    }
    void GetInput(BasicRobotInterface &robot_interface, Gamepad &gamepad)
    {
        // save necessary data from input
        std::copy(robot_interface.gyro.begin(), robot_interface.gyro.end(), base_ang_vel.begin());
        std::copy(robot_interface.projected_gravity.begin(), robot_interface.projected_gravity.end(), projected_gravity.begin());
        // record command
        cmd.at(0) = gamepad.ly; // linear_x: [-1,1]
        cmd.at(1) = -gamepad.lx; // linear_y; [-1,1]
        cmd.at(2) = -gamepad.rx; // angular_z: [-1,1]
        // record robot state
        for (int i = 0; i < 12; ++i)
        {
            jpos_processed.at(i) = robot_interface.jpos.at(i) - stand_pos.at(i);
            jvel.at(i) = robot_interface.jvel.at(i);
        }
    }
    void DummyCalculate()
    {
        // warmup the neural network
        for(int i = 0; i < num_obs; ++i)
        {
            obs.at(i) = 0.0;
        }
        std::vector<torch::jit::IValue> policy_input;
        torch::Tensor policy_input_tensor = torch::zeros({1, num_obs});
        for(int i = 0; i < num_obs; ++i)
        {
            policy_input_tensor[0][i] = obs.at(i);
        }
        policy_input.push_back(policy_input_tensor);
        torch::Tensor policy_output_tensor = policy.forward(policy_input).toTensor();
    }
    void Calculate()
    {
        // Fill observation
        for(int i = 0; i < 3; ++i)
        {
            obs.at(i) = base_ang_vel.at(i) * ang_vel_scale;
            obs.at(3 + i) = projected_gravity.at(i);
        }
        obs.at(6) = cmd.at(0) * lin_vel_scale;
        obs.at(7) = cmd.at(1) * lin_vel_scale;
        obs.at(8) = cmd.at(2) * ang_vel_scale;
        for(int i = 0; i < 12; ++i)
        {
            obs.at(9 + i) = jpos_processed.at(i) * dof_pos_scale;
            obs.at(21 + i) = jvel.at(i) * dof_vel_scale;
            obs.at(33 + i) = actions.at(i);
        }
        // Conduct Policy Inference
        std::vector<torch::jit::IValue> policy_input;
        torch::Tensor policy_input_tensor = torch::zeros({1, num_obs});
        for(int i = 0; i < num_obs; ++i)
        {
            policy_input_tensor[0][i] = obs.at(i);
        }
        policy_input.push_back(policy_input_tensor);
        torch::Tensor policy_output_tensor = policy.forward(policy_input).toTensor();
        std::array<float, 12> actions_scaled;
        for(int i = 0; i < 12; ++i)
        {
            actions.at(i) = policy_output_tensor[0][i].item<float>();
            actions_scaled.at(i) = actions.at(i) * action_scale;
            jpos_des.at(i) = actions_scaled.at(i) + stand_pos.at(i);
        }
    }
    std::vector<float> GetLog()
    {
        // record input, output and other info into a vector
        std::vector<float> log;
        log.push_back(cmd.at(0));
        log.push_back(cmd.at(1));
        log.push_back(cmd.at(2));
        return log;
    }

    std::string config_file_name;
    
    // observation
    std::array<float, 3> base_ang_vel;
    std::array<float, 3> projected_gravity;
    std::array<float, 3> cmd;
    std::array<float, 12> jpos_processed;       //joint sequence: sim
    std::array<float, 12> jvel;
    std::array<float, 12> actions;
    std::vector<float> obs;                     // current observation
    int num_obs;                                // length of observation
    // normalization parameters
    float lin_vel_scale;
    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    // control params
    float action_scale;
    // NN model
    std::string policy_name;
    torch::jit::script::Module policy;
};
/**
 * @brief Inheritance of BasicUserController, controller for Walk These Ways
 * 
 */
class WTWController : public BasicUserController
{
    public:
        WTWController(const std::string& cfg_file): BasicUserController()
         {
            // observation init to 0
            base_ang_vel.fill(0.0);
            projected_gravity.fill(0.);
            projected_gravity.at(2) = -1.0;
            cmd.fill(0.0);
            jpos_processed.fill(0.0);
            jvel.fill(0.0);
            actions.fill(0.0);
            clock_input.fill(0.0);
            theta.fill(0.0);
            gait_choice = 0; // default to the first gait
            config_file_name = cfg_file;
        }

        void loadParam()
        {
            WTWCfg cfg(config_file_name);
            dt = cfg.dt;
            stand_kp = cfg.stand_kp;
            stand_kd = cfg.stand_kd;
            action_scale = cfg.action_scale;
            lin_vel_scale = cfg.lin_vel_scale;
            ang_vel_scale = cfg.ang_vel_scale;
            dof_pos_scale = cfg.dof_pos_scale;
            dof_vel_scale = cfg.dof_vel_scale;
            policy_name = cfg.policy_name;
            ctrl_kp = cfg.ctrl_kp;
            ctrl_kd = cfg.ctrl_kd;
            frame_stack = cfg.frame_stack;
            num_single_obs = cfg.num_single_obs;
            num_gaits = cfg.num_gaits;
            theta_fl.resize(num_gaits);
            theta_fr.resize(num_gaits);
            theta_rl.resize(num_gaits);
            theta_rr.resize(num_gaits);
            for (int i = 0; i < 12; ++i)
            {
                stand_pos.at(i) = cfg.stand_pos.at(i);
                sit_pos.at(i) = cfg.sit_pos.at(i);
            }
            for (int i = 0; i < num_gaits; ++i)
            {
                theta_fl.at(i) = cfg.theta_fl.at(i);
                theta_fr.at(i) = cfg.theta_fr.at(i);
                theta_rl.at(i) = cfg.theta_rl.at(i);
                theta_rr.at(i) = cfg.theta_rr.at(i);
            }
            // Read behavior param range
            for (int i = 0; i < 2; ++i)
            {
                gait_period_range.at(i) = cfg.gait_period_range.at(i);
                base_height_target_range.at(i) = cfg.base_height_target_range.at(i);
                foot_clearance_target_range.at(i) = cfg.foot_clearance_target_range.at(i);
                pitch_target_range.at(i) = cfg.pitch_target_range.at(i);
            }
            gait_period = gait_period_range.at(1);
            base_height_target = base_height_target_range.at(1);
            foot_clearance_target = foot_clearance_target_range.at(1);
            pitch_target = pitch_target_range.at(1);
            theta.at(0) = theta_fl.at(gait_choice);
            theta.at(1) = theta_fr.at(gait_choice);
            theta.at(2) = theta_rl.at(gait_choice);
            theta.at(3) = theta_rr.at(gait_choice);
            // initialize history observation buffer
            // prepare history buffers
            single_step_obs.resize(num_single_obs);
            single_step_obs.clear();
            for(int i = 0; i < num_single_obs; ++i)
            {
                single_step_obs.push_back(0.0);
            }
            history_obs.resize(frame_stack);
            for(int i = 0; i < frame_stack; ++i)
            {
                history_obs.at(i).resize(num_single_obs);
                history_obs.at(i) = single_step_obs;
            }
        }

        /**
         * @brief 载入运动策略
         * @note  载入后缀为.pt的模型文件, 该模型文件使用torch.jit.save()保存.
         * @note  默认模型文件路径为../models/
        */
        void loadPolicy()
        {
            fs::path model_path = fs::current_path() / "../models";
            policy = torch::jit::load(model_path / policy_name);
            std::cout << "Load policy from: " << model_path / policy_name << std::endl;
        }

        void GetInput(BasicRobotInterface &robot_interface, Gamepad &gamepad)
        {
            // save necessary data from input
            std::copy(robot_interface.gyro.begin(), robot_interface.gyro.end(), base_ang_vel.begin());
            std::copy(robot_interface.projected_gravity.begin(), robot_interface.projected_gravity.end(), projected_gravity.begin());

            // Left and Right for gait period
            if(gamepad.left.pressed)
            {
                gait_period += 0.01;
                gait_period = std::min(gait_period, gait_period_range.at(1));
            }
            else if(gamepad.right.pressed)
            {
                gait_period -= 0.01;
                gait_period = std::max(gait_period, gait_period_range.at(0));
            }
            // Up and Down for base height target
            if(gamepad.up.pressed)
            {
                base_height_target += 0.01;
                base_height_target = std::min(base_height_target, base_height_target_range.at(1));
            }
            else if(gamepad.down.pressed)
            {
                base_height_target -= 0.01;
                base_height_target = std::max(base_height_target, base_height_target_range.at(0));
            }
            // A and B for foot clearance target
            if(gamepad.A.pressed)
            {
                foot_clearance_target += 0.01;
                foot_clearance_target = std::min(foot_clearance_target, foot_clearance_target_range.at(1));
            }
            else if(gamepad.B.pressed)
            {
                foot_clearance_target -= 0.01;
                foot_clearance_target = std::max(foot_clearance_target, foot_clearance_target_range.at(0));
            }
            // X and Y for pitch target
            if(gamepad.X.pressed)
            {
                pitch_target += 0.01;
                pitch_target = std::min(pitch_target, pitch_target_range.at(1));
            }
            else if(gamepad.Y.pressed)
            {
                pitch_target -= 0.01;
                pitch_target = std::max(pitch_target, pitch_target_range.at(0));
            }
            // R1 and R2 for gait choice
            if(gamepad.R1.on_press)
            {
                gait_choice = (gait_choice + 1) % num_gaits;
            }
            else if(gamepad.R2.on_press)
            {
                gait_choice = (gait_choice - 1 + num_gaits) % num_gaits;
            }
            // record command
            cmd.at(0) = gamepad.ly; // linear_x: [-1,1]
            cmd.at(1) = -gamepad.lx; // linear_y; [-1,1]
            cmd.at(2) = -gamepad.rx; // angular_z: [-1,1]

            // record robot state
            for (int i = 0; i < 12; ++i)
            {
                jpos_processed.at(i) = robot_interface.jpos.at(i) - stand_pos.at(i);
                jvel.at(i) = robot_interface.jvel.at(i);
            }
        }

        void reset(BasicRobotInterface &robot_interface, Gamepad &gamepad)
        {
            gait_time = 0.0;
            phi = 0.0;
            gait_choice = 0; 
            gait_period = gait_period_range.at(1);
            base_height_target = base_height_target_range.at(1);
            foot_clearance_target = foot_clearance_target_range.at(1);
            pitch_target = pitch_target_range.at(1);
            theta.at(0) = theta_fl.at(gait_choice);
            theta.at(1) = theta_fr.at(gait_choice);
            theta.at(2) = theta_rl.at(gait_choice);
            theta.at(3) = theta_rr.at(gait_choice);
            // reset history observation buffer
            for(int i = 0; i < num_single_obs; ++i)
            {
                single_step_obs.at(i) = 0.0;
            }
            for(int i = 0; i < frame_stack; ++i)
            {
                history_obs.at(i).resize(num_single_obs);
                history_obs.at(i) = single_step_obs;
            }
        }

        void DummyCalculate()
        {
            // warmup the neural network
            for(int i = 0; i < num_single_obs; ++i)
            {
                single_step_obs.at(i) = 0.0;
            }
            history_obs.pop_front();
            history_obs.push_back(single_step_obs);
            std::vector<torch::jit::IValue> policy_input;
            torch::Tensor policy_input_tensor = torch::zeros({1, frame_stack*num_single_obs});
            for(int i = 0; i < frame_stack; ++i)
            {
                for(int j = 0; j < num_single_obs; ++j)
                {
                    policy_input_tensor[0][i*num_single_obs + j] = history_obs.at(i).at(j);
                }
            }
            policy_input.push_back(policy_input_tensor);
            torch::Tensor policy_output_tensor = policy.forward(policy_input).toTensor();
        }

        void Calculate()
        {
            pre_process();
            // put current observation into history buffer
            fill_single_step_obs();
            history_obs.pop_front();
            history_obs.push_back(single_step_obs);
            std::vector<torch::jit::IValue> policy_input;
            torch::Tensor policy_input_tensor = torch::zeros({1, frame_stack*num_single_obs});
            for(int i = 0; i < frame_stack; ++i)
            {
                for(int j = 0; j < num_single_obs; ++j)
                {
                    policy_input_tensor[0][i*num_single_obs + j] = history_obs.at(i).at(j);
                }
            }
            policy_input.push_back(policy_input_tensor);
            torch::Tensor policy_output_tensor = policy.forward(policy_input).toTensor();
            /***** 计算期望位置 *****/
            std::array<float, 12> actions_scaled;
            for(int i = 0; i < 12; ++i)
            {
                actions.at(i) = policy_output_tensor[0][i].item<float>();
                actions_scaled.at(i) = actions.at(i) * action_scale;
                jpos_des.at(i) = stand_pos.at(i) + actions_scaled.at(i);
            }
        }

        std::vector<float> GetLog()
        {
            // record input, output and other info into a vector
            std::vector<float> log;
            for (int i = 0; i < 3; ++i)
            {
                log.push_back(cmd.at(i));
            }
            for (int i = 0; i < 3; ++i)
            {
                log.push_back(projected_gravity.at(i));
            }
            for (int i = 0; i < 3; ++i)
            {
                log.push_back(base_ang_vel.at(i));
            }
            for (int i = 0; i < 12; ++i)
            {
                log.push_back(jpos_processed.at(i));
            }
            for (int i = 0; i < 12; ++i)
            {
                log.push_back(jvel.at(i));
            }
            for (int i = 0; i < 12; ++i)
            {
                log.push_back(actions.at(i));
            }
            for (int i = 0; i < 4; i++)
            {
                log.push_back(clock_input.at(i));
            }
            log.push_back(gait_period);
            log.push_back(base_height_target);
            log.push_back(foot_clearance_target);
            log.push_back(pitch_target);
            for (int i = 0; i < 4; i++)
            {
                log.push_back(theta.at(i));
            }
            return log;
        }
        
        std::string config_file_name;
    
        // observation
        std::array<float, 3> base_ang_vel;
        std::array<float, 3> projected_gravity;
        std::array<float, 3> cmd;
        std::array<float, 12> jpos_processed;       //joint sequence: sim
        std::array<float, 12> jvel;
        std::array<float, 12> actions;
        std::array<float, 4> clock_input;
        float gait_period;
        float base_height_target;
        float foot_clearance_target;
        float pitch_target;
        std::array<float, 4> theta; // theta_fl, theta_fr, theta_rl, theta_rr
        int frame_stack;
        int num_single_obs; // length of single step observation
        std::vector<float> single_step_obs; // 用于记录单步观测数据
        std::deque<std::vector<float>> history_obs; // 用于记录历史观测数据

        // normalization parameters
        float lin_vel_scale;
        float ang_vel_scale;
        float dof_pos_scale;
        float dof_vel_scale;
        // control params
        float action_scale;
        // gait
        int num_gaits;
        int gait_choice;
        std::array<float, 2> gait_period_range;
        std::array<float, 2> base_height_target_range;
        std::array<float, 2> foot_clearance_target_range;
        std::array<float, 2> pitch_target_range;
        std::vector<float> theta_fl;
        std::vector<float> theta_fr;
        std::vector<float> theta_rl;
        std::vector<float> theta_rr;
        float gait_time;
        float phi;
        // NN model
        std::string policy_name;
        torch::jit::script::Module policy;

    private:
        
        void calc_periodic_obs()
        {
            for (int i = 0; i < 4; i++)
            {
                clock_input.at(i) = sin(2 * M_PI * (phi + theta.at(i)));
            }
        }

        void pre_process()
        {
            // gait time step
            gait_time += dt;
            if(gait_time > (gait_period - dt/2))
            {
                gait_time = 0.0;
            }
            phi = gait_time / gait_period;
            // choose gait
            theta.at(0) = theta_fl.at(gait_choice);
            theta.at(1) = theta_fr.at(gait_choice);
            theta.at(2) = theta_rl.at(gait_choice);
            theta.at(3) = theta_rr.at(gait_choice);
            // calculate clock input
            calc_periodic_obs();
        }

        void fill_single_step_obs()
        {
            single_step_obs.at(0) = cmd.at(0) * lin_vel_scale;
            single_step_obs.at(1) = cmd.at(1) * lin_vel_scale;
            single_step_obs.at(2) = cmd.at(2) * ang_vel_scale;
            for(int i = 0; i < 3; ++i)
            {
                single_step_obs.at(i+3) = projected_gravity.at(i);
                single_step_obs.at(i+6) = base_ang_vel.at(i) * ang_vel_scale;
            }
            for(int i = 0; i < 12; ++i)
            {
                single_step_obs.at(i+9) = jpos_processed.at(i) * dof_pos_scale;
                single_step_obs.at(i+21) = jvel.at(i) * dof_vel_scale;
                single_step_obs.at(i+33) = actions.at(i);
            }
            for(int i = 0; i < 4; ++i)
            {
                single_step_obs.at(i+45) = clock_input.at(i);
                single_step_obs.at(i+53) = theta.at(i);
            }
            single_step_obs.at(49) = gait_period;
            single_step_obs.at(50) = base_height_target;
            single_step_obs.at(51) = foot_clearance_target; 
            single_step_obs.at(52) = pitch_target;
        }
};

} // namespace unitree::common
