#pragma once

#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace unitree::common
{
    class BaseCfg
    {
    public:
        BaseCfg(const std::string &filename){};

    };

    class SimpleRLCfg : public BaseCfg
    {
    public:
        SimpleRLCfg(const std::string &filename) : BaseCfg(filename)
        {
            auto cfg = YAML::LoadFile(filename);
            try
            {
                policy_name = cfg["policy_name"].as<std::string>();
                dt = cfg["dt"].as<float>();
                stand_kp = cfg["stand_kp"].as<float>();
                stand_kd = cfg["stand_kd"].as<float>();
                ctrl_kp = cfg["ctrl_kp"].as<float>();
                ctrl_kd = cfg["ctrl_kd"].as<float>();
                action_scale = cfg["action_scale"].as<float>();
                lin_vel_scale = cfg["lin_vel_scale"].as<float>();
                ang_vel_scale = cfg["ang_vel_scale"].as<float>();
                dof_pos_scale = cfg["dof_pos_scale"].as<float>();
                dof_vel_scale = cfg["dof_vel_scale"].as<float>();
                num_obs = cfg["num_obs"].as<int>();
                for(const auto& v : cfg["stand_pos"])
                {
                    stand_pos.push_back(v.as<float>());
                }
                for(const auto& v : cfg["sit_pos"])
                {
                    sit_pos.push_back(v.as<float>());
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                exit(EXIT_FAILURE);
            }
        }
        
        float stand_kp;
        float stand_kd;
        float ctrl_kp;
        float ctrl_kd;
        float dt;
        float action_scale;
        float lin_vel_scale;
        float ang_vel_scale;
        float dof_pos_scale;
        float dof_vel_scale;
        int num_obs;
        std::string policy_name;
        std::vector<float> stand_pos;
        std::vector<float> sit_pos;
        
    };
    
    class WTWCfg : public BaseCfg
    {
    public:
        WTWCfg(const std::string &filename) : BaseCfg(filename)
        {
            auto cfg = YAML::LoadFile(filename);
            try
            {
                policy_name = cfg["policy_name"].as<std::string>();
                use_genesis = cfg["use_genesis"].as<bool>();
                dt = cfg["dt"].as<float>();
                stand_kp = cfg["stand_kp"].as<float>();
                stand_kd = cfg["stand_kd"].as<float>();
                ctrl_kp = cfg["ctrl_kp"].as<float>();
                ctrl_kd = cfg["ctrl_kd"].as<float>();
                action_scale = cfg["action_scale"].as<float>();
                lin_vel_scale = cfg["lin_vel_scale"].as<float>();
                ang_vel_scale = cfg["ang_vel_scale"].as<float>();
                dof_pos_scale = cfg["dof_pos_scale"].as<float>();
                dof_vel_scale = cfg["dof_vel_scale"].as<float>();
                frame_stack = cfg["frame_stack"].as<int>();
                num_single_obs = cfg["num_single_obs"].as<int>();
                num_gaits = cfg["num_gaits"].as<int>();
                for(const auto& v : cfg["gait_period_range"])
                {
                    gait_period_range.push_back(v.as<float>());
                }
                for(const auto& v : cfg["base_height_target_range"])
                {
                    base_height_target_range.push_back(v.as<float>());
                }
                for(const auto& v : cfg["foot_clearance_target_range"])
                {
                    foot_clearance_target_range.push_back(v.as<float>());
                }
                for(const auto& v : cfg["pitch_target_range"])
                {
                    pitch_target_range.push_back(v.as<float>());
                }
                for(const auto& v : cfg["theta_fl"])
                {
                    theta_fl.push_back(v.as<float>());
                }
                for(const auto& v : cfg["theta_fr"])
                {
                    theta_fr.push_back(v.as<float>());
                }
                for(const auto& v : cfg["theta_rl"])
                {
                    theta_rl.push_back(v.as<float>());
                }
                for(const auto& v : cfg["theta_rr"])
                {
                    theta_rr.push_back(v.as<float>());
                }
                for(const auto& v : cfg["stand_pos"])
                {
                    stand_pos.push_back(v.as<float>());
                }
                for(const auto& v : cfg["sit_pos"])
                {
                    sit_pos.push_back(v.as<float>());
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                exit(EXIT_FAILURE);
            }
        }

        bool use_genesis;
        float ctrl_kp;
        float ctrl_kd;
        float stand_kp;
        float stand_kd;
        float dt;
        float action_scale;
        float lin_vel_scale;
        float ang_vel_scale;
        float dof_pos_scale;
        float dof_vel_scale;
        int frame_stack;
        int num_single_obs;
        std::string policy_name;
        std::vector<float> stand_pos;
        std::vector<float> sit_pos;
        // gait parameters
        int num_gaits;
        std::vector<float> gait_period_range;
        std::vector<float> base_height_target_range;
        std::vector<float> foot_clearance_target_range;
        std::vector<float> pitch_target_range;
        std::vector<float> theta_fl;
        std::vector<float> theta_fr;
        std::vector<float> theta_rl;
        std::vector<float> theta_rr;
    };

    class TSCfg : public BaseCfg
    {
    public:
        TSCfg(const std::string &filename) : BaseCfg(filename)
        {
            auto cfg = YAML::LoadFile(filename);
            try
            {
                policy_name = cfg["policy_name"].as<std::string>();
                encoder_name = cfg["encoder_name"].as<std::string>();
                use_genesis = cfg["use_genesis"].as<bool>();
                dt = cfg["dt"].as<float>();
                stand_kp = cfg["stand_kp"].as<float>();
                stand_kd = cfg["stand_kd"].as<float>();
                ctrl_kp = cfg["ctrl_kp"].as<float>();
                ctrl_kd = cfg["ctrl_kd"].as<float>();
                action_scale = cfg["action_scale"].as<float>();
                lin_vel_scale = cfg["lin_vel_scale"].as<float>();
                ang_vel_scale = cfg["ang_vel_scale"].as<float>();
                dof_pos_scale = cfg["dof_pos_scale"].as<float>();
                dof_vel_scale = cfg["dof_vel_scale"].as<float>();
                frame_stack = cfg["frame_stack"].as<int>();
                num_single_obs = cfg["num_single_obs"].as<int>();
                for(const auto& v : cfg["stand_pos"])
                {
                    stand_pos.push_back(v.as<float>());
                }
                for(const auto& v : cfg["sit_pos"])
                {
                    sit_pos.push_back(v.as<float>());
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                exit(EXIT_FAILURE);
            }
        }

        bool use_genesis;
        float ctrl_kp;
        float ctrl_kd;
        float stand_kp;
        float stand_kd;
        float dt;
        float action_scale;
        float lin_vel_scale;
        float ang_vel_scale;
        float dof_pos_scale;
        float dof_vel_scale;
        int frame_stack;
        int num_single_obs;
        std::string policy_name;
        std::string encoder_name;
        std::vector<float> stand_pos;
        std::vector<float> sit_pos;
    };
}