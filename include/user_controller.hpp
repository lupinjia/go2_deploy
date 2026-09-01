#pragma once

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include "gamepad.hpp"
#include "observation_builder.hpp"
#include "onnx_metadata.hpp"
#include "robot_interface.hpp"

namespace fs = std::filesystem;

namespace unitree::common
{

class BasicUserController
{
public:
    BasicUserController() {}
    virtual ~BasicUserController() = default;

    virtual void loadParam() = 0;
    virtual void loadPolicy() = 0;
    virtual void reset(Go2RobotInterface &robot_interface, Gamepad &gamepad) = 0;
    virtual void GetInput(Go2RobotInterface &robot_interface, Gamepad &gamepad) = 0;
    virtual void DummyCalculate() = 0;
    virtual void Calculate() = 0;
    virtual void set_depth_frame(const float *data, size_t n) { (void)data; (void)n; }
    virtual std::vector<float> GetLog() = 0;

    // shared robot interface / deployment params (implemented by the controller)
    virtual Go2RobotInterface &robot() = 0;
    virtual float stand_kp() const = 0;
    virtual float stand_kd() const = 0;
    virtual const std::vector<float> &kp() const = 0;
    virtual const std::vector<float> &kd() const = 0;
    virtual const std::vector<float> &stand_pos_real() const = 0;
    virtual const std::vector<float> &last_obs() const = 0;
    virtual const std::vector<float> &last_action() const = 0;

    void save_jpos(Go2RobotInterface &robot_interface)
    {
        std::copy(robot_interface.jpos.begin(), robot_interface.jpos.end(), start_pos.begin());
    }

    float dt = 0.02f;
    std::vector<float> start_pos;  // pose captured on entering stand
    std::vector<float> stand_pos;  // stand pose target (metadata default_joint_pos, policy order)
    std::vector<float> sit_pos;    // sit pose target
    std::vector<float> jpos_des;   // current joint position targets (real robot order)
};

// ---------------------------------------------------------------------------
// Go2 locomotion controller driven by an mjlab-exported ONNX policy.
//
// A single controller type that replaces the previous SimpleRL / WTW / TS
// controllers.  All observation terms, scales, history, action scale, joint
// order, default pose and PD gains are read automatically from the ONNX
// metadata (mirroring booster_deploy_cpp's K1OnnxController).  Observations
// are built term-major by ObservationBuilder.  The yaml only supplies the
// policy file, control period, transition poses and transition gains.
// ---------------------------------------------------------------------------

class Go2OnnxController : public BasicUserController
{
public:
    Go2OnnxController(const std::string &cfg_file, std::vector<std::string> robot_joint_names)
        : cfg_file_(cfg_file), robot_joint_names_(std::move(robot_joint_names))
    {
    }

    void loadParam() override
    {
        auto cfg = YAML::LoadFile(cfg_file_);
        try
        {
            policy_name_ = cfg["policy_name"].as<std::string>();
            dt = cfg["dt"].as<float>();
            stand_kp_ = cfg["stand_kp"].as<float>();
            stand_kd_ = cfg["stand_kd"].as<float>();
            ctrl_kp_fallback_ = cfg["ctrl_kp"].as<float>();
            ctrl_kd_fallback_ = cfg["ctrl_kd"].as<float>();
            if (cfg["sit_pos"])
            {
                for (const auto &v : cfg["sit_pos"])
                {
                    sit_pos.push_back(v.as<float>());
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "[go2_onnx] failed to parse " << cfg_file_ << ": " << e.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    void loadPolicy() override
    {
        fs::path policy_file = policy_name_;
        if (policy_file.is_relative())
        {
            policy_file = fs::current_path() / "../models" / policy_name_;
        }
        if (!fs::exists(policy_file))
        {
            std::cerr << "[go2_onnx] policy not found: " << policy_file << std::endl;
            std::exit(EXIT_FAILURE);
        }

        // ---- onnxruntime session ----
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "go2_deploy");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(4);
        session_ = std::make_unique<Ort::Session>(*env_, policy_file.c_str(), session_options);

        // ---- metadata -> deployment config ----
        metadata_ = std::make_unique<booster_deploy::OnnxMetadata>(
            booster_deploy::load_onnx_metadata(*session_));
        if (metadata_->num_joints() != static_cast<int>(robot_joint_names_.size()))
        {
            std::cerr << "[go2_onnx] metadata joints " << metadata_->num_joints()
                      << " != robot joints " << robot_joint_names_.size() << std::endl;
            std::exit(EXIT_FAILURE);
        }
        metadata_->print_summary();

        // policy input/output names
        Ort::AllocatorWithDefaultOptions allocator;
        auto input_name = session_->GetInputNameAllocated(0, allocator);
        auto output_name = session_->GetOutputNameAllocated(0, allocator);
        input_name_ = input_name.get();
        output_name_ = output_name.get();

        // ---- robot interface + obs builder ----
        robot_ = std::make_unique<Go2RobotInterface>(robot_joint_names_);
        obs_builder_ = std::make_unique<booster_deploy::ObservationBuilder>(*metadata_, *robot_);

        // ---- metadata-driven joint config ----
        stand_pos = metadata_->default_joint_pos;   // policy order
        start_pos = stand_pos;
        if (sit_pos.size() != stand_pos.size())
        {
            sit_pos = stand_pos;
        }
        stand_pos_real_.assign(robot_joint_names_.size(), 0.f);
        for (size_t i = 0; i < metadata_->joint_names.size(); ++i)
        {
            stand_pos_real_[obs_builder_->meta2real()[i]] = stand_pos[i];
        }
        jpos_des = stand_pos_real_;
        action_scale_ = metadata_->action_scale;

        kp_ = std::vector<float>(robot_joint_names_.size(), ctrl_kp_fallback_);
        kd_ = std::vector<float>(robot_joint_names_.size(), ctrl_kd_fallback_);
        if (metadata_->joint_stiffness.size() == metadata_->joint_names.size() &&
            metadata_->joint_damping.size() == metadata_->joint_names.size())
        {
            // per-joint PD gains, mapped from policy order to real robot order
            for (size_t i = 0; i < metadata_->joint_names.size(); ++i)
            {
                int real_idx = obs_builder_->meta2real()[i];
                kp_[real_idx] = metadata_->joint_stiffness[i];
                kd_[real_idx] = metadata_->joint_damping[i];
            }
        }
        std::cout << "[go2_onnx] policy loaded: " << policy_file
                  << "  (input: " << input_name_ << ", obs_dim=" << metadata_->actor_obs_dim
                  << ", output: " << output_name_ << ")" << std::endl;
    }

    void reset(Go2RobotInterface &robot_interface, Gamepad &gamepad) override
    {
        (void)robot_interface;
        (void)gamepad;
        obs_builder_->reset();
        obs_builder_->last_action_.assign(obs_builder_->num_policy_joints(), 0.f);
        actions_.assign(obs_builder_->num_policy_joints(), 0.f);
    }

    void GetInput(Go2RobotInterface &robot_interface, Gamepad &gamepad) override
    {
        (void)robot_interface;  // state is read from the shared robot interface
        // velocity command from gamepad (same convention as the previous controllers)
        obs_builder_->command_[0] = gamepad.ly;   // linear_x: [-1,1]
        obs_builder_->command_[1] = -gamepad.lx;  // linear_y: [-1,1]
        obs_builder_->command_[2] = -gamepad.rx;  // angular_z: [-1,1]
    }

    void Calculate() override
    {
        if (!robot_ || !obs_builder_)
        {
            std::cerr << "[go2_onnx] Calculate called before loadPolicy" << std::endl;
            std::abort();
        }

        std::vector<float> obs = obs_builder_->compute();
        if (static_cast<int>(obs.size()) != metadata_->actor_obs_dim)
        {
            std::cerr << "[go2_onnx] obs dim " << obs.size()
                      << " != actor_obs_dim " << metadata_->actor_obs_dim << std::endl;
            std::abort();
        }
        last_obs_ = obs;

        // ---- inference ----
        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::vector<int64_t> input_shape{1, metadata_->actor_obs_dim};
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem, obs.data(), obs.size(), input_shape.data(), input_shape.size());

        const char *input_names[] = {input_name_.c_str()};
        const char *output_names[] = {output_name_.c_str()};
        auto outputs = session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1,
                                     output_names, 1);
        float *action_raw = outputs[0].GetTensorMutableData<float>();

        // raw (pre-scale) action for the next obs step
        int n = metadata_->num_joints();
        actions_.assign(action_raw, action_raw + n);
        obs_builder_->last_action_ = actions_;

        // dof targets in real robot order: default pose + scaled action
        jpos_des = stand_pos_real_;
        const auto &meta2real = obs_builder_->meta2real();
        for (int i = 0; i < n; ++i)
        {
            jpos_des[meta2real[i]] += actions_[i] * action_scale_[i];
        }
    }

    // apply a zero-observation warmup inference
    void DummyCalculate() override
    {
        if (!session_ || !metadata_) return;
        std::vector<float> obs(metadata_->actor_obs_dim, 0.f);
        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        std::vector<int64_t> shape{1, metadata_->actor_obs_dim};
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem, obs.data(), obs.size(), shape.data(), shape.size());
        const char *input_names[] = {input_name_.c_str()};
        const char *output_names[] = {output_name_.c_str()};
        session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
    }

    std::vector<float> GetLog() override
    {
        // record input, output and other info into a vector
        std::vector<float> log;
        for (int i = 0; i < 3; ++i)
        {
            log.push_back(obs_builder_->command_.at(i));
        }
        return log;
    }

    // per-joint PD gains for the CTRL state (real robot order)
    const std::vector<float> &kp() const override { return kp_; }
    const std::vector<float> &kd() const override { return kd_; }
    float stand_kp() const override { return stand_kp_; }
    float stand_kd() const override { return stand_kd_; }
    const std::vector<float> &stand_pos_real() const override { return stand_pos_real_; }

    Go2RobotInterface &robot() override { return *robot_; }
    const std::vector<float> &last_obs() const override { return last_obs_; }
    const std::vector<float> &last_action() const override { return actions_; }

private:
    std::string cfg_file_;
    std::vector<std::string> robot_joint_names_;

    std::string policy_name_;
    float stand_kp_ = 60.f, stand_kd_ = 5.f;
    float ctrl_kp_fallback_ = 20.f, ctrl_kd_fallback_ = 0.5f;

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::string input_name_, output_name_;

    std::unique_ptr<booster_deploy::OnnxMetadata> metadata_;
    std::unique_ptr<Go2RobotInterface> robot_;
    std::unique_ptr<booster_deploy::ObservationBuilder> obs_builder_;

    std::vector<float> action_scale_;
    std::vector<float> stand_pos_real_;
    std::vector<float> kp_, kd_;
    std::vector<float> actions_;
    std::vector<float> last_obs_;
};

} // namespace unitree::common
