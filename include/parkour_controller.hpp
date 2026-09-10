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
#include "user_controller.hpp"  // for BasicUserController

namespace fs = std::filesystem;

namespace unitree::common
{

// Depth-image parkour (越障) policy controller.
//
// Runs the 4-input / 2-output CTS-Depth GRU model:
//   inputs  : current_obs [1,45], obs_history [1,900],
//             depth_image [1,2,30,53], hidden_state [1,1,512]
//   outputs : actions [1,12], new_hidden_state [1,1,512]
//
// All joint config, PD gains, obs scales/clips/history, depth params and RNN
// dims are read from the ONNX metadata. The depth frame is fed in from a
// DepthImageSubscriber via set_depth_frame().
class Go2ParkourController : public BasicUserController
{
public:
    Go2ParkourController(const std::string &cfg_file,
                         std::vector<std::string> robot_joint_names)
        : cfg_file_(cfg_file), robot_joint_names_(std::move(robot_joint_names))
    {
    }

    void loadParam() override
    {
        auto cfg = YAML::LoadFile(cfg_file_);
        try
        {
            policy_name_ = cfg["parkour_policy_name"].as<std::string>();
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
            std::cerr << "[parkour] failed to parse " << cfg_file_ << ": " << e.what() << std::endl;
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
            std::cerr << "[parkour] policy not found: " << policy_file << std::endl;
            std::exit(EXIT_FAILURE);
        }

        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "go2_parkour");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(4);
        session_ = std::make_unique<Ort::Session>(*env_, policy_file.c_str(), session_options);

        metadata_ = std::make_unique<booster_deploy::OnnxMetadata>(
            booster_deploy::load_onnx_metadata(*session_));
        if (!metadata_->has_perception())
        {
            std::cerr << "[parkour] model has no perception metadata (onnx_input_names) — "
                         "not a parkour/depth model"
                      << std::endl;
            std::exit(EXIT_FAILURE);
        }
        if (metadata_->num_joints() != static_cast<int>(robot_joint_names_.size()))
        {
            std::cerr << "[parkour] metadata joints " << metadata_->num_joints()
                      << " != robot joints " << robot_joint_names_.size() << std::endl;
            std::exit(EXIT_FAILURE);
        }
        metadata_->print_summary();

        // input/output names in graph order
        input_names_.clear();
        for (const auto &n : metadata_->onnx_input_names) input_names_.push_back(n);
        output_names_.clear();
        for (const auto &n : metadata_->onnx_output_names) output_names_.push_back(n);

        robot_ = std::make_unique<Go2RobotInterface>(robot_joint_names_);
        obs_builder_ = std::make_unique<booster_deploy::ObservationBuilder>(*metadata_, *robot_);

        // joint config (same mapping as Go2OnnxController)
        stand_pos = metadata_->default_joint_pos;
        start_pos = stand_pos;
        if (sit_pos.size() != stand_pos.size()) sit_pos = stand_pos;
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
            for (size_t i = 0; i < metadata_->joint_names.size(); ++i)
            {
                int real_idx = obs_builder_->meta2real()[i];
                kp_[real_idx] = metadata_->joint_stiffness[i];
                kd_[real_idx] = metadata_->joint_damping[i];
            }
        }

        // GRU hidden state, initialized to zeros
        hidden_state_.assign(static_cast<size_t>(metadata_->rnn_hidden_dim), 0.f);

        std::cout << "[parkour] policy loaded: " << policy_file << "  (inputs:";
        for (const auto &n : metadata_->onnx_input_names) std::cout << " " << n;
        std::cout << "; rnn=" << metadata_->rnn_type
                  << " " << metadata_->rnn_hidden_dim << ")" << std::endl;
    }

    void reset(Go2RobotInterface &robot_interface, Gamepad &gamepad) override
    {
        (void)robot_interface;
        (void)gamepad;
        obs_builder_->reset();
        obs_builder_->last_action_.assign(obs_builder_->num_policy_joints(), 0.f);
        actions_.assign(obs_builder_->num_policy_joints(), 0.f);
        std::fill(hidden_state_.begin(), hidden_state_.end(), 0.f);
    }

    void GetInput(Go2RobotInterface &robot_interface, Gamepad &gamepad) override
    {
        (void)robot_interface;  // state read from shared robot interface
        obs_builder_->command_[0] = gamepad.ly;   // linear_x
        obs_builder_->command_[1] = -gamepad.lx;  // linear_y
        obs_builder_->command_[2] = -gamepad.rx;  // angular_z
    }

    void Calculate() override
    {
        if (!robot_ || !obs_builder_)
        {
            std::cerr << "[parkour] Calculate called before loadPolicy" << std::endl;
            std::abort();
        }

        // ---- build 4 input tensors ----
        auto cur = obs_builder_->compute_current();          // [45]
        auto hist = obs_builder_->compute_history();         // [900]
        auto depth = obs_builder_->depth_input();            // [2*30*53]
        size_t depth_n = static_cast<size_t>(metadata_->depth_height())
                       * metadata_->depth_width() * 2;
        if (depth.size() != depth_n)
        {
            // no depth frame yet: fill with 1.0 (far plane / background)
            depth.assign(depth_n, 1.0f);
        }

        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto cur_shape = metadata_->input_shape("current_obs");
        auto hist_shape = metadata_->input_shape("obs_history");
        auto depth_shape = metadata_->input_shape("depth_image");
        auto hidden_shape = metadata_->input_shape("hidden_state");

        Ort::Value cur_t = Ort::Value::CreateTensor<float>(
            mem, cur.data(), cur.size(), cur_shape.data(), cur_shape.size());
        Ort::Value hist_t = Ort::Value::CreateTensor<float>(
            mem, hist.data(), hist.size(), hist_shape.data(), hist_shape.size());
        Ort::Value depth_t = Ort::Value::CreateTensor<float>(
            mem, depth.data(), depth.size(), depth_shape.data(), depth_shape.size());
        Ort::Value hidden_t = Ort::Value::CreateTensor<float>(
            mem, hidden_state_.data(), hidden_state_.size(),
            hidden_shape.data(), hidden_shape.size());

        Ort::Value inputs[] = {std::move(cur_t), std::move(hist_t),
                               std::move(depth_t), std::move(hidden_t)};
        const char *input_ptrs[4] = {input_names_[0].c_str(), input_names_[1].c_str(),
                                     input_names_[2].c_str(), input_names_[3].c_str()};
        const char *output_ptrs[2] = {output_names_[0].c_str(), output_names_[1].c_str()};

        auto outputs = session_->Run(Ort::RunOptions{nullptr}, input_ptrs, inputs, 4,
                                     output_ptrs, 2);

        // output 0: actions [1,12]; output 1: new_hidden_state [1,1,512]
        float *action_raw = outputs[0].GetTensorMutableData<float>();
        int n = metadata_->num_joints();
        actions_.assign(action_raw, action_raw + n);
        obs_builder_->last_action_ = actions_;

        float *new_hidden = outputs[1].GetTensorMutableData<float>();
        hidden_state_.assign(new_hidden, new_hidden + metadata_->rnn_hidden_dim);

        // dof targets in real order: default pose + scaled action
        jpos_des = stand_pos_real_;
        const auto &meta2real = obs_builder_->meta2real();
        for (int i = 0; i < n; ++i)
        {
            jpos_des[meta2real[i]] += actions_[i] * action_scale_[i];
        }
    }

    // warmup with zero inputs (0 obs, 0 hidden, 0 depth)
    void DummyCalculate() override
    {
        if (!session_ || !metadata_) return;
        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputs[4];
        for (size_t i = 0; i < 4 && i < metadata_->onnx_input_names.size(); ++i)
        {
            auto sh = metadata_->input_shape(metadata_->onnx_input_names[i]);
            size_t total = 1;
            for (auto d : sh) total *= static_cast<size_t>(d);
            std::vector<float> zeros(total, 0.f);
            inputs[i] = Ort::Value::CreateTensor<float>(mem, zeros.data(), zeros.size(),
                                                        sh.data(), sh.size());
        }
        const char *input_ptrs[4] = {input_names_[0].c_str(), input_names_[1].c_str(),
                                     input_names_[2].c_str(), input_names_[3].c_str()};
        const char *output_ptrs[2] = {output_names_[0].c_str(), output_names_[1].c_str()};
        session_->Run(Ort::RunOptions{nullptr}, input_ptrs, inputs, 4, output_ptrs, 2);
    }

    // accessors required by BasicUserController / RobotController
    Go2RobotInterface &robot() override { return *robot_; }
    float stand_kp() const override { return stand_kp_; }
    float stand_kd() const override { return stand_kd_; }
    const std::vector<float> &kp() const override { return kp_; }
    const std::vector<float> &kd() const override { return kd_; }
    const std::vector<float> &stand_pos_real() const override { return stand_pos_real_; }
    const std::vector<float> &last_obs() const override { return last_obs_; }
    const std::vector<float> &last_action() const override { return actions_; }

    // parkour-specific: feed the latest depth frame into the obs builder
    void set_depth_frame(const float *data, size_t n,
                         int source_width, int source_height) override
    {
        if (obs_builder_)
            obs_builder_->set_depth_frame(data, n, source_width, source_height);
    }

    std::vector<float> depth_frame_for_visualization(
        const float *data, size_t n, int source_width, int source_height,
        int &output_width, int &output_height) const override
    {
        output_width = metadata_ ? metadata_->depth_width() : source_width;
        output_height = metadata_ ? metadata_->depth_height() : source_height;
        if (!obs_builder_) return {};
        return obs_builder_->crop_depth_frame(data, n, source_width, source_height);
    }

    std::vector<float> GetLog() override
    {
        std::vector<float> log;
        for (int i = 0; i < 3; ++i) log.push_back(obs_builder_->command_.at(i));
        return log;
    }

private:
    std::string cfg_file_;
    std::vector<std::string> robot_joint_names_;
    std::string policy_name_;
    float stand_kp_ = 60.f, stand_kd_ = 5.f;
    float ctrl_kp_fallback_ = 20.f, ctrl_kd_fallback_ = 0.5f;

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<std::string> input_names_, output_names_;

    std::unique_ptr<booster_deploy::OnnxMetadata> metadata_;
    std::unique_ptr<Go2RobotInterface> robot_;
    std::unique_ptr<booster_deploy::ObservationBuilder> obs_builder_;

    std::vector<float> action_scale_;
    std::vector<float> stand_pos_real_;
    std::vector<float> kp_, kd_;
    std::vector<float> actions_;
    std::vector<float> last_obs_;
    std::vector<float> hidden_state_;
};

} // namespace unitree::common
