#pragma once

// Observation builder keyed by observation name (C++ port of
// booster_deploy/utils/observation_builder.py, adapted from
// booster_deploy_cpp/include/observation_builder.hpp).
//
// Replicates the mjlab ObservationManager pipeline for deployment:
//   - each observation term is looked up by its name
//   - per-term pipeline: compute -> clip -> scale -> history
//   - per-term history uses term-major ordering: each term's full history is
//     flattened before concatenating across terms, i.e.
//     [A_t0, A_t1, ..., A_tH-1, B_t0, B_t1, ...]
//   - history buffers are backfilled with the first frame (mjlab
//     CircularBuffer behavior), not zeros
//
// The set of terms and their parameters (scale / clip / history) come from
// the ONNX metadata, so the builder is task-agnostic.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

#include "onnx_metadata.hpp"
#include "robot_interface.hpp"

namespace booster_deploy
{

class ObservationBuilder
{
public:
    ObservationBuilder(const OnnxMetadata &metadata, unitree::common::Go2RobotInterface &robot)
        : metadata_(metadata), robot_(robot)
    {
        policy_joint_names_ = metadata_.joint_names;
        num_policy_joints_ = static_cast<int>(policy_joint_names_.size());

        // policy joint order -> real robot order
        const auto &real_names = robot_.joint_names();
        meta2real_.resize(num_policy_joints_);
        for (int i = 0; i < num_policy_joints_; ++i)
        {
            auto it = std::find(real_names.begin(), real_names.end(), policy_joint_names_[i]);
            if (it == real_names.end())
            {
                std::cerr << "[obs_builder] policy joint '" << policy_joint_names_[i]
                          << "' not found in robot joint order" << std::endl;
                std::abort();
            }
            meta2real_[i] = static_cast<int>(it - real_names.begin());
        }

        default_joint_pos_ = metadata_.default_joint_pos;
        if (default_joint_pos_.size() != static_cast<size_t>(num_policy_joints_))
        {
            default_joint_pos_.assign(num_policy_joints_, 0.f);
        }

        last_action_.assign(num_policy_joints_, 0.f);
        command_.assign(3, 0.f);
    }

    void reset()
    {
        history_.clear();
        history_filled_.clear();
        last_action_.assign(num_policy_joints_, 0.f);
    }

    // Compute the full observation (term-major, history flattened per term).
    std::vector<float> compute()
    {
        std::vector<float> out;
        out.reserve(metadata_.actor_obs_dim > 0 ? metadata_.actor_obs_dim : 512);
        for (const auto &name : metadata_.observation_names)
        {
            auto term_obs = compute_term(name);
            out.insert(out.end(), term_obs.begin(), term_obs.end());
        }
        return out;
    }

    // Dimension of a single frame of a term (before history).
    int term_dim(const std::string &name) const
    {
        return compute_raw_dim(name);
    }

    int num_policy_joints() const { return num_policy_joints_; }
    const std::vector<int> &meta2real() const { return meta2real_; }

    // Inputs set by the controller each policy step.
    std::vector<float> command_;      // [vx, vy, vyaw]
    std::vector<float> last_action_;  // raw (pre-scale) action, policy order

private:
    // -- term implementations ---------------------------------------------

    std::vector<float> compute_raw(const std::string &name) const
    {
        if (name == "command")
        {
            return command_;
        }
        if (name == "base_ang_vel")
        {
            return {robot_.gyro[0], robot_.gyro[1], robot_.gyro[2]};
        }
        if (name == "projected_gravity")
        {
            return {robot_.projected_gravity[0], robot_.projected_gravity[1],
                    robot_.projected_gravity[2]};
        }
        if (name == "joint_pos")
        {
            std::vector<float> out(num_policy_joints_);
            for (int i = 0; i < num_policy_joints_; ++i)
            {
                out[i] = robot_.jpos[meta2real_[i]] - default_joint_pos_[i];
            }
            return out;
        }
        if (name == "joint_vel")
        {
            std::vector<float> out(num_policy_joints_);
            for (int i = 0; i < num_policy_joints_; ++i)
            {
                out[i] = robot_.jvel[meta2real_[i]];
            }
            return out;
        }
        if (name == "actions")
        {
            return last_action_;
        }
        std::cerr << "[obs_builder] no implementation for observation term '"
                  << name << "'" << std::endl;
        std::abort();
    }

    int compute_raw_dim(const std::string &name) const
    {
        if (name == "command" || name == "base_ang_vel" || name == "projected_gravity")
            return 3;
        if (name == "joint_pos" || name == "joint_vel" || name == "actions")
            return num_policy_joints_;
        return 0;
    }

    // -- pipeline: clip -> scale -> history -------------------------------

    std::vector<float> compute_term(const std::string &name)
    {
        const ObservationTermCfg *cfg = metadata_.term(name);
        std::vector<float> obs = compute_raw(name);

        // clip
        if (!std::isinf(cfg->clip_min) || !std::isinf(cfg->clip_max))
        {
            for (float &v : obs)
            {
                v = std::max(cfg->clip_min, std::min(cfg->clip_max, v));
            }
        }
        // scale (per-dim scale with broadcast)
        for (size_t i = 0; i < obs.size(); ++i)
        {
            obs[i] *= cfg->scale[std::min(i, cfg->scale.size() - 1)];
        }
        // history
        if (cfg->history_length > 0)
        {
            obs = push_history(name, std::move(obs), cfg->history_length);
        }
        return obs;
    }

    std::vector<float> push_history(const std::string &name, std::vector<float> obs, int length)
    {
        auto &buf = history_[name];
        auto &filled = history_filled_[name];

        if (!filled)
        {
            for (int i = 0; i < length; ++i)
            {
                buf.push_back(obs);
            }
            filled = true;
        }
        else
        {
            buf.push_back(std::move(obs));
            while (static_cast<int>(buf.size()) > length)
            {
                buf.pop_front();
            }
        }

        // chronological order: oldest -> newest, flattened
        std::vector<float> out;
        size_t frame = buf.front().size();
        out.reserve(buf.size() * frame);
        for (const auto &f : buf)
        {
            out.insert(out.end(), f.begin(), f.end());
        }
        return out;
    }

    const OnnxMetadata &metadata_;
    unitree::common::Go2RobotInterface &robot_;

    std::vector<std::string> policy_joint_names_;
    std::vector<int> meta2real_;
    std::vector<float> default_joint_pos_;
    int num_policy_joints_ = 0;

    std::unordered_map<std::string, std::deque<std::vector<float>>> history_;
    std::unordered_map<std::string, bool> history_filled_;
};

} // namespace booster_deploy
