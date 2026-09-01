#pragma once

// ONNX metadata parser (C++ port of booster_deploy/utils/onnx_metadata.py).
//
// The training repo (mjlab) writes deployment-relevant parameters into the
// ONNX model's ``metadata_props``.  This header reads those props back via
// the onnxruntime C++ API (Ort::ModelMetadata::GetCustomMetadataMap*) and
// exposes them as a structured OnnxMetadata so the deployment controller can
// auto-populate its config without hardcoding values.
//
// Serialization format (mirror of mjlab's exporter_utils):
//   - lists are joined with ',' (outer delimiter)
//   - nested sequences (e.g. clip ranges, per-dim scales) use ';' (inner)
//   - floats are formatted with 3 decimals; -inf/inf pass through
//   - non-numeric items (joint/obs names) are kept as plain strings

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace booster_deploy
{

struct ObservationTermCfg
{
    std::string name;
    std::vector<float> scale;      // per-dim scale (default {1.0})
    float clip_min = -std::numeric_limits<float>::infinity();
    float clip_max = std::numeric_limits<float>::infinity();
    int history_length = 0;
};

struct ObsGroupCfg
{
    int history_length = 0;
    bool flatten_history_dim = true;
    std::vector<std::string> term_names;
    std::unordered_map<std::string, ObservationTermCfg> terms;

    const ObservationTermCfg *term(const std::string &name) const
    {
        auto it = terms.find(name);
        if (it == terms.end())
        {
            std::cerr << "[onnx_metadata] obs_group term '" << name
                      << "' not found; available:";
            for (const auto &kv : terms) std::cerr << " " << kv.first;
            std::cerr << std::endl;
            std::abort();
        }
        return &it->second;
    }
};

struct OnnxMetadata
{
    std::string run_path;
    std::string experiment_name;
    std::string run_name;
    std::string git_user_name;

    std::vector<std::string> joint_names;
    std::vector<float> joint_stiffness;
    std::vector<float> joint_damping;
    std::vector<float> default_joint_pos;

    std::vector<std::string> command_names;
    std::vector<std::string> observation_names;
    std::unordered_map<std::string, ObservationTermCfg> observation_terms;

    std::vector<float> action_scale;   // per-joint (scalar expanded)
    int actor_obs_dim = 0;
    int actor_history_length = 0;
    bool actor_flatten_history_dim = true;

    std::string anchor_body_name;
    std::vector<std::string> body_names;

    // -- perception (depth / RNN / multi-input) -----------------------------
    std::vector<std::string> onnx_input_names;
    std::vector<std::vector<int64_t>> onnx_input_shapes;
    std::vector<std::string> onnx_input_groups;
    std::vector<std::string> onnx_output_names;
    std::vector<std::vector<int64_t>> onnx_output_shapes;
    std::map<std::string, ObsGroupCfg> obs_groups;

    int depth_image_height = 0;
    int depth_image_width = 0;
    float depth_fovy_ = 0.f;
    float depth_far_plane_ = 0.f;
    float depth_cutoff_distance_ = 0.f;
    float depth_min_depth_ = 0.f;

    std::string rnn_type;
    int rnn_hidden_dim = 0;
    int rnn_num_layers = 0;

    bool has_perception() const { return !onnx_input_names.empty(); }

    std::vector<int64_t> input_shape(const std::string &name) const
    {
        for (size_t i = 0; i < onnx_input_names.size(); ++i)
            if (onnx_input_names[i] == name) return onnx_input_shapes[i];
        return {};
    }
    int depth_width() const { return depth_image_width; }
    int depth_height() const { return depth_image_height; }
    float depth_cutoff_distance() const { return depth_cutoff_distance_; }
    float depth_min_depth() const { return depth_min_depth_; }
    const ObsGroupCfg &obs_group(const std::string &name) const
    {
        auto it = obs_groups.find(name);
        if (it == obs_groups.end())
        {
            std::cerr << "[onnx_metadata] obs group '" << name
                      << "' not found; available:";
            for (const auto &kv : obs_groups) std::cerr << " " << kv.first;
            std::cerr << std::endl;
            std::abort();
        }
        return it->second;
    }

    // -- term lookups -----------------------------------------------------

    const ObservationTermCfg *term(const std::string &name) const
    {
        auto it = observation_terms.find(name);
        if (it == observation_terms.end())
        {
            std::cerr << "[onnx_metadata] observation term '" << name
                      << "' not found in ONNX metadata; available:";
            for (const auto &kv : observation_terms)
            {
                std::cerr << " " << kv.first;
            }
            std::cerr << std::endl;
            std::abort();
        }
        return &it->second;
    }

    int num_single_obs() const
    {
        return actor_obs_dim / std::max(actor_history_length, 1);
    }

    int num_joints() const { return static_cast<int>(joint_names.size()); }

    void print_summary() const
    {
        std::cout << "ONNX policy metadata:" << std::endl;
        std::cout << "  experiment_name    : " << experiment_name << std::endl;
        std::cout << "  run_name           : " << run_name << std::endl;
        std::cout << "  command_names      :";
        for (const auto &c : command_names) std::cout << " " << c;
        std::cout << std::endl;
        std::cout << "  actor_obs_dim      : " << actor_obs_dim << std::endl;
        std::cout << "  num_single_obs     : " << num_single_obs() << std::endl;
        std::cout << "  actor_history_len  : " << actor_history_length << std::endl;
        std::cout << "  flatten_history    : " << (actor_flatten_history_dim ? "true" : "false") << std::endl;
        std::cout << "  joints (name / action_scale / default_joint_pos):" << std::endl;
        int num = num_joints();
        for (int i = 0; i < num; ++i)
        {
            float scale = i < static_cast<int>(action_scale.size()) ? action_scale[i] : 0.f;
            float pos = i < static_cast<int>(default_joint_pos.size()) ? default_joint_pos[i] : 0.f;
            std::printf("    %2d. %-24s scale=%8.4f  default=%8.4f\n",
                        i, joint_names[i].c_str(), scale, pos);
        }
        std::cout << "  observation terms:" << std::endl;
        for (const auto &name : observation_names)
        {
            const auto *cfg = term(name);
            std::cout << "    - " << name << ": scale=";
            if (cfg->scale.size() == 1) std::cout << cfg->scale[0];
            else
            {
                std::cout << "[";
                for (size_t i = 0; i < cfg->scale.size(); ++i)
                    std::cout << cfg->scale[i] << (i + 1 < cfg->scale.size() ? ", " : "");
                std::cout << "]";
            }
            std::cout << ", clip=(" << cfg->clip_min << ", " << cfg->clip_max
                      << "), history=" << cfg->history_length << std::endl;
        }
        if (depth_image_height > 0)
        {
            std::cout << "  depth camera:" << std::endl;
            std::cout << "    resolution     : " << depth_image_width << "x" << depth_image_height << std::endl;
            std::cout << "    fovy          : " << depth_fovy_ << std::endl;
            std::cout << "    far_plane     : " << depth_far_plane_ << std::endl;
            std::cout << "    cutoff_distance: " << depth_cutoff_distance_ << std::endl;
            std::cout << "    min_depth     : " << depth_min_depth_ << std::endl;
        }
        for (const auto &[gname, gcfg] : obs_groups)
        {
            std::cout << "  obs_group '" << gname << "':"
                      << " history=" << gcfg.history_length
                      << ", flatten=" << (gcfg.flatten_history_dim ? "true" : "false")
                      << std::endl;
            std::cout << "    terms:";
            for (const auto &t : gcfg.term_names) std::cout << " " << t;
            std::cout << std::endl;
            for (const auto &tname : gcfg.term_names)
            {
                auto it = gcfg.terms.find(tname);
                if (it == gcfg.terms.end()) continue;
                const auto &t = it->second;
                std::cout << "    - " << tname << ": scale=";
                if (t.scale.size() == 1) std::cout << t.scale[0];
                else { std::cout << "["; for (size_t i = 0; i < t.scale.size(); ++i) std::cout << t.scale[i] << (i+1<t.scale.size()?", ":""); std::cout << "]"; }
                std::cout << ", clip=(" << t.clip_min << ", " << t.clip_max
                          << "), history=" << t.history_length << std::endl;
            }
        }
    }
};

// ---------------------------------------------------------------------------
// raw prop parsing (CSV: outer ',', inner ';')
// ---------------------------------------------------------------------------

namespace detail
{

inline std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> out;
    std::string cur;
    std::istringstream ss(s);
    while (std::getline(ss, cur, delim))
    {
        out.push_back(cur);
    }
    return out;
}

inline float parse_scalar(const std::string &tok)
{
    if (tok == "-inf") return -std::numeric_limits<float>::infinity();
    if (tok == "inf" || tok == "+inf") return std::numeric_limits<float>::infinity();
    return std::strtof(tok.c_str(), nullptr);
}

// one outer-CSV entry -> scalar or nested list
inline std::vector<float> parse_entry(const std::string &entry)
{
    std::vector<float> out;
    if (entry.find(';') != std::string::npos)
    {
        for (const auto &t : split(entry, ';'))
        {
            out.push_back(parse_scalar(t));
        }
    }
    else
    {
        out.push_back(parse_scalar(entry));
    }
    return out;
}

// full raw value -> vector of entries (each entry a scalar or list)
inline std::vector<std::vector<float>> parse_raw_numeric(const std::string &raw)
{
    std::vector<std::vector<float>> out;
    if (raw.empty()) return out;
    for (const auto &entry : split(raw, ','))
    {
        out.push_back(parse_entry(entry));
    }
    return out;
}

inline std::vector<std::string> parse_raw_strings(const std::string &raw)
{
    std::vector<std::string> out;
    if (raw.empty()) return out;
    for (const auto &entry : split(raw, ','))
    {
        out.push_back(entry);
    }
    return out;
}

inline bool parse_bool(const std::string &raw)
{
    std::string v = raw;
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return v == "true" || v == "1";
}

inline std::vector<float> as_float_list(const std::vector<std::vector<float>> &entries)
{
    std::vector<float> out;
    for (const auto &e : entries)
    {
        out.push_back(e.empty() ? 0.f : e[0]);
    }
    return out;
}

// obs_scale.<term> value: scalar or per-dim list
inline std::vector<float> parse_obs_scale(const std::string &raw)
{
    auto entries = parse_raw_numeric(raw);
    if (entries.empty()) return {1.0f};
    if (entries.size() == 1) return entries[0];
    std::vector<float> out;
    for (const auto &e : entries) out.push_back(e.empty() ? 0.f : e[0]);
    return out;
}

} // namespace detail

// ---------------------------------------------------------------------------
// public API
// ---------------------------------------------------------------------------

// Extract metadata_props from an onnxruntime session and parse them.
inline OnnxMetadata load_onnx_metadata(Ort::Session &session)
{
    Ort::AllocatorWithDefaultOptions allocator;
    Ort::ModelMetadata meta = session.GetModelMetadata();

    std::unordered_map<std::string, std::string> props;
    auto keys = meta.GetCustomMetadataMapKeysAllocated(allocator);
    for (const auto &key : keys)
    {
        auto value = meta.LookupCustomMetadataMapAllocated(key.get(), allocator);
        if (value) props[key.get()] = value.get();
    }

    if (props.find("observation_names") == props.end())
    {
        std::cerr << "[onnx_metadata] ONNX model has no 'observation_names' "
                     "metadata. The model may not have been exported with "
                     "mjlab's exporter_utils."
                  << std::endl;
        std::abort();
    }

    OnnxMetadata m;
    auto get = [&props](const std::string &k) -> const std::string & {
        static const std::string empty;
        auto it = props.find(k);
        return it == props.end() ? empty : it->second;
    };

    m.run_path = get("run_path");
    m.experiment_name = get("experiment_name");
    m.run_name = get("run_name");
    m.git_user_name = get("git_user_name");
    m.joint_names = detail::parse_raw_strings(get("joint_names"));
    m.joint_stiffness = detail::as_float_list(detail::parse_raw_numeric(get("joint_stiffness")));
    m.joint_damping = detail::as_float_list(detail::parse_raw_numeric(get("joint_damping")));
    m.default_joint_pos = detail::as_float_list(detail::parse_raw_numeric(get("default_joint_pos")));
    m.command_names = detail::parse_raw_strings(get("command_names"));
    m.observation_names = detail::parse_raw_strings(get("observation_names"));
    m.actor_obs_dim = static_cast<int>(std::strtol(get("actor_obs_dim").c_str(), nullptr, 10));
    m.actor_history_length = static_cast<int>(std::strtol(get("actor_history_length").c_str(), nullptr, 10));
    m.anchor_body_name = get("anchor_body_name");
    m.body_names = detail::parse_raw_strings(get("body_names"));

    // action_scale: scalar -> per-joint, or explicit list
    auto scale_entries = detail::parse_raw_numeric(get("action_scale"));
    if (scale_entries.size() <= 1)
    {
        float s = scale_entries.empty() ? 1.0f : (scale_entries[0].empty() ? 1.0f : scale_entries[0][0]);
        m.action_scale.assign(std::max(m.num_joints(), 1), s);
    }
    else
    {
        m.action_scale = detail::as_float_list(scale_entries);
    }

    // flatten history flag (with legacy key)
    if (props.count("actor_flatten_history_dim"))
        m.actor_flatten_history_dim = detail::parse_bool(get("actor_flatten_history_dim"));
    else if (props.count("observation_terms_flatten_history_dim"))
        m.actor_flatten_history_dim = detail::parse_bool(get("observation_terms_flatten_history_dim"));

    // per-term scales: "obs_scale.<name>" (new) or "observation_terms_scale" (legacy)
    std::unordered_map<std::string, std::vector<float>> scales;
    for (const auto &kv : props)
    {
        if (kv.first.rfind("obs_scale.", 0) == 0)
        {
            scales[kv.first.substr(10)] = detail::parse_obs_scale(kv.second);
        }
    }
    if (props.count("observation_terms_scale"))
    {
        auto legacy = detail::parse_raw_numeric(get("observation_terms_scale"));
        for (size_t i = 0; i < m.observation_names.size() && i < legacy.size(); ++i)
        {
            if (!scales.count(m.observation_names[i]))
                scales[m.observation_names[i]] = legacy[i];
        }
    }

    // per-term clips
    std::unordered_map<std::string, std::pair<float, float>> clips;
    if (props.count("observation_terms_clip"))
    {
        auto raw_clips = detail::parse_raw_numeric(get("observation_terms_clip"));
        for (size_t i = 0; i < m.observation_names.size() && i < raw_clips.size(); ++i)
        {
            const auto &c = raw_clips[i];
            if (c.size() >= 2)
                clips[m.observation_names[i]] = {c[0], c[1]};
        }
    }

    // per-term history lengths
    std::unordered_map<std::string, int> history;
    if (props.count("observation_terms_history_length"))
    {
        auto raw_hist = detail::parse_raw_numeric(get("observation_terms_history_length"));
        for (size_t i = 0; i < m.observation_names.size() && i < raw_hist.size(); ++i)
        {
            if (!raw_hist[i].empty())
                history[m.observation_names[i]] = static_cast<int>(raw_hist[i][0]);
        }
    }

    for (const auto &name : m.observation_names)
    {
        ObservationTermCfg cfg;
        cfg.name = name;
        auto sit = scales.find(name);
        if (sit != scales.end()) cfg.scale = sit->second;
        else cfg.scale = {1.0f};
        auto cit = clips.find(name);
        if (cit != clips.end())
        {
            cfg.clip_min = cit->second.first;
            cfg.clip_max = cit->second.second;
        }
        auto hit = history.find(name);
        if (hit != history.end()) cfg.history_length = hit->second;
        m.observation_terms[name] = std::move(cfg);
    }

    // ---- perception keys (depth / RNN / multi-input) ----
    m.onnx_input_names = detail::parse_raw_strings(get("onnx_input_names"));
    m.onnx_input_groups = detail::parse_raw_strings(get("onnx_input_groups"));
    m.onnx_output_names = detail::parse_raw_strings(get("onnx_output_names"));
    auto parse_shapes = [&](const std::string &key) {
        std::vector<std::vector<int64_t>> out;
        for (const auto &entry : detail::split(get(key), ','))
        {
            std::vector<int64_t> dims;
            for (const auto &tok : detail::split(entry, ';'))
                dims.push_back(static_cast<int64_t>(detail::parse_scalar(tok)));
            out.push_back(dims);
        }
        return out;
    };
    m.onnx_input_shapes = parse_shapes("onnx_input_shapes");
    m.onnx_output_shapes = parse_shapes("onnx_output_shapes");

    // obs groups: obs_group.<g>.*  (2 passes: per-term keys need term_names populated first;
    // props is unordered_map, so key order is not guaranteed)
    {
        std::map<std::string, ObsGroupCfg> groups;
        static const std::string prefix = "obs_group.";
        auto group_key = [&](const std::string &k, std::string &g, std::string &key) -> bool {
            if (k.rfind(prefix, 0) != 0) return false;
            std::string rest = k.substr(prefix.size());
            auto dot = rest.find('.');
            if (dot == std::string::npos) return false;
            g = rest.substr(0, dot);
            key = rest.substr(dot + 1);
            return true;
        };
        for (const auto &kv : props)
        {
            std::string g, key;
            if (!group_key(kv.first, g, key)) continue;
            auto &cfg = groups[g];
            if (key == "history_length")
                cfg.history_length = static_cast<int>(std::strtol(kv.second.c_str(), nullptr, 10));
            else if (key == "flatten_history_dim")
                cfg.flatten_history_dim = detail::parse_bool(kv.second);
            else if (key == "term_names")
                cfg.term_names = detail::parse_raw_strings(kv.second);
        }
        for (const auto &kv : props)
        {
            std::string g, key;
            if (!group_key(kv.first, g, key)) continue;
            auto &cfg = groups[g];
            if (key == "term_history_lengths")
            {
                auto raw = detail::parse_raw_numeric(kv.second);
                for (size_t i = 0; i < raw.size() && i < cfg.term_names.size(); ++i)
                    if (!raw[i].empty())
                        cfg.terms[cfg.term_names[i]].history_length = static_cast<int>(raw[i][0]);
            }
            else if (key == "term_clips")
            {
                auto raw = detail::parse_raw_numeric(kv.second);
                for (size_t i = 0; i < raw.size() && i < cfg.term_names.size(); ++i)
                {
                    auto &t = cfg.terms[cfg.term_names[i]];
                    if (raw[i].size() >= 2) { t.clip_min = raw[i][0]; t.clip_max = raw[i][1]; }
                }
            }
            else if (key == "term_scales")
            {
                auto raw = detail::parse_raw_numeric(kv.second);
                for (size_t i = 0; i < raw.size() && i < cfg.term_names.size(); ++i)
                {
                    auto &t = cfg.terms[cfg.term_names[i]];
                    t.name = cfg.term_names[i];
                    if (raw[i].empty()) t.scale = {1.0f};
                    else if (raw[i].size() == 1) t.scale = {raw[i][0]};
                    else t.scale = raw[i];
                }
            }
        }
        m.obs_groups = std::move(groups);
    }

    // depth camera params
    m.depth_image_height = static_cast<int>(std::strtol(get("depth_image_height").c_str(), nullptr, 10));
    m.depth_image_width = static_cast<int>(std::strtol(get("depth_image_width").c_str(), nullptr, 10));
    m.depth_fovy_ = detail::parse_scalar(get("depth_fovy"));
    m.depth_far_plane_ = detail::parse_scalar(get("depth_far_plane"));
    m.depth_cutoff_distance_ = detail::parse_scalar(get("depth_cutoff_distance"));
    m.depth_min_depth_ = detail::parse_scalar(get("depth_min_depth"));

    // RNN params
    m.rnn_type = get("rnn_type");
    m.rnn_hidden_dim = static_cast<int>(std::strtol(get("rnn_hidden_dim").c_str(), nullptr, 10));
    m.rnn_num_layers = static_cast<int>(std::strtol(get("rnn_num_layers").c_str(), nullptr, 10));

    return m;
}

} // namespace booster_deploy
