# Parkour 深度图策略集成 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Integrate the mjlab-trained depth-image parkour (越障) policy into go2_deploy, running in a new `PARKOUR` state, receiving depth via DDS `rt/depthimage` from MuJoCo (sim2sim), with deployment params driven by ONNX metadata.

**Architecture:** New `Go2ParkourController` class handles the 4-input/2-output GRU model (current_obs + obs_history + depth_image + hidden_state → actions + new_hidden_state). Depth arrives via a new `DepthImageSubscriber` (vendored `unitree_go::msg::dds_::DepthImage_` type), feeding a 2-frame ring buffer. `OnnxMetadata` and `ObservationBuilder` are extended to parse perception/obs-group keys and emit three separate tensors. State machine gains `STATES::PARKOUR` (L1+X enter, L1+Y stop). MuJoCo's C++ `DepthDisplay` is modified to publish 53×30 depth normalized identically to training (`clamp(d,0.105,2.5)/2.5`).

**Tech Stack:** C++17, onnxruntime 1.27.1 (vendored), unitree_sdk2 (CycloneDDS), yaml-cpp, OpenCV (new), MuJoCo (sim side).

---

**Spec:** `docs/superpowers/specs/2026-09-01-parkour-depth-policy-design.md`

**Reference files (read before starting):**
- `/home/lupinjia/test_scripts/depth_image_latency_cpp_unitree_sdk/DepthImage_.hpp` and `DepthImage_.cpp` — the DDS message to vendor
- `/home/lupinjia/test_scripts/depth_image_latency_cpp_unitree_sdk/rs_image_subscriber.hpp` — subscription pattern
- `/home/lupinjia/unitree_robotics/unitree_mujoco/simulate/src/utils/depth_display.hpp` — MuJoCo publisher to modify
- `/home/lupinjia/unitree_robotics/unitree_mujoco/simulate/src/mujoco/simulate.cc:2980-3035` — depth render/normalize to modify
- `/home/lupinjia/unitree_robotics/go2_deploy/models/2026-09-01_11-32-56_parkour_cts_depth_9000.onnx` — parkour model (4 inputs, 2 outputs)

---

## File Structure

**New files (go2_deploy):**
- `include/msg/DepthImage_.hpp` — vendored DDS message type (copy)
- `include/msg/DepthImage_.cpp` — vendored DDS type support (copy)
- `include/depth_image_subscriber.hpp` — DDS subscriber + 2-frame ring buffer
- `include/parkour_controller.hpp` — `Go2ParkourController` class
- `tests/test_metadata.cpp` — metadata parser unit test
- `tests/test_obs_builder.cpp` — obs builder unit test

**Modified files (go2_deploy):**
- `include/onnx_metadata.hpp` — add perception/obs_group/depth/rnn parsing
- `include/observation_builder.hpp` — `vel_command` alias, `compute_current()`, `compute_history()`, depth buffer
- `include/state_machine.hpp` — `STATES::PARKOUR`, `Parkour()` transition
- `include/robot_controller.hpp` — L1+X handling, `ParkourCallback`, PARKOUR branch in `ControlStep`, depth subscriber ownership
- `src/main.cpp` — controller factory by metadata presence
- `params/go2_config.yaml` — depth/parkour fields
- `CMakeLists.txt` — add `DepthImage_.cpp`, OpenCV, test target

**Modified files (unitree_mujoco):**
- `simulate/config.yaml` — depth 53×30, crop 0, enable_depth 1
- `simulate/src/mujoco/simulate.cc` — normalize to `clamp(d,0.105,2.5)/2.5`
- `simulate/src/utils/depth_display.hpp` — publish 53×30

---

## Task 1: Vendor DepthImage_ DDS message into go2_deploy

**Files:**
- Copy: `/home/lupinjia/test_scripts/depth_image_latency_cpp_unitree_sdk/DepthImage_.hpp` → `include/msg/DepthImage_.hpp`
- Copy: `/home/lupinjia/test_scripts/depth_image_latency_cpp_unitree_sdk/DepthImage_.cpp` → `include/msg/DepthImage_.cpp`
- Modify: `CMakeLists.txt:40-41`

- [ ] **Step 1: Copy the message files**

```bash
mkdir -p /home/lupinjia/unitree_robotics/go2_deploy/include/msg
cp /home/lupinjia/test_scripts/depth_image_latency_cpp_unitree_sdk/DepthImage_.hpp \
   /home/lupinjia/unitree_robotics/go2_deploy/include/msg/
cp /home/lupinjia/test_scripts/depth_image_latency_cpp_unitree_sdk/DepthImage_.cpp \
   /home/lupinjia/unitree_robotics/go2_deploy/include/msg/
```

- [ ] **Step 2: Add `DepthImage_.cpp` to the build**

In `CMakeLists.txt`, change:
```cmake
add_executable(go2_deploy 
src/main.cpp)
```
to:
```cmake
add_executable(go2_deploy 
src/main.cpp
include/msg/DepthImage_.cpp)
```

- [ ] **Step 3: Build**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy && rm -rf build && mkdir build && cd build && cmake .. && make -j16
```
Expected: build succeeds (the `get_type_props<::unitree_go::msg::dds_::DepthImage_>` specialization compiles and links).

- [ ] **Step 4: Commit**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy
git add include/msg/DepthImage_.hpp include/msg/DepthImage_.cpp CMakeLists.txt
git commit -m "[feat]: vendor DepthImage_ DDS message type for parkour depth input"
```

---

## Task 2: Extend OnnxMetadata with perception/obs_group/depth/rnn parsing

**Files:**
- Modify: `include/onnx_metadata.hpp`
- Create: `tests/test_metadata.cpp`

The parkour model carries extra metadata keys beyond the blind-model base keys. Confirmed values (from the model):
```
onnx_input_names:   current_obs,obs_history,depth_image,hidden_state
onnx_input_shapes:  1.000;45.000,1.000;900.000,1.000;2.000;30.000;53.000,1.000;1.000;512.000
onnx_input_groups:  actor,history,depth_image,state
onnx_output_names:  actions,new_hidden_state
onnx_output_shapes: 1.000;12.000,1.000;1.000;512.000
obs_group.actor.history_length: 0
obs_group.actor.term_names: base_ang_vel,projected_gravity,vel_command,joint_pos,joint_vel,actions
obs_group.actor.term_scales: 0.250,1.000,1.000;1.000;0.250,1.000,0.050,0.200
obs_group.history.history_length: 20
obs_group.depth_image.history_length: 2
obs_group.depth_image.flatten_history_dim: False
depth_image_height: 30   depth_image_width: 53
depth_fovy: 58   depth_far_plane: 5.0   depth_cutoff_distance: 2.5   depth_min_depth: 0.105
rnn_type: gru   rnn_hidden_dim: 512   rnn_num_layers: 1
```

- [ ] **Step 1: Write the failing test**

Create `tests/test_metadata.cpp`:

```cpp
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include "onnx_metadata.hpp"

static int failures = 0;
#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::cerr << "FAIL: " << msg << " (line " << __LINE__ << ")\n";  \
            ++failures;                                                      \
        }                                                                    \
    } while (0)

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: test_metadata <model.onnx>\n";
        return 2;
    }
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
    Ort::SessionOptions opts;
    Ort::Session session(env, argv[1], opts);
    auto md = booster_deploy::load_onnx_metadata(session);

    CHECK(md.has_perception(), "has_perception() should be true for parkour model");
    CHECK(md.onnx_input_names.size() == 4, "expect 4 inputs");
    CHECK(md.onnx_input_names[0] == "current_obs", "input0 name");
    CHECK(md.onnx_input_names[2] == "depth_image", "input2 name");
    CHECK(md.onnx_output_names.size() == 2, "expect 2 outputs");
    CHECK(md.onnx_output_names[0] == "actions", "output0 name");
    CHECK(md.input_shape("current_obs").size() == 2, "current_obs rank 2");
    CHECK(md.input_shape("current_obs")[1] == 45, "current_obs dim1 == 45");
    CHECK(md.input_shape("obs_history")[1] == 900, "obs_history dim1 == 900");
    auto dshape = md.input_shape("depth_image");
    CHECK(dshape.size() == 4, "depth rank 4");
    CHECK(dshape[1] == 2 && dshape[2] == 30 && dshape[3] == 53, "depth [1,2,30,53]");
    CHECK(md.depth_width() == 53, "depth width 53");
    CHECK(md.depth_height() == 30, "depth height 30");
    CHECK(md.depth_cutoff_distance() == 2.5f, "cutoff 2.5");
    CHECK(md.depth_min_depth() == 0.105f, "min depth 0.105");
    CHECK(md.rnn_type == "gru", "rnn type gru");
    CHECK(md.rnn_hidden_dim == 512, "rnn hidden 512");
    const auto& actor = md.obs_group("actor");
    CHECK(actor.history_length == 0, "actor history 0");
    CHECK(actor.term_names.size() == 6, "actor 6 terms");
    CHECK(actor.term_names[2] == "vel_command", "actor term[2] vel_command");
    CHECK(actor.term("joint_pos").scale.size() == 1, "joint_pos scale len 1");
    CHECK(actor.term("vel_command").scale.size() == 3, "vel_command per-dim scale 3");
    const auto& hist = md.obs_group("history");
    CHECK(hist.history_length == 20, "history group length 20");
    CHECK(hist.term("joint_pos").history_length == 20, "history term hist 20");
    const auto& dg = md.obs_group("depth_image");
    CHECK(dg.history_length == 2, "depth group history 2");
    CHECK(!dg.flatten_history_dim, "depth group not flattened");

    if (failures == 0)
        std::cout << "ALL PASS\n";
    return failures == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Add the test target to CMake**

In `CMakeLists.txt`, after the `go2_deploy` target block, add:

```cmake
add_executable(test_metadata tests/test_metadata.cpp)
target_link_libraries(test_metadata
    yaml-cpp
    ${ONNXRUNTIME_LIBRARY}
    pthread
    ${CMAKE_DL_LIBS}
    )
set_target_properties(test_metadata PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}"
    INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    )
```

- [ ] **Step 3: Build and run test — verify it FAILS**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy/build && cmake .. && make test_metadata -j16 2>&1 | tail -5
```
Expected: compile fails (`OnnxMetadata` has no `has_perception` / `obs_group` / `input_shape` / depth getters). This is the failing test.

- [ ] **Step 4: Implement the metadata extensions**

In `include/onnx_metadata.hpp`, after `struct ObservationTermCfg` (line 41), add:

```cpp
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
```

Inside `struct OnnxMetadata` (after `body_names`), add:

```cpp
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
```

> Requires `#include <map>` at the top of `onnx_metadata.hpp`.

In `load_onnx_metadata()`, after the base-key parsing and before `return m;`, add the perception parsing:

```cpp
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

    // obs groups: obs_group.<g>.*
    {
        std::map<std::string, ObsGroupCfg> groups;
        for (const auto &kv : props)
        {
            static const std::string prefix = "obs_group.";
            if (kv.first.rfind(prefix, 0) != 0) continue;
            std::string rest = kv.first.substr(prefix.size());
            auto dot = rest.find('.');
            if (dot == std::string::npos) continue;
            std::string g = rest.substr(0, dot);
            std::string key = rest.substr(dot + 1);
            auto &cfg = groups[g];
            if (key == "history_length")
                cfg.history_length = static_cast<int>(std::strtol(kv.second.c_str(), nullptr, 10));
            else if (key == "flatten_history_dim")
                cfg.flatten_history_dim = detail::parse_bool(kv.second);
            else if (key == "term_names")
                cfg.term_names = detail::parse_raw_strings(kv.second);
            else if (key == "term_history_lengths")
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
```

- [ ] **Step 5: Build and run test — verify it PASSES**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy/build && cmake .. && make test_metadata -j16 && ./test_metadata ../models/2026-09-01_11-32-56_parkour_cts_depth_9000.onnx
```
Expected: `ALL PASS` and exit 0.

- [ ] **Step 6: Commit**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy
git add include/onnx_metadata.hpp tests/test_metadata.cpp CMakeLists.txt
git commit -m "[feat]: parse perception/obs_group/depth/rnn ONNX metadata keys"
```

---

## Task 3: Extend ObservationBuilder with vel_command alias + current/history/depth outputs

**Files:**
- Modify: `include/observation_builder.hpp`
- Create: `tests/test_obs_builder.cpp`

The blind model uses obs term `command`; the parkour model uses `vel_command` (per-dim scale `[1,1,0.25]`). The parkour policy needs three separate tensors: `current_obs` (no history, term-major), `obs_history` (20-frame per-term term-major), `depth_image` (2-frame channel-stacked).

- [ ] **Step 1: Write the failing test**

Create `tests/test_obs_builder.cpp`:

```cpp
#include <cstdlib>
#include <iostream>
#include <vector>

#include "observation_builder.hpp"
#include "onnx_metadata.hpp"

static int failures = 0;
#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::cerr << "FAIL: " << msg << " (line " << __LINE__ << ")\n";  \
            ++failures;                                                      \
        }                                                                    \
    } while (0)

// Minimal fake robot: 12 joints in real order, plus the fields used by terms.
struct FakeRobot
{
    std::vector<float> jpos{12, 0.f}, jvel{12, 0.f}, tau{12, 0.f};
    std::array<float, 4> quat{1,0,0,0};
    std::array<float, 3> rpy{0,0,0}, gyro{0.1f,0.2f,0.3f}, projected_gravity{0,0,-1.f}, acc{0,0,0};
    const std::vector<std::string> &joint_names() const { static const std::vector<std::string> n = {
        "FR_hip_joint","FR_thigh_joint","FR_calf_joint",
        "FL_hip_joint","FL_thigh_joint","FL_calf_joint",
        "RR_hip_joint","RR_thigh_joint","RR_calf_joint",
        "RL_hip_joint","RL_thigh_joint","RL_calf_joint"}; return n; }
};

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: test_obs_builder <model.onnx>\n";
        return 2;
    }
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
    Ort::SessionOptions opts;
    Ort::Session session(env, argv[1], opts);
    auto md = booster_deploy::load_onnx_metadata(session);

    // Build a builder with a fake robot. The real ctor takes Go2RobotInterface&,
    // so we temporarily use the actual class with real robot joint names.
    auto real = unitree::common::go2_joint_names();
    unitree::common::Go2RobotInterface robot(real);
    robot.jpos.assign(12, 0.1f);
    robot.jvel.assign(12, 0.05f);
    robot.gyro = {0.1f, 0.2f, 0.3f};
    robot.projected_gravity = {0.f, 0.f, -1.f};

    booster_deploy::ObservationBuilder builder(md, robot);
    builder.command_ = {0.5f, -0.2f, 0.1f};
    builder.last_action_.assign(12, 0.3f);

    auto cur = builder.compute_current();
    CHECK(static_cast<int>(cur.size()) == 45, "current_obs size == 45");
    // term order: base_ang_vel(3), projected_gravity(3), vel_command(3), joint_pos(12), joint_vel(12), actions(12)
    CHECK(std::abs(cur[0] - 0.1f * 0.25f) < 1e-5f, "base_ang_vel[0] scaled by 0.25");
    CHECK(std::abs(cur[6] - 0.5f) < 1e-5f, "vel_command[0] == cmd x (scale 1)");
    CHECK(std::abs(cur[8] - 0.1f * 0.25f) < 1e-5f, "vel_command[2] == cmd z x 0.25");
    CHECK(std::abs(cur[9] - (0.1f - 0.0f)) < 1e-5f, "joint_pos[0] rel default, scale 1");
    CHECK(std::abs(cur[21] - 0.05f * 0.05f) < 1e-5f, "joint_vel[0] scaled by 0.05");
    CHECK(std::abs(cur[33] - 0.3f * 0.2f) < 1e-5f, "actions[0] scaled by 0.2");

    auto hist = builder.compute_history();
    CHECK(static_cast<int>(hist.size()) == 900, "obs_history size == 900");
    // first-frame backfill: frame0 == frame1 == current values
    CHECK(std::abs(hist[0] - cur[0]) < 1e-5f, "history[0] == current[0] (backfill)");
    CHECK(std::abs(hist[45] - cur[0]) < 1e-5f, "history[45] == current[0] (frame1)");
    CHECK(std::abs(hist[60*19] - cur[0]) < 1e-5f, "last frame base_ang_vel[0] backfilled");

    // depth buffer: set two frames, expect [2,30,53]
    std::vector<float> frame(30 * 53, 0.5f);
    builder.set_depth_frame(frame.data(), frame.size());
    builder.set_depth_frame(frame.data(), frame.size());
    auto depth = builder.depth_input();
    CHECK(static_cast<int>(depth.size()) == 2 * 30 * 53, "depth_input size == 3180");
    CHECK(std::abs(depth[0] - 0.5f) < 1e-5f, "depth ch0 == 0.5");
    CHECK(std::abs(depth[30*53] - 0.5f) < 1e-5f, "depth ch1 == 0.5");

    if (failures == 0)
        std::cout << "ALL PASS\n";
    return failures == 0 ? 0 : 1;
}
```

> The fake-robot approach above is intentionally simplified: `ObservationBuilder` takes `unitree::common::Go2RobotInterface &`, so the test uses the real class with `go2_joint_names()` and sets its public state members directly. `compute_current`/`compute_history`/`set_depth_frame`/`depth_input` must exist (they do not yet — this is the failing test).

- [ ] **Step 2: Add the test target to CMake**

In `CMakeLists.txt` after the `test_metadata` block:

```cmake
add_executable(test_obs_builder tests/test_obs_builder.cpp)
target_link_libraries(test_obs_builder
    yaml-cpp
    ${ONNXRUNTIME_LIBRARY}
    pthread
    ${CMAKE_DL_LIBS}
    )
set_target_properties(test_obs_builder PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}"
    INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    )
```

- [ ] **Step 3: Build and run test — verify it FAILS**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy/build && cmake .. && make test_obs_builder -j16 2>&1 | tail -5
```
Expected: compile fails (`compute_current` / `compute_history` / `set_depth_frame` / `depth_input` not declared).

- [ ] **Step 4: Implement the builder extensions**

In `include/observation_builder.hpp`:

(4a) In `compute_raw()`, change the command term to accept `vel_command` as an alias:

```cpp
        if (name == "command" || name == "vel_command")
        {
            return command_;
        }
```

(4b) In `compute_raw_dim()`, change:

```cpp
        if (name == "command" || name == "base_ang_vel" || name == "projected_gravity")
            return 3;
```
to:
```cpp
        if (name == "command" || name == "vel_command" || name == "base_ang_vel" || name == "projected_gravity")
            return 3;
```

(4c) Add public methods after `compute()`:

```cpp
    // Current obs (no history): term-major, each term clip+scale.
    // Uses the "actor" obs group term order/scales.
    std::vector<float> compute_current()
    {
        return compute_group_terms("actor", /*history=*/false);
    }

    // Full history obs (term-major, per-term history). Uses the "history"
    // obs group; each term is flattened oldest->newest.
    std::vector<float> compute_history()
    {
        return compute_group_terms("history", /*history=*/true);
    }

    // Set the latest depth frame (already normalized [0,1] 30x53). Keeps a
    // 2-frame ring buffer (oldest in channel 0, newest in channel 1).
    void set_depth_frame(const float *data, size_t n)
    {
        const int hw = metadata_.depth_height() * metadata_.depth_width();
        if (static_cast<int>(n) != hw) return;
        std::vector<float> frame(data, data + n);
        depth_frames_.push_back(std::move(frame));
        while (static_cast<int>(depth_frames_.size()) > depth_history_len_)
            depth_frames_.pop_front();
    }

    // Depth input tensor [depth_history, H, W], channel-stacked (old->new).
    std::vector<float> depth_input()
    {
        std::vector<float> out;
        int hw = metadata_.depth_height() * metadata_.depth_width();
        out.reserve(depth_frames_.size() * hw);
        for (const auto &f : depth_frames_)
            out.insert(out.end(), f.begin(), f.end());
        return out;
    }
```

(4d) Add private helpers:

```cpp
    // Compute terms of an obs group; optionally apply per-term history.
    std::vector<float> compute_group_terms(const std::string &group_name, bool with_history)
    {
        const auto &g = metadata_.obs_group(group_name);
        std::vector<float> out;
        for (const auto &name : g.term_names)
        {
            std::vector<float> term_obs = compute_raw(name);
            const ObservationTermCfg *cfg = g.term(name);
            if (!std::isinf(cfg->clip_min) || !std::isinf(cfg->clip_max))
                for (float &v : term_obs)
                    v = std::max(cfg->clip_min, std::min(cfg->clip_max, v));
            for (size_t i = 0; i < term_obs.size(); ++i)
                term_obs[i] *= cfg->scale[std::min(i, cfg->scale.size() - 1)];
            if (with_history && cfg->history_length > 0)
                term_obs = push_history(name, std::move(term_obs), cfg->history_length);
            out.insert(out.end(), term_obs.begin(), term_obs.end());
        }
        return out;
    }
```

(4e) Add private members:

```cpp
    std::deque<std::vector<float>> depth_frames_;
    int depth_history_len_ = 2;
```

> NOTE: `compute()` (the blind-policy single-vector path) stays unchanged; `compute_current`/`compute_history` are the new parkour paths. The existing `compute_term`/`push_history` are reused.

- [ ] **Step 5: Build and run test — verify it PASSES**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy/build && cmake .. && make test_obs_builder -j16 && ./test_obs_builder ../models/2026-09-01_11-32-56_parkour_cts_depth_9000.onnx
```
Expected: `ALL PASS` and exit 0.

- [ ] **Step 6: Commit**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy
git add include/observation_builder.hpp tests/test_obs_builder.cpp CMakeLists.txt
git commit -m "[feat]: obs builder vel_command alias + current/history/depth outputs for parkour"
```

---

## Task 4: DepthImageSubscriber (DDS depth receive + safe frame copy)

**Files:**
- Create: `include/depth_image_subscriber.hpp`

- [ ] **Step 1: Write the header**

Create `include/depth_image_subscriber.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "unitree/robot/channel/channel_subscriber.hpp"

#include "msg/DepthImage_.hpp"

namespace unitree::common
{

// Subscribes to MuJoCo's depth image on `rt/depthimage`
// (unitree_go::msg::dds_::DepthImage_ : width, height, normalized_value [0,1]).
// The callback runs on the DDS reader thread; the frame is copied out under a
// mutex. The publisher (Task 7) outputs exactly 53x30 already-normalized
// frames, so no denormalize/resize happens here.
class DepthImageSubscriber
{
public:
    DepthImageSubscriber(int domain_id, const std::string &interface,
                         const std::string &topic = "rt/depthimage")
    {
        sub_.reset(new ChannelSubscriber<unitree_go::msg::dds_::DepthImage_>(topic));
        sub_->InitChannel(
            std::bind(&DepthImageSubscriber::onDepthImage, this, std::placeholders::_1), 1);
    }

    bool latest_frame(std::vector<float> &out, uint16_t &width, uint16_t &height)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!have_frame_) return false;
        out = frame_;
        width = width_;
        height = height_;
        return true;
    }

private:
    void onDepthImage(const void *data)
    {
        const auto *msg = static_cast<const unitree_go::msg::dds_::DepthImage_ *>(data);
        std::lock_guard<std::mutex> lock(mtx_);
        width_ = msg->width();
        height_ = msg->height();
        frame_.assign(msg->normalized_value().begin(), msg->normalized_value().end());
        have_frame_ = true;
    }

    ChannelSubscriberPtr<unitree_go::msg::dds_::DepthImage_> sub_;
    std::mutex mtx_;
    std::vector<float> frame_;
    uint16_t width_ = 0;
    uint16_t height_ = 0;
    bool have_frame_ = false;
};

} // namespace unitree::common
```

- [ ] **Step 2: Commit** (header-only; built when included by the robot controller)

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy
git add include/depth_image_subscriber.hpp
git commit -m "[feat]: DepthImageSubscriber for rt/depthimage"
```

---

## Task 5: Go2ParkourController (4-input / 2-output GRU inference)

**Files:**
- Create: `include/parkour_controller.hpp`

This class reuses `Go2OnnxController`'s yaml loading, joint-config mapping, PD mapping and command reading. It differs only in inference: 4 inputs (current_obs, obs_history, depth_image, hidden_state) → 2 outputs (actions, new_hidden_state), with GRU hidden-state feedback.

- [ ] **Step 1: Write the controller header (part 1 — class + load)**

Create `include/parkour_controller.hpp`:

```cpp
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
    void set_depth_frame(const float *data, size_t n)
    {
        if (obs_builder_) obs_builder_->set_depth_frame(data, n);
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
```

- [ ] **Step 2: Build**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy/build && make -j16 2>&1 | tail -5
```
Expected: builds (header only; not yet included by main — compile check happens when wired in Task 6/8. If `parkour_controller.hpp` is not referenced anywhere it may not compile yet; that is expected until Task 8 wires it.)

- [ ] **Step 3: Commit**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy
git add include/parkour_controller.hpp
git commit -m "[feat]: Go2ParkourController (4-input/2-output GRU inference)"
```

---

## Task 6: State machine PARKOUR state + robot controller wiring

**Files:**
- Modify: `include/state_machine.hpp`
- Modify: `include/robot_controller.hpp`

- [ ] **Step 1: Add STATES::PARKOUR**

In `include/state_machine.hpp`, change the enum:

```cpp
    enum class STATES
    {
        DAMPING = 0,
        SIT = 1,
        STAND = 2,
        CTRL = 3,
        PARKOUR = 4
    };
```

- [ ] **Step 2: Add Parkour() transition to RLStateMachine**

In `include/state_machine.hpp`, inside `class RLStateMachine`, after `Stand()`:

```cpp
        bool Parkour() // enter parkour from STAND (parallel to Ctrl)
        {
            if (state == STATES::STAND)
            {
                state = STATES::PARKOUR;
                return true;
            }
            else
            {
                return false;
            }
        }
```

- [ ] **Step 3: Wire L1+X into robot_controller UpdateStateMachine**

In `include/robot_controller.hpp`, in `UpdateStateMachine()`, add after the Ctrl block:

```cpp
        if (gamepad.X.on_press && gamepad.L1.pressed)
        {
            if (state_machine.Parkour())
            {
                ParkourCallback();
            }
        }
```

> The existing `L1 + Y -> Stop` already returns to DAMPING and is reused for leaving parkour.

- [ ] **Step 4: Add ParkourCallback**

In `include/robot_controller.hpp`, after `CtrlCallback()`:

```cpp
    void ParkourCallback()
    {
        // reset controller: RNN hidden_state, obs history, last_action
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
```

- [ ] **Step 5: Add PARKOUR branch in ControlStep**

In `include/robot_controller.hpp`, in `ControlStep()`, after the CTRL block:

```cpp
        if (state_machine.state == STATES::PARKOUR)
        {
            // feed the latest depth frame (if any) into the controller
            std::vector<float> depth;
            uint16_t dw = 0, dh = 0;
            if (depth_subscriber_ && depth_subscriber_->latest_frame(depth, dw, dh))
            {
                ctrl->set_depth_frame(depth.data(), depth.size());
            }
            UserControlStep(true);
            if (CheckTermination())
            {
                state_machine.Stop();
                Damping();
            }
        }
```

- [ ] **Step 6: Add depth subscriber member + init**

In `include/robot_controller.hpp`:
- Add `#include "depth_image_subscriber.hpp"` and `#include "parkour_controller.hpp"` at the top (after the existing includes).
- Add a protected member:

```cpp
    std::unique_ptr<DepthImageSubscriber> depth_subscriber_;
```

- In `InitDdsModel()`, after the lowstate subscriber init, add (only for parkour; guarded by config):

```cpp
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
        }
        catch (...) {}
        if (depth_enabled)
        {
            depth_subscriber_.reset(new DepthImageSubscriber(depth_domain, depth_interface));
        }
```

> `config_path_` is a new member holding `param / "go2_config.yaml"`; set it in a constructor or `loadParam`. The subscriber calls `ChannelFactory::Instance()->Init(...)` internally.

- [ ] **Step 7: Add `ctrl->set_depth_frame` and `ctrl->reset` to BasicUserController interface**

In `include/user_controller.hpp`, add to `BasicUserController`:

```cpp
    virtual void set_depth_frame(const float *data, size_t n) { (void)data; (void)n; }
```

(Default no-op so `Go2OnnxController` is unaffected; `Go2ParkourController` overrides it. `reset` is already virtual.)

- [ ] **Step 8: Build**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy/build && make -j16 2>&1 | tail -8
```
Expected: builds.

- [ ] **Step 9: Commit**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy
git add include/state_machine.hpp include/robot_controller.hpp include/user_controller.hpp
git commit -m "[feat]: PARKOUR state (L1+X) + depth subscriber wiring"
```

---

## Task 7: MuJoCo side — publish 53×30 depth normalized like training

**Files (unitree_mujoco repo):**
- Modify: `simulate/config.yaml`
- Modify: `simulate/src/mujoco/simulate.cc`
- Modify: `simulate/src/utils/depth_display.hpp`

> NOTE: These edits are in `/home/lupinjia/unitree_robotics/unitree_mujoco`, a **separate repo** from go2_deploy. Commit there separately.

- [ ] **Step 1: Config — 53×30, no crop, enable**

In `simulate/config.yaml`, change:

```yaml
enable_depth: 0
depth_width: 320
depth_height: 240
crop_left: 8
```
to:
```yaml
enable_depth: 1
depth_width: 53
depth_height: 30
crop_left: 0
```

> Keep `near_clip`/`far_clip` as-is if present (the normalize formula below is replaced, so they become unused for the published value — leave them for the raw-buffer clip if the code still references them; see Step 2).

- [ ] **Step 2: simulate.cc — replace the [0,1] normalize with training normalization**

In `simulate/src/mujoco/simulate.cc`, the depth block currently does (lines ~3014-3027):

```cpp
          depth_value_[val_index] = depth_buffer_[buf_index];
          depth_value_[val_index] = near / (1.0f - depth_value_[val_index] * (1.0f - near / far) );
          // clip depth value according to near_clip and far_clip (set in config)
          if (depth_value_[val_index] < near_clip_)
          {
            depth_value_[val_index] = near_clip_;
          }
          if (depth_value_[val_index] > far_clip_)
          {
            depth_value_[val_index] = far_clip_;
          }
          // normalize to [0,1]
          depth_value_[val_index] = (depth_value_[val_index] - near_clip_) 
                                    / (far_clip_ - near_clip_);
```

Replace the whole inner body with the training-identical normalization:

```cpp
          // convert raw [0,1] depth buffer to metric depth (m)
          depth_value_[val_index] = near / (1.0f - depth_value_[val_index] * (1.0f - near / far) );
          // training-identical normalization: clamp(depth, 0.105, 2.5) / 2.5
          const float kMinDepth = 0.105f;
          const float kCutoff   = 2.5f;
          if (depth_value_[val_index] < kMinDepth)
          {
            depth_value_[val_index] = kMinDepth;
          }
          if (depth_value_[val_index] > kCutoff)
          {
            depth_value_[val_index] = kCutoff;
          }
          depth_value_[val_index] = depth_value_[val_index] / kCutoff;
```

> This matches mjlab's `camera_depth()`: `clamp(depth_m, 0.105, 2.5) / 2.5` → [0,1]. `near_clip_`/`far_clip_` are no longer used for the published value; they can stay (harmless) or be removed if the compiler warns about unused members.

- [ ] **Step 3: depth_display.hpp — publish 53×30 (already uses width_/height_ from config)**

`depth_display.hpp` builds `depth_image(height_, width_ - crop_left_, CV_32FC1)` from `depth_value_` and publishes `normalized_value` — with the config in Step 1 (`width_=53, height_=30, crop_left_=0`) this is already 53×30. **Verify** `depth_value_` size matches `(53-0)*30` in `simulate.cc` (`new float[(depth_width_ - depth_crop_left_) * depth_height_]` → 53*30). No code change needed unless the `cv::resize` path is desired; the direct 53×30 render is simpler.

> If the 53×30 render sub-rect is too small to view in the window, optionally keep rendering at 320×240 and `cv::resize(depth_image, depth_image, cv::Size(53, 30))` just before publishing — but the direct config change is sufficient for sim2sim.

- [ ] **Step 4: Build MuJoCo**

```bash
cd /home/lupinjia/unitree_robotics/unitree_mujoco && mkdir -p build && cd build && cmake .. && make -j16 2>&1 | tail -8
```
Expected: builds.

- [ ] **Step 5: Commit (unitree_mujoco repo)**

```bash
cd /home/lupinjia/unitree_robotics/unitree_mujoco
git add simulate/config.yaml simulate/src/mujoco/simulate.cc simulate/src/utils/depth_display.hpp
git commit -m "[feat]: publish 53x30 depth normalized like training (clamp/2.5)"
```

---

## Task 8: Controller factory (metadata-driven), config yaml, CMake/OpenCV

**Files:**
- Modify: `src/main.cpp`
- Modify: `params/go2_config.yaml`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the controller factory in main.cpp**

In `src/main.cpp`, add a helper that decides between `Go2OnnxController` and `Go2ParkourController` based on whether the model has perception metadata. Since reading metadata needs a session, the factory loads the model once, checks, then constructs the right controller (which loads its own session again in `loadPolicy` — acceptable at startup).

Add after the includes:

```cpp
#include "parkour_controller.hpp"

#include <onnxruntime_cxx_api.h>

// Decide controller type from the ONNX metadata: presence of onnx_input_groups
// (perception/depth/RNN model) selects Go2ParkourController, else the blind
// Go2OnnxController.
static BasicUserController *create_controller(const std::string &cfg_file,
                                              const std::vector<std::string> &joint_names)
{
    auto cfg = YAML::LoadFile(cfg_file);
    std::string policy_name = cfg["policy_name"].as<std::string>();
    fs::path policy_file = policy_name;
    if (policy_file.is_relative())
        policy_file = fs::current_path() / "../models" / policy_name;
    if (!fs::exists(policy_file))
    {
        std::cerr << "policy not found: " << policy_file << std::endl;
        std::exit(EXIT_FAILURE);
    }

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "go2_deploy_factory");
    Ort::SessionOptions opts;
    Ort::Session session(env, policy_file.c_str(), opts);
    auto md = booster_deploy::load_onnx_metadata(session);

    if (md.has_perception())
    {
        std::cout << "[main] perception model -> Go2ParkourController" << std::endl;
        return new Go2ParkourController(cfg_file, joint_names);
    }
    std::cout << "[main] blind model -> Go2OnnxController" << std::endl;
    return new Go2OnnxController(cfg_file, joint_names);
}
```

Then in `main()`, replace:

```cpp
    BasicUserController* user_ctrl = new Go2OnnxController(config_file_dir, go2_joint_names());
```
with:
```cpp
    BasicUserController* user_ctrl = create_controller(config_file_dir, go2_joint_names());
```

- [ ] **Step 2: Add depth/parkour fields to params/go2_config.yaml**

Append to `params/go2_config.yaml`:

```yaml
# --- parkour / depth policy (sim2sim) ---
# depth DDS subscription; sim2sim uses domain 1 + "lo", real robot domain 0 + NIC
depth_enabled: true
depth_domain: 1
depth_interface: "lo"
depth_topic: "rt/depthimage"
```

> Switch to the parkour model by setting `policy_name: "2026-09-01_11-32-56_parkour_cts_depth_9000.onnx"` (the factory auto-selects the controller). Keep `policy_name: "go2_rough.onnx"` to use the blind CTRL policy.

- [ ] **Step 3: CMake — add parkour controller source (header-only) + OpenCV**

In `CMakeLists.txt`, after `find_package(unitree_sdk2 REQUIRED)`, add OpenCV:

```cmake
find_package(OpenCV REQUIRED)
```

Add OpenCV to the link libraries:

```cmake
target_link_libraries(go2_deploy
    unitree_sdk2 
    yaml-cpp
    ${ONNXRUNTIME_LIBRARY}
    ${OpenCV_LIBS}
    pthread
    ${CMAKE_DL_LIBS}
    )
```

> `parkour_controller.hpp` and `depth_image_subscriber.hpp` are header-only; they are compiled when included by `main.cpp`/`robot_controller.hpp`. OpenCV is linked because the design decided to use it for depth handling; if no OpenCV symbol is actually used yet, linking is still required to satisfy `-l` (and future resize). If the Jetson lacks OpenCV, use the pure-float path (no `cv::` calls) and drop the OpenCV link — flag this in review.

- [ ] **Step 4: Build + run (blind model regression + parkour load)**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy/build && cmake .. && make -j16 2>&1 | tail -8
```

Run with the parkour model to verify factory + metadata path:

```bash
# temporarily point go2_config.yaml policy_name at the parkour model, then:
timeout 6 ./go2_deploy lo 2>&1 | grep -iE "perception|parkour|policy loaded|error|assert" | head
```
Expected: `[main] perception model -> Go2ParkourController`, `[parkour] policy loaded ... inputs: current_obs obs_history depth_image hidden_state`, no assert. Then restore `policy_name: "go2_rough.onnx"` and rerun to confirm the blind path still selects `Go2OnnxController`.

- [ ] **Step 5: Commit**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy
git add src/main.cpp params/go2_config.yaml CMakeLists.txt
git commit -m "[feat]: metadata-driven controller factory + depth config + OpenCV"
```

---

## Task 9: sim2sim integration test (MuJoCo + go2_deploy)

**Files:**
- Modify: `params/go2_config.yaml` (point at the parkour model)
- Test-only: no source changes

This is the end-to-end verification. It requires a display (MuJoCo UI) and the DDS loopback.

- [ ] **Step 1: Set config to the parkour model**

In `params/go2_config.yaml`, set:

```yaml
policy_name: "2026-09-01_11-32-56_parkour_cts_depth_9000.onnx"
```

- [ ] **Step 2: Launch MuJoCo (with depth enabled)**

```bash
cd /home/lupinjia/unitree_robotics/unitree_mujoco/build && ./unitree_mujoco ../simulate/config.yaml 2>&1 | tail -5
```
Expected: prints `DepthDisplay: initialized depth image publisher on topic rt/depthimage` and opens the sim window. (Adjust the config path/args to match how the sim is normally launched.)

- [ ] **Step 3: Launch go2_deploy (domain 1, lo)**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy/build && ./go2_deploy 2>&1 | tee /tmp/parkour_run.log
```
Expected:
- `[main] perception model -> Go2ParkourController`
- `[parkour] policy loaded ... inputs: current_obs obs_history depth_image hidden_state; rnn=gru 512`
- reaches `Press START button to start!`

- [ ] **Step 4: Verify depth frames arrive + PARKOUR works**

While the sim runs, in the go2_deploy window:
1. Check the depth image appears (MuJoCo window shows a 53×30 depth view).
2. Press `L1 + X` (via the virtual gamepad / connected controller) → expect `Parkour enabled.` and the robot standing under the parkour policy.
3. Confirm `Performance: mean ... ms` stays within the 50 Hz budget (~20 ms) with depth inference included.

If depth frames are not reaching the controller, add a temporary `std::cerr` in `DepthImageSubscriber::onDepthImage` counting frames (or check `latest_frame` returns true) — the most likely failure is a DDS type/domain mismatch between the MuJoCo publisher (domain 1, lo) and the subscriber config.

- [ ] **Step 5: Blind-policy regression**

Restore `policy_name: "go2_rough.onnx"` and `depth_enabled: false` in `params/go2_config.yaml`, rebuild if needed, rerun:
```bash
cd /home/lupinjia/unitree_robotics/go2_deploy/build && ./go2_deploy
```
Expected: `[main] blind model -> Go2OnnxController`, existing CTRL flow unchanged (no parkour state, no depth subscriber).

- [ ] **Step 6: Commit (config restore)**

```bash
cd /home/lupinjia/unitree_robotics/go2_deploy
git add params/go2_config.yaml
git commit -m "[test]: parkour sim2sim integration verified (blind regression passes)"
```

---

## Self-Review (completed by plan author)

- **Spec coverage:** All 9 spec sections map to tasks: metadata (T2), obs builder (T3), parkour controller (T5), depth subscriber (T4), state machine (T6), MuJoCo publish side (T7), config/main factory (T8), build/CMake/OpenCV (T1+T8), sim2sim test (T9). No spec requirement left uncovered.
- **Placeholder scan:** No TBD/TODO; every code step is complete. The one flagged deviation (`DummyCalculate` history dim placeholder) was removed — `DummyCalculate` now builds zero tensors from input shapes directly.
- **Type consistency:** `compute_current()` / `compute_history()` / `set_depth_frame()` / `depth_input()` names match across T3 (builder), T5 (controller), T6 (robot_controller → `ctrl->set_depth_frame`), and the `BasicUserController::set_depth_frame` default in T6 Step 7. `has_perception()` / `obs_group()` / `input_shape()` / depth getters match across T2 test and T5 usage. `DepthImageSubscriber` is constructed in T6 with config domain/interface and used via `latest_frame()`. Consistent.
- **Test/obs_builder.cpp note:** The fake-robot approach uses the real `Go2RobotInterface` with `go2_joint_names()` — verified those names exist in `robot_interface.hpp`. The test links onnxruntime to load real metadata; the model path is passed as argv[1].
