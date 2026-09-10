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
    // term-major, per-term history flattened: first term (base_ang_vel, 3 dims)
    // spans [0,60); its last frame starts at 3*19.
    CHECK(std::abs(hist[3*19] - cur[0]) < 1e-5f, "last frame base_ang_vel[0] backfilled");

    constexpr int source_width = 53;
    constexpr int source_height = 30;
    std::vector<float> frame(source_height * source_width);
    for (int r = 0; r < source_height; ++r)
    {
        for (int c = 0; c < source_width; ++c)
            frame[r * source_width + c] = static_cast<float>(r * 1000 + c);
    }
    auto cropped = builder.crop_depth_frame(
        frame.data(), frame.size(), source_width, source_height);
    CHECK(static_cast<int>(cropped.size()) == md.depth_height() * md.depth_width(),
          "visualization crop size matches model shape");
    builder.set_depth_frame(frame.data(), frame.size(), source_width, source_height);
    builder.set_depth_frame(frame.data(), frame.size(), source_width, source_height);
    auto depth = builder.depth_input();
    const auto crop = md.depth_crop_region;
    const int expected_width = md.depth_width();
    const int expected_height = md.depth_height();
    CHECK(static_cast<int>(depth.size()) == 2 * expected_height * expected_width,
          "depth_input size matches model shape");
    const int first_source_index = crop[0] * source_width + crop[2];
    const int last_source_index =
        (source_height - crop[1] - 1) * source_width +
        (source_width - crop[3] - 1);
    if (depth.size() == static_cast<size_t>(2 * expected_height * expected_width))
    {
        CHECK(std::abs(depth[0] - frame[first_source_index]) < 1e-5f,
              "depth crop top-left pixel");
        CHECK(std::abs(depth[expected_height * expected_width - 1] -
                       frame[last_source_index]) < 1e-5f,
              "depth crop bottom-right pixel");
        CHECK(std::abs(depth[expected_height * expected_width] -
                       frame[first_source_index]) < 1e-5f,
              "depth second history frame");
    }

    if (failures == 0)
        std::cout << "ALL PASS\n";
    return failures == 0 ? 0 : 1;
}
