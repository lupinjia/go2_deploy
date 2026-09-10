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
        std::cerr << "usage: test_metadata <model.onnx> [top bottom left right]\n";
        return 2;
    }
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
    Ort::SessionOptions opts;
    Ort::Session session(env, argv[1], opts);
    auto md = booster_deploy::load_onnx_metadata(session);

    if (argc == 6)
    {
        for (int i = 0; i < 4; ++i)
        {
            CHECK(md.depth_crop_region[i] == std::atoi(argv[i + 2]),
                  "depth_crop_region value");
        }
    }

    if (md.has_perception())
    {
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
    CHECK(dshape[1] == 2 && dshape[2] == md.depth_height() &&
              dshape[3] == md.depth_width(),
          "depth graph shape matches metadata");
    CHECK(md.depth_width() > 0, "depth width is positive");
    CHECK(md.depth_height() > 0, "depth height is positive");
    CHECK(md.depth_cutoff_distance() == 2.5f, "cutoff 2.5");
    CHECK(md.depth_min_depth() == 0.105f, "min depth 0.105");
    CHECK(md.rnn_type == "gru", "rnn type gru");
    CHECK(md.rnn_hidden_dim == 512, "rnn hidden 512");
    const auto& actor = md.obs_group("actor");
    CHECK(actor.history_length == 0, "actor history 0");
    CHECK(actor.term_names.size() == 6, "actor 6 terms");
    CHECK(actor.term_names[2] == "vel_command", "actor term[2] vel_command");
    CHECK(actor.term("joint_pos")->scale.size() == 1, "joint_pos scale len 1");
    CHECK(actor.term("vel_command")->scale.size() == 3, "vel_command per-dim scale 3");
    const auto& hist = md.obs_group("history");
    CHECK(hist.history_length == 20, "history group length 20");
    CHECK(hist.term("joint_pos")->history_length == 20, "history term hist 20");
    const auto& dg = md.obs_group("depth_image");
    CHECK(dg.history_length == 2, "depth group history 2");
    CHECK(!dg.flatten_history_dim, "depth group not flattened");
    }

    if (failures == 0)
        std::cout << "ALL PASS\n";
    return failures == 0 ? 0 : 1;
}
