#include "unitree/common/json/jsonize.hpp"
#include <vector>
#include <iostream>

namespace unitree::common
{
    class ExampleCfg : public Jsonize
    {
    public:
        ExampleCfg() : kp(0), kd(0), dt(0)
        {
        }

        void fromJson(JsonMap &json)
        {
            FromJson(json["kp"], kp);
            FromJson(json["kd"], kd);
            FromJson(json["dt"], dt);
            FromAny<float>(json["stand_pos"], stand_pos);
        }

        void toJson(JsonMap &json) const
        {
            ToJson(kp, json["kp"]);
            ToJson(kd, json["kd"]);
            ToJson(dt, json["dt"]);
            ToAny<float>(stand_pos, json["stand_pos"]);
        }

        float kp;
        float kd;
        float dt;

        std::vector<float> stand_pos;
    };

    class RLCfg : public Jsonize
    {
    public:
        RLCfg() : stand_kp(0), stand_kd(0), ctrl_kp(0), ctrl_kd(0), dt(0), 
                  action_scale(0.1), lin_vel_scale(2.0), ang_vel_scale(0.25), dof_vel_scale(0.05)
        {
        }

        void fromJson(JsonMap &json)
        {
            FromJson(json["stand_kp"], stand_kp);
            FromJson(json["stand_kd"], stand_kd);
            FromJson(json["dt"], dt);
            FromJson(json["action_scale"], action_scale);
            FromJson(json["lin_vel_scale"], lin_vel_scale);
            FromJson(json["ang_vel_scale"], ang_vel_scale);
            FromJson(json["dof_vel_scale"], dof_vel_scale);
            FromJson(json["ctrl_kp"], ctrl_kp);
            FromJson(json["ctrl_kd"], ctrl_kd);
            FromJson(json["num_gaits"], num_gaits);
            FromJson(json["policy_name"], policy_name);
            FromJson(json["frame_stack"], frame_stack);
            FromJson(json["num_single_obs"], num_single_obs);
            FromAny<float>(json["gait_periods"], gait_periods);
            FromAny<float>(json["swing_phase_ratios"], swing_phase_ratios);
            FromAny<float>(json["stand_pos"], stand_pos);
            FromAny<float>(json["sit_pos"], sit_pos);
            FromAny<float>(json["theta_fl"], theta_fl);
            FromAny<float>(json["theta_fr"], theta_fr);
            FromAny<float>(json["theta_rl"], theta_rl);
            FromAny<float>(json["theta_rr"], theta_rr);
        }

        void toJson(JsonMap &json) const
        {
            ToJson(stand_kp, json["stand_kp"]);
            ToJson(stand_kd, json["stand_kd"]);
            ToJson(dt, json["dt"]);
            ToJson(action_scale, json["action_scale"]);
            ToJson(lin_vel_scale, json["lin_vel_scale"]);
            ToJson(ang_vel_scale, json["ang_vel_scale"]);
            ToJson(dof_vel_scale, json["dof_vel_scale"]);
            ToJson(ctrl_kp, json["ctrl_kp"]);
            ToJson(ctrl_kd, json["ctrl_kd"]);
            ToJson(num_gaits, json["num_gaits"]);
            ToJson(policy_name, json["policy_name"]);
            ToJson(frame_stack, json["frame_stack"]);
            ToJson(num_single_obs, json["num_single_obs"]);
            ToAny<float>(swing_phase_ratios, json["swing_phase_ratios"]);
            ToAny<float>(gait_periods, json["gait_periods"]);
            ToAny<float>(stand_pos, json["stand_pos"]);
            ToAny<float>(sit_pos, json["sit_pos"]);
            ToAny<float>(theta_fl, json["theta_fl"]);
            ToAny<float>(theta_fr, json["theta_fr"]);
            ToAny<float>(theta_rl, json["theta_rl"]);
            ToAny<float>(theta_rr, json["theta_rr"]);
        }

        float stand_kp;
        float stand_kd;
        float dt;
        float action_scale;
        float lin_vel_scale;
        float ang_vel_scale;
        float dof_vel_scale;
        float ctrl_kp;
        float ctrl_kd;
        int num_actions;
        int frame_stack;
        int num_single_obs;
        
        std::string policy_name;
        
        // gait parameters
        int num_gaits;
        std::vector<float> gait_periods;
        std::vector<float> swing_phase_ratios;
        std::vector<float> theta_fl;
        std::vector<float> theta_fr;
        std::vector<float> theta_rl;
        std::vector<float> theta_rr;

        std::vector<float> stand_pos;
        std::vector<float> sit_pos;
        
    };
}