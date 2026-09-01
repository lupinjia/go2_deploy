#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <optional>
#include <chrono>
#include <iomanip>

#include "robot_controller.hpp"
#include "user_controller.hpp"
#include "robot_interface.hpp"
#include "parkour_controller.hpp"

using namespace unitree::common;
using namespace unitree::robot;

namespace fs = std::filesystem;

// Decide controller type from the ONNX metadata: presence of onnx_input_groups
// (perception/depth/RNN model) selects Go2ParkourController, else the blind
// Go2OnnxController.
static BasicUserController *create_controller(const std::string &cfg_file,
                                              const std::vector<std::string> &joint_names)
{
    auto cfg = YAML::LoadFile(cfg_file);
    std::string controller = cfg["controller"].as<std::string>("blind");

    if (controller == "parkour")
    {
        std::string parkour_policy = cfg["parkour_policy_name"].as<std::string>();
        fs::path policy_file = parkour_policy;
        if (policy_file.is_relative())
            policy_file = fs::current_path() / "../models" / parkour_policy;
        if (!fs::exists(policy_file))
        {
            std::cerr << "parkour policy not found: " << policy_file << std::endl;
            std::exit(EXIT_FAILURE);
        }
        std::cout << "[main] controller: parkour (" << parkour_policy << ")" << std::endl;
        return new Go2ParkourController(cfg_file, joint_names);
    }

    std::cout << "[main] controller: blind" << std::endl;
    return new Go2OnnxController(cfg_file, joint_names);
}

int main(int argc, char const *argv[])
{
    // constant param_folder
    std::string param_folder = "../params";
    fs::path param = fs::current_path() / param_folder;

    /***** log *****/
    // get the current time
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y%m%d-%H%M%S");
    // create log folder named by current time
    fs::path log_folder = fs::current_path() / "logs" / ss.str();
    fs::create_directories(log_folder);
    // using csv format to save log data
    fs::path log_file_name = log_folder / "log.csv";

    /***** Controller *****/
    if (argc < 2) // local loopback
    {
        ChannelFactory::Instance()->Init(1, "lo"); // "lo" for local loopback
    }
    else // real robot
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }

    // load config and user_controller
    std::string config_file_dir = (param / "go2_config.yaml").string();
    BasicUserController* user_ctrl = create_controller(config_file_dir, go2_joint_names());

    RobotController* robot_controller = new RobotController(log_file_name, user_ctrl);
    // load neural network model
    robot_controller->loadParam();
    robot_controller->loadPolicy();

    if (argc >= 2)
    {
        // deactivate the mcf service
        robot_controller->initRobotStateClient();
        std::cout<<"Try to deactivate the service: "<<"mcf"<<std::endl;
        int serviceStatus = 0;
        while (!serviceStatus)
        {
            robot_controller->activateService("mcf", 0, serviceStatus);
            sleep(1);
        }
        std::cout<<"Deactivate the service: "<<"mcf"<<std::endl;
    }

    // initialize dds model
    robot_controller->InitDdsModel();

    // start control loop
    robot_controller->StartControl();

    return 0;
}
