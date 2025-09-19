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

using namespace unitree::common;
using namespace unitree::robot;

namespace fs = std::filesystem;

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
    if (argc < 3) // local loopback
    {
        ChannelFactory::Instance()->Init(1, "lo"); // "lo" for local loopback
    }
    else // real robot
    {
        ChannelFactory::Instance()->Init(0, argv[2]);
    }

    BasicUserController* user_ctrl;
    // load config and user_controller according to the argument
    if(argv[1] == std::string("wtw"))
    {
        std::string config_file_dir = param / "wtw_config.yaml";
        user_ctrl = new WTWController(config_file_dir);
    }
    else if(argv[1] == std::string("simple_rl"))
    {
        std::string config_file_dir = param / "simple_rl_config.yaml";
        user_ctrl = new SimpleRLController(config_file_dir);
    }
    else if(argv[1] == std::string("ts"))
    {
        std::string config_file_dir = param / "ts_config.yaml";
        user_ctrl = new TSController(config_file_dir);
    }
    else
    {
        std::cerr << "Unknown controller type: " << argv[1] << std::endl;
        return -1;
    }

    RobotController* robot_controller = new RobotController(log_file_name, user_ctrl);
    // load neural network model
    robot_controller->loadParam();
    robot_controller->loadPolicy();

    if (argc >= 3)
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