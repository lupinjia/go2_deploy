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
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

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
    RobotController<RLController> *robot_controller;
    robot_controller = new RobotController<RLController>(log_file_name);
    // load parameter from .json file
    robot_controller->LoadParam(param);
    // load neural network model
    robot_controller->loadPolicy();

    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo"); // "lo" for local loopback
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
        // Init robot state client
        RobotStateClient rsc;
        rsc.SetTimeout(10.0f); 
        rsc.Init();
        // deactivate the mcf service
        int serviceStatus = 0;
        while(serviceStatus == 0)
        {
            std::cout<<"Try to deactivate the service: mcf" <<std::endl;
            rsc.ServiceSwitch("mcf", 0, serviceStatus);
            sleep(1);
        }
        std::cout << "Service mcf deactivated" << std::endl;
    }

    // initialize dds model
    robot_controller->InitDdsModel();

    // start control loop
    robot_controller->StartControl();

    return 0;
}