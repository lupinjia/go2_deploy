#include <filesystem>
#include <fstream>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace fs = std::filesystem;

class CSVWriter 
{
public:
    CSVWriter() {}
    CSVWriter(const std::string& folder_prefix)
    {
        /***** Create log folder and log file *****/
        // get the current time
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << folder_prefix << "_";
        ss << std::put_time(std::localtime(&time), "%Y%m%d-%H%M%S");
        // create log folder named by current time
        fs::path log_folder = fs::current_path() / "logs" / ss.str();
        fs::create_directories(log_folder);
        // using csv format to save log data
        fs::path log_file_name = log_folder / "log.csv";
        log_file = std::ofstream(log_file_name);

        std::cout << "Log file created: " << log_file_name.string() << std::endl;
        
    }

    ~CSVWriter() {}

    void setHeader(const std::stringstream& header)
    {
        log_file << header.str();
    }

    void write(const auto& data)
    {
        if (log_file.is_open())
        {
            for(auto& d : data)
            {
                log_file << d << ",";
            }
            log_file << std::endl;
        }
    }

private:
    std::ofstream log_file;

};
    