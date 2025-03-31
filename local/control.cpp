#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <fstream>
#include "mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h"

mavsdk::Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s
    )
{
    mavsdk::Mission::MissionItem new_item{};
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    return new_item;
}

std::vector<mavsdk::Mission::MissionItem> mission_from_file(const std::string& mission_csv) {
    std::ifstream file(mission_csv);
    std::vector<mavsdk::Mission::MissionItem> mission_items;
    std::string line;
    while(std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        std::vector<std::string> tokens;
        while(std::getline(iss, token, ',')) {
            tokens.push_back(token);
        }
        if(tokens.size() != 4) {
            std::cerr << "Invalid mission item: " << line << std::endl;
            continue;
        }
        mission_items.push_back(make_mission_item(
            std::stod(tokens[0]),
            std::stod(tokens[1]),
            std::stof(tokens[2]),
            std::stof(tokens[3])
        ));
    }
    file.close();
    return mission_items;
}


using namespace std::chrono_literals;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url> <mission_path>\n"
              << "Connection URL format should be :\n"
              << " For TCP server: tcpin://<our_ip>:<port>\n"
              << " For TCP client: tcpout://<remote_ip>:<port>\n"
              << " For UDP server: udp://<our_ip>:<port>\n"
              << " For UDP client: udp://<remote_ip>:<port>\n"
              << " For Serial : serial://</path/to/serial/dev>:<baudrate>]\n"
              << "For example, to connect to the simulator use URL: udpin://0.0.0.0:14540\n";
}

int main(int argc, char** argv)
{
    if (argc != 3) {
        usage(argv[0]);
        return 1;
    }

    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}};
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(-1);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Instantiate plugins.
    auto telemetry = mavsdk::Telemetry{system.value()};
    auto action = mavsdk::Action{system.value()};
    auto mission = mavsdk::Mission{system.value()};


    // We want to listen to the altitude of the drone at 1 Hz.
    const auto set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != mavsdk::Telemetry::Result::Success) {
        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
        return 1;
    }

    // Set up callback to monitor altitude while the vehicle is in flight
    telemetry.subscribe_position([](mavsdk::Telemetry::Position position) {
        std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
    });

    // Check until vehicle is ready to arm

    // auto mission_items = mission_from_file(argv[2]);
    // mavsdk::Mission::MissionPlan mission_plan{};
    // mission_plan.mission_items = mission_items;
    // auto upload_result = mission.upload_mission(mission_plan);
    //
    // if(upload_result != mavsdk::Mission::Result::Success) {
    //     std::cerr << "Mission upload failed: " << upload_result << '\n';
    //     return 1;
    // }

    // Arm vehicle
    std::cout << "Arming...\n";

    mavsdk::MavlinkPassthrough mavlink_passthrough(system.value());
    auto send_result = MAV_RESULT_ACCEPTED;
    mavlink_passthrough.queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
        mavlink_message_t message;
        mavlink_msg_command_ack_encode(
            mavlink_address.system_id,
            mavlink_address.component_id,
            channel,
            &message,
            MAV_CMD_DO_SET_MODE,


            );
        return message;
    });



    // auto mode_guided = action.
    auto arm_result = action.arm();
    while ((arm_result = action.arm()) != mavsdk::Action::Result::Success) {
        std::this_thread::sleep_for(5s);
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }
    //
    // std::atomic<bool> want_to_pause{false};
    // mission.subscribe_mission_progress([&want_to_pause](mavsdk::Mission::MissionProgress mission_progress) {
    // std::cout << "Mission status update: " << mission_progress.current << " / "
    //           << mission_progress.total << '\n';
    //
    // if (mission_progress.current >= 2) {
    //     want_to_pause = true;
    // }
    // });

    auto start_mission_result = mission.start_mission();
    if (start_mission_result != mavsdk::Mission::Result::Success) {
        std::cerr << "Starting mission failed: " << start_mission_result << '\n';
        return 1;
    }

    // while (!want_to_pause) {
    //     std::this_thread::sleep_for(1s);
    // }

    std::cout << "Pausing mission...\n";
    auto pause_mission_result = mission.pause_mission();

    if (pause_mission_result != mavsdk::Mission::Result::Success) {
        std::cerr << "Failed to pause mission:" << pause_mission_result << '\n';
    }
    std::cout << "Mission paused.\n";

    // Pause for 5 seconds.
    std::this_thread::sleep_for(5s);

    // Then continue.
    auto start_mission_again_result = mission.start_mission();
    if (start_mission_again_result != mavsdk::Mission::Result::Success) {
        std::cerr << "Starting mission again failed: " << start_mission_again_result << '\n';
        return 1;
    }

    while (!mission.is_mission_finished().second) {
        std::this_thread::sleep_for(1s);
    }

    std::cout << "Commanding RTL...\n";
    auto rtl_result = action.return_to_launch();
    if (rtl_result != mavsdk::Action::Result::Success) {
        std::cout << "Failed to command RTL: " << rtl_result << '\n';
        return 1;
    }
    std::cout << "Commanded RTL.\n";

    // We need to wait a bit, otherwise the armed state might not be correct yet.
    std::this_thread::sleep_for(2s);

    while (telemetry.armed()) {
        // Wait until we're done.
        std::this_thread::sleep_for(1s);
    }

    return 0;
}