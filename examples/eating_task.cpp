/* 
 *  This is the default license template.
 *  
 *  File: eating_task.cpp
 *  Author: Nathan Dunkelberger
 *  Copyright (c) 2020 Nathan Dunkelberger
 *  
 *  To edit this license information: Press Ctrl+Shift+P and press 'Create new License Template...'.
 */

#include <Mahi/Daq.hpp>
#include <Mahi/Com.hpp>
#include <Mahi/Util.hpp>
#include <MEII/MEII.hpp>
#include <vector>

using namespace mahi::com;
using namespace mahi::daq;
using namespace mahi::util;
using namespace meii;

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char *argv[]) {
    register_ctrl_handler(handler);

    // make options
    Options options("eating_task.exe", "Nathan's Position Control Demo");
    options.add_options()
        ("c,calibrate", "Calibrates the MAHI Exo-II")
        ("r,record", "MAHI Exo-II records position and velocity until enter is pressed")
        ("h,help", "Prints this help message");

    auto result = options.parse(argc, argv);

    // if -h, print the help option
    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }

    // enable Windows realtime
    enable_realtime();

    /////////////////////////////////
    // construct and config MEII   //
    /////////////////////////////////

    Q8Usb q8;
    q8.open();
    std::vector<TTL> idle_values(8,TTL_HIGH);
    q8.DO.enable_values.set({0,1,2,3,4,5,6,7},idle_values);
    q8.DO.disable_values.set({0,1,2,3,4,5,6,7},idle_values);
    q8.DO.expire_values.write({0,1,2,3,4,5,6,7},idle_values);    
    MeiiConfigurationHardware config_hw(q8); 
    MahiExoIIHardware meii(config_hw);

    // calibrate - robot automatically calibrates encoders
    if (result.count("calibrate") > 0) {
        meii.calibrate_auto(stop);
        LOG(Info) << "MAHI Exo-II encoders calibrated.";
        return 0;
    }

    // make MelShares
    MelShare ms_anat_pos("ms_anat_pos");
    MelShare ms_robo_pos("ms_robo_pos");
    MelShare ms_anat_vel("ms_anat_vel");
    MelShare ms_robo_vel("ms_robo_vel");

    Time Ts = milliseconds(1); // sample period for DAQ

    // construct timer in hybrid mode to avoid using 100% CPU
    Timer timer(Ts, Timer::Hybrid);
    timer.set_acceptable_miss_rate(0.05);

    // construct clock for regulating keypress
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds(0.5);

    // preparing to save data
    bool save_data = false;
    std::string filepath = "../../data/Eating_task_example_.csv";

    // initialize double vectors for collecting data
    std::vector<std::string> log_header = {"Time (s)", 
                                            "anat_pos_1", "anat_pos_2", "anat_pos_3", "anat_pos_4", "anat_pos_5",
                                            "robo_pos_1", "robo_pos_2", "robo_pos_3", "robo_pos_4", "robo_pos_5",
                                            "anat_vel_1", "anat_vel_2", "anat_vel_3", "anat_vel_4", "anat_vel_5",
                                            "robo_vel_1", "robo_vel_2", "robo_vel_3", "robo_vel_4", "robo_vel_5"};
    std::vector<double> anat_joint_positions(0,meii.N_aj_);
    std::vector<double> robo_joint_positions(0,meii.N_aj_);
    std::vector<double> anat_joint_velocities(0,meii.N_aj_);
    std::vector<double> robo_joint_velocities(0,meii.N_aj_);
    std::vector<std::vector<double>> robot_log;

    // enable DAQ and exo
    q8.enable();
    meii.enable();

    q8.watchdog.start();
    // if the user used flag -r, record data
    if (result.count("record") > 0) {
        LOG(Info) << "MAHI Exo-II Eating Task Recording has started.";
        while (!stop){

            // update all DAQ input channels
            q8.update_input();

            // update MahiExoII kinematics
            meii.update_kinematics();

            // Gather the data from the DAQ
            anat_joint_positions = meii.get_anatomical_joint_positions();
            robo_joint_positions = meii.get_joint_positions();
            anat_joint_velocities = meii.get_anatomical_joint_velocities();
            robo_joint_velocities = meii.get_joint_velocities();

            // write to robot data log
            std::vector<double> robot_log_row;
            robot_log_row.push_back(timer.get_elapsed_time().as_seconds());
            robot_log_row.insert(robot_log_row.end(),anat_joint_positions.begin(),anat_joint_positions.end());
            robot_log_row.insert(robot_log_row.end(),robo_joint_positions.begin(),robo_joint_positions.end());
            robot_log_row.insert(robot_log_row.end(),anat_joint_velocities.begin(),anat_joint_velocities.end());
            robot_log_row.insert(robot_log_row.end(),robo_joint_velocities.begin(),robo_joint_velocities.end());

            robot_log.push_back(robot_log_row);

            ms_anat_pos.write_data(anat_joint_positions);
            ms_robo_pos.write_data(robo_joint_positions);
            ms_anat_vel.write_data(anat_joint_velocities);
            ms_robo_vel.write_data(robo_joint_velocities);

            // check for save key
            if (Keyboard::is_key_pressed(Key::Enter)) {
                stop = true;
                save_data = true;
            }

            // check for exit key
            if (Keyboard::is_key_pressed(Key::Escape)) {
                stop = true;
            }

            // kick watchdog
            if (!q8.watchdog.kick() || meii.any_limit_exceeded()) {
                stop = true;
            }

            timer.wait();
        }
    }

    // save the data if the user wants
    if (save_data) {
        print("Do you want to save the robot data log? (Y/N)");
        Key key = Keyboard::wait_for_any_keys({ Key::Y, Key::N });
        if (key == Key::Y) {
            csv_write_row(filepath, log_header);
            csv_append_rows(filepath, robot_log);
        }
    }
    
    disable_realtime();
    Keyboard::clear_console_input_buffer();
    return 0;
}

