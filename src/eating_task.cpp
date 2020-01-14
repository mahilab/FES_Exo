#include <MEL/Daq/Quanser/Q8Usb.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <MEL/Utility/System.hpp>
#include <MEL/Communications/MelShare.hpp>
#include <MEL/Utility/Options.hpp>
#include <MEL/Core/Timer.hpp>
#include <MEL/Core/Console.hpp>
#include <MEL/Logging/Log.hpp>
#include <MEL/Logging/Csv.hpp>
#include <MEL/Devices/Windows/Keyboard.hpp>
#include <MEL/Math/Integrator.hpp>
#include <vector>

using namespace mel;
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
    Options options("ex_pos_control_nathan.exe", "Nathan's Position Control Demo");
    options.add_options()
        ("c,calibrate", "Calibrates the MAHI Exo-II")
        ("r,record", "MAHI Exo-II records position and velocity until enter is pressed")
        // ("t,test", "Runs the code without the exoskeleton portions for testing")
        ("h,help", "Prints this help message");

    auto result = options.parse(argc, argv);

    // if -h, print the help option
    if (result.count("help") > 0) {
        print(options.help());
        return 0;
    }

    // enable Windows realtime
    enable_realtime();

    // construct Q8 USB and configure
    Q8Usb q8;
    q8.open();
    q8.DO.set_enable_values(std::vector<Logic>(8, High));
    q8.DO.set_disable_values(std::vector<Logic>(8, High));
    q8.DO.set_expire_values(std::vector<Logic>(8, High));

    Time Ts = milliseconds(1); // sample period for DAQ

    // create MahiExoII and bind Q8 channels to it
    std::vector<Amplifier> amplifiers;
    for (uint32 i = 0; i < 2; ++i) {
        amplifiers.push_back(
            Amplifier("meii_amp_" + std::to_string(i),
                Low,
                q8.DO[i + 1],
                1.8,
                q8.AO[i + 1])
        );
    }
    for (uint32 i = 2; i < 5; ++i) {
        amplifiers.push_back(
            Amplifier("meii_amp_" + std::to_string(i),
                Low,
                q8.DO[i + 1],
                0.184,
                q8.AO[i + 1])
        );
    }
    MeiiConfiguration config(q8, q8.watchdog, q8.encoder[{1, 2, 3, 4, 5}], amplifiers);
    MahiExoII meii(config);

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
        LOG(Info) << "MAHI Exo-II Trajectory Recording has started.";
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

