// MIT License
// 
// Copyright (c) 2020 MAHI Lab
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// Author: Nathan Dunkelberger (nbd2@rice.edu)

#include <MEII/MEII.hpp>
#include <Mahi/Fes.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Daq.hpp>
#include <Mahi/Robo.hpp>
#include <FESExo/MuscleData.hpp>
#include <FESExo/Utility.hpp>
#include <algorithm>
#include <random>

using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::fes;
using namespace mahi::robo;
using namespace meii;

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y_%m_%d_%H_%M_%S", &tstruct);

    return buf;
}

std::vector<double> generate_new_goal_pos(std::vector<std::vector<double>> record_positions, std::vector<uint8> num_test_points, std::size_t &iteration_num, const std::vector<std::vector<int>> &pos_order, std::vector<int> &current_it, double slop_pos){

    current_it = pos_order[iteration_num];

    for (size_t i = 0; i < num_test_points.size(); i++){
        if (current_it[i] >= num_test_points[i]){
            LOG(Error) << "Invalid test point for joint " << i << ". Returning empty vector.";
            return std::vector<double>();
        }
    }

    std::vector<double> new_pos(record_positions.size(),0);
    for (size_t i = 0; i < record_positions.size(); i++){
        new_pos[i] = record_positions[i][current_it[i]];
    }
    new_pos.push_back(slop_pos);
    
    iteration_num++;

    return new_pos;
}

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

std::mutex mtx;
std::vector<unsigned int> write_vals;
void update_stim(std::vector<Channel> channels, Stimulator* stim){
    std::vector<unsigned int> local_write_vals(channels.size(),0);
    Timer stim_timer(50_ms);

    while(!stop){
        {
            std::lock_guard<std::mutex> guard(mtx);
            copy(write_vals.begin(), write_vals.end(), local_write_vals.begin());            
        }
        stim->write_pws(channels,local_write_vals);
        stim->update();
        stim_timer.wait();
    }
    stim->write_pws(channels,std::vector<unsigned int>(channels.size(),0)); 
    stim->update();
}

void write_to_file(std::string filepath, std::vector<std::string> header, std::vector<std::vector<double>> data){
    csv_write_row(filepath, header);
	csv_append_rows(filepath, data);
}

int main(int argc, char* argv[]) {
    // register ctrl-c handler
    register_ctrl_handler(handler);

    // make options
    Options options("ex_pos_control_nathan.exe", "Nathan's Position Control Demo");
    options.add_options()
		("c,calibrate", "Calibrates the MAHI Exo-II")
        ("m,virtual_meii", "meii is virtual and will communicate with the unity sim")
        ("f,virtual_fes", "fes is virtual and will not wait for responses from the board")
        ("v,visualize", "fes visualizer will be used to show the current status")
        ("s,subject", "subject number, ie. -s 1001",cxxopts::value<int>(), "N")
        ("i,iteration", "iteration number to start at, ie. -i 18", cxxopts::value<int>(), "N")
		("h,help", "Proper usage: ./fes_calibration.exe (-m) (-f) (-v) (-i 5) -s 1001");

    auto result = options.parse(argc, argv);

    // if -h, print the help option
	if (result.count("help") > 0) {
		print_var(options.help());
		return 0;
	}

    if (!result.count("subject")) {
		LOG(Error) << "You must input a subject number to run this program. Exiting Program.";
		return 0;
	}

    int subject_num = result["subject"].as<int>();
    
    // INITIALIZE MEII

    std::shared_ptr<MahiExoII> meii = nullptr;
    std::shared_ptr<Q8Usb> q8 = nullptr;
    
    if(result.count("virtual_meii") > 0){
        MeiiConfigurationVirtual config_vr;
        meii = std::make_shared<MahiExoIIVirtual>(config_vr);
    }
    else{
        q8 = std::make_shared<Q8Usb>();
        q8->open();
        std::vector<TTL> idle_values(8,TTL_HIGH);
        q8->DO.enable_values.set({0,1,2,3,4,5,6,7},idle_values);
        q8->DO.disable_values.set({0,1,2,3,4,5,6,7},idle_values);
        q8->DO.expire_values.write({0,1,2,3,4,5,6,7},idle_values);
        MeiiConfigurationHardware config_hw(*q8);
        meii = std::make_shared<MahiExoIIHardware>(config_hw);
    }

	// calibrate - zero the encoders if the -c argument was give
	if (result.count("calibrate") > 0) {
		meii->calibrate_auto(stop);
		LOG(Info) << "MAHI Exo-II encoders calibrated.";
		return 0;
	}

    // END MEII INITIALIZATION

    // INITIALIZE STIMULATOR

    // create channels of interest
    std::vector<Channel> channels;
    // Channel 1 - Bicep
    Channel bicep("Bicep", CH_1, AN_CA_1, 100, 250); 
    channels.push_back(bicep);
    // Channel 2 - Tricep
    Channel tricep("Tricep", CH_2, AN_CA_2, 100, 250);
    channels.push_back(tricep);
    // Channel 3 - Pronator Teres
    Channel pronator_teres("Pronator Teres", CH_3, AN_CA_3, 100, 250);
    channels.push_back(pronator_teres);
    // Channel 4 - Brachioradialis
    Channel brachioradialis("Brachioradialis", CH_4, AN_CA_4, 100, 250);
    channels.push_back(brachioradialis);
    // Channel 5 - Flexor Carpi Radialis
    Channel flexor_carpi_radialis("Flexor Carpi Radialis", CH_5, AN_CA_1, 100, 250);
    channels.push_back(flexor_carpi_radialis);
    // Channel 6 - Palmaris Longus
    Channel palmaris_longus("Palmaris Longus", CH_6, AN_CA_2, 100, 250);
    channels.push_back(palmaris_longus);
    // Channel 7 - Flexor Carpi Ulnaris
    Channel flexor_carpi_ulnaris("Flexor Carpi Ulnaris", CH_7, AN_CA_3, 100, 250);
    channels.push_back(flexor_carpi_ulnaris);
    // Channel 8 - Extensor Carpi Radialis Longus
    Channel extensor_carpi_radialis_longus("Extensor Carpi Radialis Longus", CH_8, AN_CA_4, 100, 250);
    channels.push_back(extensor_carpi_radialis_longus);

    bool virt_stim = (result.count("virtual_fes") > 0);
    bool visualizer_on = (result.count("virtual_fes") > 0);
    
    Stimulator stim("UECU Board", channels, "COM4", "COM5");
    stim.create_scheduler(0xAA, 40); // 40 hz frequency 
    stim.add_events(channels);       // add all channels as events

    uint8 current_stim_channel = 0;
    uint8 total_channels_stimmed = 0;
    std::random_device rd;
    std::mt19937 g(rd());
    std::vector<uint8> randomized_channels = {1, 2, 3, 4, 5, 6, 7, 8};
    std::shuffle(randomized_channels.begin(),randomized_channels.end(),g);
    uint8 num_channels = static_cast<uint8>(channels.size());
   
    MuscleData muscle_data(get_muscle_info(subject_num));

    // done importing subject-specific parameters

    std::vector<unsigned int> max_stim_vals_int = muscle_data.get_max_pulsewidths();
    std::vector<unsigned int> stim_amplitudes = muscle_data.get_amplitudes();

    std::vector<double> max_stim_vals(max_stim_vals_int.begin(),max_stim_vals_int.end());

    std::vector<bool> stim_actives = muscle_data.get_actives();
    
    std::thread viz_thread([&stim]() {
        Visualizer visualizer(&stim);
        visualizer.run();
    });

    // END INITIALIZE STIMULATOR

    const size_t total_iterations = 72;
    std::vector<std::vector<int>> pos_order(total_iterations , std::vector<int> (4, 0));
    csv_read_rows("C:\\Git\\FES_Exo\\data\\CollectionPointsRandomized.csv",pos_order);

    // generate data for goal waypoints
    std::vector<uint8> num_test_points = {3, 2, 2, 2};

    std::vector<int> current_iteration(4,0);
    std::vector<int> last_iteration(4,0);

    size_t iteration_num = (result.count("iteration")) ? static_cast<size_t>(result["iteration"].as<int>()) : 0;

    if(current_iteration.size() != 4){
        stop = true;
        LOG(Error) << "Wrong size iteration input vector. Stopping program.";
    }

    double slop_pos = 0.105; // position for the forearm slop position

    std::vector<std::vector<double>> record_positions = { {-60.0*DEG2RAD, -32.5*DEG2RAD, -5.0*DEG2RAD}, // Elbow FE
                                                          {-40.0*DEG2RAD,  40.0*DEG2RAD},               // Forearm PS
                                                          {-10.0*DEG2RAD,  10.0*DEG2RAD},               // Wrist FE
                                                          {-10.0*DEG2RAD,  10.0*DEG2RAD} };             // Wrist RU

    std::vector<double> max_diff = { 40 * DEG2RAD, 40 * DEG2RAD, 20 * DEG2RAD, 20 * DEG2RAD, 0.01 };

    Time to_pos_time  = 4_s;
    Time ramp_time    = 0.5_s;
    Time hold_time    = 1_s;
    Time collect_time = 1_s;

    std::vector<WayPoint> traj_waypoints;
    
    traj_waypoints.push_back(WayPoint(Time::Zero, {-PI/4, 0, 0, 0, slop_pos}));
    traj_waypoints.push_back(WayPoint(to_pos_time, {-PI/4, 0, 0, 0, slop_pos}));   

    Trajectory next_point_traj;

    std::vector<double> ref(5,0);

    enum state {
        setup,        // 0
        to_next_pos,  // 1
        command_stim, // 2
        finish        // 3
    };

    state current_state = setup;

    std::vector<double> aj_positions(5,0);

    // data collection setup
    std::vector<std::vector<double>> data;
    std::vector<std::string> header = {"stim_pw [us]", "stim_amp [mA]", "com_elbow_fe [rad]", "com_forearm_ps [rad]", "com_wrist_fe [rad]", "com_wrist_ru [rad]", "act_elbow_fe [rad]", "act_forearm_ps [rad]", "act_wrist_fe [rad]", "act_wrist_ru [rad]", "elbow_fe_trq [Nm]", "forearm_ps_trq [Nm]", "wrist_fe_trq [Nm]", "wrist_ru_trq [Nm]"};

    Timer timer(1_ms);
    Clock state_clock;

    stim.begin();
    stim.set_amps(channels, stim_amplitudes);

    // enable DAQ and exo
	meii->daq_enable();
	meii->enable();
	// meii->daq_watchdog_start();  

    meii->daq_read_all();
    meii->update_kinematics();

    meii->rps_init_par_ref_.start(meii->get_wrist_parallel_positions(), state_clock.get_elapsed_time());

    enable_realtime();

    std::thread stim_thread(update_stim, channels, &stim);

    // while it has not been stopped, 
    while(!stop){
        meii->daq_read_all();
        meii->update_kinematics();

        aj_positions = meii->get_anatomical_joint_positions();

        Time current_time = state_clock.get_elapsed_time();

        std::vector<unsigned int> stim_vals(num_channels,0);

        switch (current_state){
            // INITIALIZING RPS  MECHANISM
            case setup:
            {
                std::vector<double> trqs = meii->set_robot_smooth_pos_ctrl_torques(meii->rps_init_par_ref_, current_time);

                if (meii->check_rps_init()){
                    print("RPS initialized");
                    current_state = to_next_pos;
                    traj_waypoints[0] = WayPoint(Time::Zero, aj_positions);
                    traj_waypoints[1] = WayPoint(to_pos_time, generate_new_goal_pos(record_positions, num_test_points, iteration_num, pos_order, current_iteration, slop_pos));
                    print("Starting iteration {}: ({}, {}, {}, {})", iteration_num-1, current_iteration[0], current_iteration[1], current_iteration[2], current_iteration[3]);
                    last_iteration = current_iteration;
                    next_point_traj.set_waypoints(5, traj_waypoints, Trajectory::Interp::Linear, max_diff);
                    
                    if(!next_point_traj.validate()){
                        print("Not valid traj");
                    } 
                    state_clock.restart();
                }
                break;
            }
            // MOVING TO NEXT REFERENCE POSITION FOR THE MEII
            case to_next_pos:
                ref = next_point_traj.at_time(current_time);
                meii->set_anat_pos_ctrl_torques(ref);

                if (current_time >= to_pos_time){
                    current_state = command_stim;
                    set_tight_pds(meii);
                    state_clock.restart();
                }

                break;

            // COMMANDING STIM AND RECORDING DATA
            case command_stim:
            {
                // command the exo to stay at the last ref, which would have been the back of the last trajectory
                std::vector<double> anat_command_torques = meii->set_anat_pos_ctrl_torques(ref);

                // if we are in the ramping section, interpolate the stim up to the max value over ramp_time
                if ((current_time < ramp_time) && (current_stim_channel != 0)){
                    stim_vals[current_stim_channel-1] = static_cast<int>(interp(current_time.as_seconds(),0.0,ramp_time.as_seconds(),0.0,max_stim_vals[current_stim_channel-1]));   
                }
                // if we are in the hold section, we command the max stim value for the hold duration
                else if(current_time <= ramp_time+hold_time+collect_time){
                    std::vector<double> data_line;
                    if(current_stim_channel != 0) stim_vals[current_stim_channel-1] = static_cast<int>(max_stim_vals[current_stim_channel-1]);     
                    if (current_time >= ramp_time+hold_time){
                        if (current_stim_channel == 0){
                            data_line.push_back(0);
                            data_line.push_back(0);
                        }
                        else{
                            data_line.push_back(max_stim_vals[current_stim_channel-1]);
                            data_line.push_back(stim_amplitudes[current_stim_channel-1]);
                        }
                        copy(ref.begin(), ref.end()-1, back_inserter(data_line));
                        copy(aj_positions.begin(), aj_positions.end()-1, back_inserter(data_line));
                        copy(anat_command_torques.begin(), anat_command_torques.end()-1, back_inserter(data_line));
                        data.push_back(data_line);
                    }
                }
                // if we are done with all of the portions of stim
                else{
                    // make new filepath for the data
                    std::string curr_it_str = "";
                    for (size_t i = 0; i < current_iteration.size(); i++) curr_it_str += std::to_string(last_iteration[i]);
                    std::string filepath = "C:/Git/FES_Exo/data/S" + std::to_string(subject_num) + "/GPR_Cal/" + curr_it_str + "/" + std::to_string(current_stim_channel) + "_calibration_data_" + currentDateTime() + ".csv";

                    // save file on a new thread and detach it
                    std::thread data_thread(write_to_file, filepath, header, data);
                    data_thread.detach();
                    data.clear();
                    
                    
                    // if we have morechannels to test, go to the next channel
                    // idk why this code structure here is so garbage but for some reason I can't do it another way and this works...
                    bool pass_through = false;
                    if (total_channels_stimmed <= num_channels){
                        do{
                            pass_through = false;
                            if (total_channels_stimmed++ >= num_channels) break;
                            else{
                                pass_through = true;
                                current_stim_channel = randomized_channels[total_channels_stimmed-1];
                            }
                            print("total: {}, current: {}", total_channels_stimmed, current_stim_channel);
                        } while ((!stim_actives[current_stim_channel - 1]));
                    }
                    // however, if we are done testing channels, move to next position and reset stim channel
                    if (total_channels_stimmed > num_channels && !pass_through){ 
                        set_normal_pds(meii);// check if we are done
                        stop = (iteration_num == total_iterations);
                        // if we aren't done
                        if(!stop){
                            current_stim_channel = 0;
                            total_channels_stimmed = 0;
                            std::shuffle(randomized_channels.begin(),randomized_channels.end(),g);
                            current_state = to_next_pos;
                            
                            // generate new trajectory
                            traj_waypoints[0].set_pos(ref);
                            traj_waypoints[1].set_pos(generate_new_goal_pos(record_positions, num_test_points, iteration_num, pos_order, current_iteration, slop_pos));
                            
                            print("Starting iteration {}: ({}, {}, {}, {})", iteration_num-1, current_iteration[0], current_iteration[1], current_iteration[2], current_iteration[3]);
                            last_iteration = current_iteration;

                            next_point_traj.set_waypoints(5, traj_waypoints, Trajectory::Interp::Linear, max_diff);
                            if(!next_point_traj.validate()){
                                print("invalid traj");
                                // stop = true;
                            } 
                        }
                    }
                    // else current_stim_channel = randomized_channels[total_channels_stimmed-1];

                    state_clock.restart();
                }                
                break;
            }

            default:
                break;
        }

        // meii->daq_watchdog_kick();
        // kick watchdog
        if (meii->any_limit_exceeded()) {
            stop = true;
        }

        if (!stop) meii->daq_write_all();

        {
            std::lock_guard<std::mutex> guard(mtx);
            write_vals = stim_vals;
        }
        timer.wait();
    }

    stim_thread.join();
    stim.write_pws(channels, std::vector<unsigned int>(num_channels,0));

    meii->disable();
    meii->daq_disable();

    viz_thread.join();
    stim.disable();

    disable_realtime();

    return 0;
}