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
#include <FESExo/MuscleData.hpp>

using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::fes;
using namespace meii;

// Get current date/time, format is YYYY_MM_DD_HH_mm_ss
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

void write_to_file(std::string filepath, std::vector<std::string> header, std::vector<std::vector<double>> data){
    csv_write_row(filepath, header);
	csv_append_rows(filepath, data);
}

struct RcCalData{
  MuscleData muscle_data;
  std::vector<int> iterations;
  int subject_num;
};

RcCalData parse_args(cxxopts::ParseResult result, Options options){
    int default_iters = 3;

    MuscleData muscle_data_default;
    RcCalData default_data{muscle_data_default, std::vector<int>(0,0), 0};

    // if -h, print the help option
    if (result.count("help") > 0) {
          print_var(options.help());
          return default_data;
    }

    // required parameters. Exit if not provided
    if (!result.count("subject")){
          LOG(Error) << "No subject number input. Exiting program.";
          return default_data;
    }

    int subject_number = result["subject"].as<int>();

    std::vector<int> muscle_nums;
    if (result.count("muscle_nums")){
      muscle_nums = result["muscle_nums"].as<std::vector<int>>();
    } 

    if (result.count("pulsewidth") + result.count("amplitude") == 1){
      LOG(Error) << "If pulsewidth or amplitude is entered, the other must also be. Exiting program";
      return default_data;
    }
    if (result.count("pulsewidth") && muscle_nums.size() != 1){
      LOG(Error) << "If pulsewidth and amplitude are entered, the muscle_nums parameter must be size 1. Exiting program";
      return default_data;
    }
    if (result.count("pulsewidth") && result["pulsewidth"].as<std::vector<int>>().size() != 2){
      LOG(Error) << "If the pulsewidth values must be size 2 {pw_min, pw_max}. Exiting program";
      return default_data;
    }

    MuscleData muscle_data;

    if (result.count("pulsewidth")){
      std::vector<MuscleInfo> muscle_info;
      auto pulsewidths = result["pulsewidth"].as<std::vector<int>>();
      auto amplitude = result["amplitude"].as<int>();
      muscle_info.emplace_back(muscle_nums[0],"default",amplitude,pulsewidths[0],pulsewidths[1],true);
      muscle_data = MuscleData(muscle_info);
    }
    else{
      // import muscle data
      nlohmann::json subject_json;
      std::string filepath = "C:/Git/FES_Exo/data/S" + std::to_string(subject_number) + "/Params/S" +  std::to_string(subject_number) + "_params.json";

      std::ifstream param_file(filepath);

      param_file >> subject_json;
      param_file.close();

      auto muscle_info = subject_json["muscle_info"].get<std::vector<MuscleInfo>>();      
      std::vector<MuscleInfo> muscle_info_final;

      for (auto i = 0; i < muscle_info.size(); i++){
          if (muscle_info[i].active && (std::find(muscle_nums.begin(), muscle_nums.end(), i) != muscle_nums.end() || muscle_nums.empty())){
            muscle_info_final.push_back(muscle_info[i]);
          }
          else if(!muscle_info[i].active && (std::find(muscle_nums.begin(), muscle_nums.end(), i) != muscle_nums.end())){
              LOG(Error) << "You entered muscle " << i << ", but that muscle is not active. Please enter a set of valid muscles.";
              return default_data;
          }
      }

      muscle_data = MuscleData(muscle_info_final);
    }

    
    std::vector<int> iterations;
    if(result.count("iterations")){
      iterations = result["iterations"].as<std::vector<int>>();
      if (iterations.size() == 1){
          iterations = std::vector<int>(muscle_data.get_num_muscles(),iterations[0]);
      } 
      else{
          if (iterations.size() != muscle_data.get_num_muscles()){
            LOG(Error) << "iteration size and muscle number size must be the same. Exiting program";
            return default_data;
          }
      }
    }
    else{
      iterations = std::vector<int>(muscle_data.get_num_muscles(),default_iters);
    }

    return RcCalData{muscle_data, iterations, subject_number};
}

void add_step(std::vector<WayPoint> &stim_waypoints, const Time& start_time, const Time& step_dur, WayPoint low, WayPoint high){
    stim_waypoints.push_back(low.set_time(start_time));                      // low  at start point
    stim_waypoints.push_back(high.set_time(start_time + 1_ms));              // high at start point + dt
    stim_waypoints.push_back(high.set_time(start_time + step_dur));       // high at start point + step_dur
    stim_waypoints.push_back(low.set_time(start_time + step_dur + 1_ms)); // low  at start point + step_dur + dt
    return;
}

void add_ramp(std::vector<WayPoint> &stim_waypoints, const Time& start_time, const Time& half_ramp_time, WayPoint low, WayPoint high){
    stim_waypoints.push_back(low.set_time(start_time));                      // low  at start point
    stim_waypoints.push_back(high.set_time(start_time + half_ramp_time));    // high at start point + dt
    stim_waypoints.push_back(low.set_time(start_time + half_ramp_time*2.0)); // high at start point + impulse_dur
    return;
}

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

std::mutex mtx;
std::vector<unsigned int> write_stim_vals;
std::vector<unsigned int> write_amp_vals;
void update_stim(std::vector<Channel> channels, Stimulator* stim){
    std::vector<unsigned int> local_write_vals(channels.size(),0);
    std::vector<unsigned int> local_amp_vals(channels.size(),0);
    Timer stim_timer(50_ms);

    while(!stop){
        {
            std::lock_guard<std::mutex> guard(mtx);
            copy(write_stim_vals.begin(), write_stim_vals.end(), local_write_vals.begin());
            copy(write_amp_vals.begin(), write_amp_vals.end(), local_amp_vals.begin());            
        }
        stim->write_pws(channels,local_write_vals);
        stim->set_amps(channels,local_amp_vals);
        stim->update();
        stim_timer.wait();
    }
    stim->write_pws(channels,std::vector<unsigned int>(channels.size(),0)); 
    stim->update();
}

int main(int argc, char* argv[]) {
    // register ctrl-c handler
    register_ctrl_handler(handler);

    // make options
    Options options("recruitment_curve.exe", "Recruitment Curve Data Collection");
    options.add_options()
    ("c,calibrate",    "Calibrates the MAHI Exo-II")
    ("m,virtual_meii", "meii is virtual and will communicate with the unity sim")
    ("f,virtual_fes",  "fes is virtual and will not wait for responses from the board")
    ("v,visualize",    "fes visualizer will be used to show the current status")
    ("s,subject",      "subject number, ie. -s 1001",                                                cxxopts::value<int>())
    ("n,muscle_nums",  "muscle(s) targeted (optional: all if not specified), ie. -n 1, or -n 0,1,4", cxxopts::value<std::vector<int>>())
    ("i,iterations",   "number of iterations for each muscle or all muscles "
                        "(optional: all 3 if not specified), ie. -i 2 or -i 2,3,2",                   cxxopts::value<std::vector<int>>())
    ("p,pulsewidth",   "min and max pulsewidth (only for single muscle), ie. -p 10,25",              cxxopts::value<std::vector<int>>())
    ("a,amplitude",    "amplitude (only for single muscle), ie. -a 60",                              cxxopts::value<int>())
    ("h,help",         "Possible syntaxes: {recruitement_curve.exe (-m) (-v) -s 1001 -i 2} "
                                            "{recruitement_curve.exe (-m) (-v) -s 1001 -n 0,1,4 -i 2,3,2} "
                                            "{recruitement_curve.exe (-m) (-v) -s 1001 -n 0,2,4,5 -i 2} "
                                            "{recruitement_curve.exe (-m) (-v) -s 1001 -n 1 -i 3} "
                                            "{recruitement_curve.exe (-m) (-v) -s 1001 -n 1 -p 10,25 -a 60} ");

    auto result = options.parse(argc, argv);

    RcCalData rc_cal_data = parse_args(result, options);
    if (rc_cal_data.iterations.empty()){
        LOG(Error) << "The calibration data struct is unusable. Please check command line args.";
        return 0;
    }

    // if -h, print the help option
	if (result.count("help") > 0) {
		print_var(options.help());
		return 0;
	}
    
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
		meii->calibrate(stop);
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
    bool visualizer_on = (result.count("visualize") > 0);
    
    Stimulator stim("UECU Board", channels, "COM5", "COM8");
    stim.create_scheduler(0xAA, 40); // 40 hz frequency 
    stim.add_events(channels);       // add all channels as events

    uint8 num_channels = static_cast<uint8>(channels.size());

    std::vector<unsigned int> stim_amplitudes(num_channels,0);
    
    std::thread viz_thread([&stim]() {
        Visualizer visualizer(&stim);
        visualizer.run();
    });

    // END INITIALIZE STIMULATOR

    // stim_amplitudes[muscle_num] = amplitude;

    double slop_pos = 0.11; // position for the forearm slop position

    std::vector<double> max_diff = { 40 * DEG2RAD, 40 * DEG2RAD, 20 * DEG2RAD, 20 * DEG2RAD, 0.01 };
    std::vector<double> neutral_pos = {-PI/4,     0, 0, 0, slop_pos};

    Trajectory traj_stim;

    Time init_delay  = 1_s;
    Time cond_time   = 1_s;
    Time cond_rest   = 5_s;
    Time pulse_rest  = 5_s;
    Time ramp_half   = 2_s;
    Time ramp_rest   = 1_s;
    Time impulse_dur = 200_ms; // was using 100_ms before

    int current_muscle_num = 0;
    int actual_muscle_num = 0;
    int current_iteration = 0;

    std::vector<WayPoint> traj_waypoints;
    Time to_pos_time = 5_s;

    Trajectory next_point_traj;

    std::vector<double> ref(5,0);

    enum state {
        setup,        // 0
        to_pos,       // 1
        stim_config,  // 2
        command_stim, // 3
    };

    state current_state = setup;

    std::vector<double> aj_positions(5,0);

    // data collection setup
    // make new filepath for the data
    // std::string filepath = "C:/Git/FES_Exo/data/S" + std::to_string(rc_cal_data.subject_num) + "/RC_Cal/" 
    //                      + std::to_string(muscle_num) + "_" + channels[muscle_num].get_channel_name() + "/" 
    //                      + "amp_" + std::to_string(stim_amplitudes[muscle_num]) + "_pw_" + std::to_string(min_pulse_width) + "_" + std::to_string(max_pulse_width) + "_rc_calibration_data_" + currentDateTime() + ".csv";
    std::vector<std::vector<double>> data;
    std::vector<std::string> header = {"time [s]", "stim_pw [us]", "stim_amp [mA]", "com_elbow_fe [rad]", "com_forearm_ps [rad]", "com_wrist_fe [rad]", "com_wrist_ru [rad]", "act_elbow_fe [rad]", "act_forearm_ps [rad]", "act_wrist_fe [rad]", "act_wrist_ru [rad]", "elbow_fe_trq [Nm]", "forearm_ps_trq [Nm]", "wrist_fe_trq [Nm]", "wrist_ru_trq [Nm]"};

    Time t = 0_s;
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
    print("Starting Muscle {}, iteration {}.", current_muscle_num, current_iteration + 1);

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
                    current_state = to_pos;
                    traj_waypoints.push_back(WayPoint(Time::Zero, aj_positions));
                    traj_waypoints.push_back(WayPoint(to_pos_time, neutral_pos));
                    next_point_traj.set_waypoints(5, traj_waypoints, Trajectory::Interp::Linear, max_diff);
                    if(!next_point_traj.validate()){
                        print("Not valid traj");
                    } 
                    //elbow pd
                    meii->anatomical_joint_pd_controllers_[0].kp = 150.0; // normally 100.0
                    meii->anatomical_joint_pd_controllers_[0].kd = 2.00;  // normally 1.25
                    // forearm pd
                    meii->anatomical_joint_pd_controllers_[1].kp = 50.0;  // normally 28.0
                    meii->anatomical_joint_pd_controllers_[1].kd = 0.40;  // normally 0.20
                    // wrist fe pd
                    meii->anatomical_joint_pd_controllers_[2].kp = 30.0;  // normally 15.0
                    meii->anatomical_joint_pd_controllers_[2].kd = 0.02;  // normally 0.01
                    // wrist ru pd
                    meii->anatomical_joint_pd_controllers_[3].kp = 30.0;  // normally 15.0
                    meii->anatomical_joint_pd_controllers_[3].kd = 0.02;  // normally 0.01

                    state_clock.restart();
                }
                break;
            }
            // MOVING TO NEUTRAL POSITION FOR THE MEII
            case to_pos:
                ref = next_point_traj.at_time(current_time);
                meii->set_anat_pos_ctrl_torques(ref);
                
                if (current_time >= to_pos_time){
                    current_state = stim_config;
                    state_clock.restart();
                }
                break;
            
            case stim_config:
            {
                actual_muscle_num = rc_cal_data.muscle_data.get_muscle_num(current_muscle_num);
                stim_vals = std::vector<unsigned int>(num_channels,0);

                stim_amplitudes = std::vector<unsigned int>(num_channels,0);
                stim_amplitudes[actual_muscle_num] = rc_cal_data.muscle_data.get_amplitude(current_muscle_num);

                std::vector<double> min_stim = {static_cast<double>(rc_cal_data.muscle_data.get_min_pulsewidth(current_muscle_num))};
                std::vector<double> max_stim = {static_cast<double>(rc_cal_data.muscle_data.get_max_pulsewidth(current_muscle_num))};

                WayPoint low(Time::Zero, min_stim);
                WayPoint high(Time::Zero, max_stim);

                std::vector<WayPoint> stim_waypoints;

                add_step(stim_waypoints,  1_s,   cond_time, low, high); // adds   step  starting at 1  s
                add_step(stim_waypoints,  7_s, impulse_dur, low, high); // adds impulse starting at 7  s
                add_step(stim_waypoints,  9_s, impulse_dur, low, high); // adds impulse starting at 9  s
                add_step(stim_waypoints, 11_s, impulse_dur, low, high); // adds impulse starting at 11 s
                add_step(stim_waypoints, 13_s, impulse_dur, low, high); // adds impulse starting at 13 s
                add_ramp(stim_waypoints, 15_s,   ramp_half, low, high); // adds   ramp  starting at 15 s // 15 for 4s
                add_ramp(stim_waypoints, 21_s,   ramp_half, low, high); // adds   ramp  starting at 21 s // 25 for 4s
                add_ramp(stim_waypoints, 27_s,   ramp_half, low, high); // adds   ramp  starting at 27 s // 35 for 4s
                add_ramp(stim_waypoints, 33_s,   ramp_half, low, high); // adds   ramp  starting at 33 s // 45 for 4s    

                traj_stim.set_waypoints(1, stim_waypoints, Trajectory::Interp::Linear);
                current_state = command_stim;
                
                state_clock.restart();
                break;
            }

            // COMMANDING STIM AND RECORDING DATA
            case command_stim:
            {
                // command the exo to stay at the last ref, which would have been the back of the last trajectory
                std::vector<double> anat_command_torques = meii->set_anat_pos_ctrl_torques(ref);

                // if we are in the ramping section, interpolate the stim up to the max value over ramp_time
                if (current_time < traj_stim.back().when() + ramp_rest){
                    // if we are in the stim command section, we command the stim value at the current time
                    stim_vals[actual_muscle_num] = static_cast<int>(traj_stim.at_time(current_time).front()); // use front because it is a size-1 vector
                    // prepare data collection
                    std::vector<double> data_line;
                    data_line.push_back(t.as_seconds()); // add stim pw value
                    data_line.push_back(stim_vals[actual_muscle_num]); // add stim pw value
                    data_line.push_back(stim_amplitudes[actual_muscle_num]); // add stim amp value
                    copy(ref.begin(), ref.end()-1, back_inserter(data_line)); // add reference position
                    copy(aj_positions.begin(), aj_positions.end()-1, back_inserter(data_line)); // add current position
                    copy(anat_command_torques.begin(), anat_command_torques.end()-1, back_inserter(data_line)); // add command torques
                    data.push_back(data_line); // add full data line to the data container
                }
                // if we are done with all of the portions of stim
                else{
                    std::string filepath = "C:/Git/FES_Exo/data/S" + std::to_string(rc_cal_data.subject_num) + "/RC_Cal/" 
                        + std::to_string(actual_muscle_num) + "_" + channels[actual_muscle_num].get_channel_name() + "/" 
                        + "amp_" + std::to_string(stim_amplitudes[actual_muscle_num]) + "_pw_" + std::to_string(rc_cal_data.muscle_data.get_min_pulsewidth(current_muscle_num)) + "_" + std::to_string(rc_cal_data.muscle_data.get_max_pulsewidth(current_muscle_num)) + "_rc_calibration_data_" + currentDateTime() + ".csv";
                    std::thread data_thread(write_to_file, filepath, header, data);
                    data_thread.detach();
                    data.clear();
                    
                    // check if we are done with iterations for this muscle
                    if (current_iteration < rc_cal_data.iterations[current_muscle_num] - 1){
                        current_iteration++;
                    }
                    else{
                        // check if we are done with muscles in the list
                        if (current_muscle_num < rc_cal_data.muscle_data.get_num_muscles() - 1){
                            current_state = stim_config;
                            current_muscle_num++;
                            current_iteration = 0;
                        }
                        else stop = true;
                    }
                    print("Starting Muscle {}, iteration {}.", current_muscle_num, current_iteration + 1);
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
            write_amp_vals = stim_amplitudes;
            write_stim_vals = stim_vals;
        }
        t = timer.wait();
    }
    stim_thread.join();

    meii->disable();
    meii->daq_disable();
    
    disable_realtime();

    stim.disable();

    viz_thread.join();  

    // write_to_file(filepath, header, data);

    return 0;
}