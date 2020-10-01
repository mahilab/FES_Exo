// #define MAHI_GUI_NO_CONSOLE
#include <MEII/MEII.hpp>
#include <Mahi/Fes.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Daq.hpp>
#include <atomic>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>

using namespace mahi::util;
using namespace mahi::gui;
using namespace mahi::fes;
using namespace mahi::daq;
using namespace meii;

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool      handler(CtrlEvent event) {
    stop = true;
    return true;
}

void update_stim(Stimulator* stim){
    while (!stop){
        stim->update();
    }    
    stim->write_pws(stim->get_channels(),std::vector<unsigned int>(stim->get_channels().size(),0)); 
    stim->update();
}

int main(int argc, char *argv[]) {
    // register ctrl-c handler
    register_ctrl_handler(handler);

    Options options("ex_pos_control_nathan.exe", "Nathan's Position Control Demo");
    options.add_options()
		("c,calibrate",    "Calibrates the MAHI Exo-II")
        ("m,virtual_meii", "meii is virtual and will communicate with the unity sim")
        ("h,help",         "prints out this help message");

    auto result = options.parse(argc, argv);

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

    std::vector<WayPoint> traj_waypoints;
    Time to_pos_time = 5_s;
    Trajectory next_point_traj;
    double slop_pos = 0.11; // position for the forearm slop position

    std::vector<double> max_diff = { 40 * DEG2RAD, 40 * DEG2RAD, 20 * DEG2RAD, 20 * DEG2RAD, 0.01 };

    std::vector<double> ref(5,0);

    enum state {
        setup,        // 0
        to_pos,       // 1
        command_stim, // 2
    };

    state current_state = setup;

    std::vector<double> aj_positions(5,0);

    // create channels of interest
    std::vector<Channel> channels;
    // Channel 1 - Bicep
    Channel bicep("Bicep", CH_1, AN_CA_1, 100, 100); 
    channels.push_back(bicep);
    // Channel 2 - Tricep
    Channel tricep("Tricep", CH_2, AN_CA_2, 100, 100);
    channels.push_back(tricep);
    // Channel 3 - Pronator Teres
    Channel pronator_teres("Pronator Teres", CH_3, AN_CA_3, 100, 100);
    channels.push_back(pronator_teres);
    // Channel 4 - "Brachioradialis"
    Channel brachioradialis("Brachioradialis", CH_4, AN_CA_4, 100, 100);
    channels.push_back(brachioradialis);
    // Channel 5 - Flexor Carpi Radialis
    Channel flexor_carpi_radialis("Flexor Carpi Radialis", CH_5, AN_CA_1, 100, 100);
    channels.push_back(flexor_carpi_radialis);
    // Channel 6 - Palmaris Longus
    Channel palmaris_longus("Palmaris Longus", CH_6, AN_CA_2, 100, 100);
    channels.push_back(palmaris_longus);
    // Channel 7 - Flexor Carpi Ulnaris
    Channel flexor_carpi_ulnaris("Flexor Carpi Ulnaris", CH_7, AN_CA_3, 100, 100);
    channels.push_back(flexor_carpi_ulnaris);
    // Channel 8 - Extensor Carpi Radialis Longus
    Channel extensor_carpi_radialis_longus("Extensor Carpi Radialis Longus", CH_8, AN_CA_4, 100, 100);
    channels.push_back(extensor_carpi_radialis_longus);

    // Create stim board with a name, comport, and channels to add
    Stimulator stim("UECU Board", channels, "COM4", "COM5");

    // Initialize scheduler with the sync character and frequency of scheduler in hertz
    stim.create_scheduler(0xAA, 40);

    // Input which events will be added to the scheduler for updates
    stim.add_events(channels);

    // start the visualization thread to run the gui. This is optional, but allows the stimulators to be updated through
    // the gui rather than in code. The code will overwrite the gui, so if gui control is desired, remove updates in the
    // while loop below
    std::thread viz_thread([&stim]() {
        Visualizer visualizer(&stim);
        visualizer.run();
    });

    // start sending stimulation to the board
    stim.begin();

    // enable DAQ and exo
	meii->daq_enable();
	meii->enable();
	// meii->daq_watchdog_start();  

    meii->daq_read_all();
    meii->update_kinematics();

    // set timer for our control loop
    Time t = 0_s;
    Timer timer(1_ms);
    // timer.disable_warnings();
    Clock state_clock;

    meii->rps_init_par_ref_.start(meii->get_wrist_parallel_positions(), state_clock.get_elapsed_time());

    std::thread stim_thread(update_stim, &stim);

    enable_realtime();

    while (!stop) {
        meii->daq_read_all();
        meii->update_kinematics();

        aj_positions = meii->get_anatomical_joint_positions();

        Time current_time = state_clock.get_elapsed_time();

        switch (current_state){
            // INITIALIZING RPS  MECHANISM
            case setup:
            {
                std::vector<double> trqs = meii->set_robot_smooth_pos_ctrl_torques(meii->rps_init_par_ref_, current_time);

                if (meii->check_rps_init()){
                    print("RPS initialized");
                    current_state = to_pos;
                    traj_waypoints.push_back(WayPoint(Time::Zero, aj_positions));
                    traj_waypoints.push_back(WayPoint(to_pos_time, {-PI/4, 0, 0, 0, slop_pos}));
                    next_point_traj.set_waypoints(5, traj_waypoints, Trajectory::Interp::Linear, max_diff);
                    if(!next_point_traj.validate()){
                        print("Not valid traj");
                    } 
                    //elbow pd
                    // meii->anatomical_joint_pd_controllers_[0].kp = 150.0; // normally 100.0
                    // meii->anatomical_joint_pd_controllers_[0].kd = 2.00;  // normally 1.25
                    // // forearm pd
                    // meii->anatomical_joint_pd_controllers_[1].kp = 50.0;  // normally 28.0
                    // meii->anatomical_joint_pd_controllers_[1].kd = 0.40;  // normally 0.20
                    // // wrist fe pd
                    // meii->anatomical_joint_pd_controllers_[2].kp = 30.0;  // normally 15.0
                    // meii->anatomical_joint_pd_controllers_[2].kd = 0.02;  // normally 0.01
                    // // wrist ru pd
                    // meii->anatomical_joint_pd_controllers_[3].kp = 30.0;  // normally 15.0
                    // meii->anatomical_joint_pd_controllers_[3].kd = 0.02;  // normally 0.01

                    state_clock.restart();
                }
                break;
            }
            // MOVING TO NEUTRAL POSITION FOR THE MEII
            case to_pos:
                ref = next_point_traj.at_time(current_time);
                meii->set_anat_pos_ctrl_torques(ref);
                
                if (current_time >= to_pos_time){
                    current_state = command_stim;
                    state_clock.restart();
                }
                break;
            // COMMANDING STIM AND RECORDING DATA
            case command_stim:
            {
                // command the exo to stay at the last ref, which would have been the back of the last trajectory
                std::vector<double> anat_command_torques = meii->set_anat_pos_ctrl_torques(ref);
                // for (const auto &torque : anat_command_torques){
                //     std::cout << torque << ", ";
                // }
                // std::cout << t << std::endl;
                
                break;
            }
            default:
                break;
        }

        if (meii->any_limit_exceeded()) {
            stop = true;
        }        
        
        if (!stop) meii->daq_write_all();

        t = timer.wait();
    }

    meii->disable();
    meii->daq_disable();

    stim.disable();

    viz_thread.join();
    stim_thread.join();

    disable_realtime();

    

    return 0;
}
