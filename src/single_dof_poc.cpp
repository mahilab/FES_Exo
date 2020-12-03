#include <MEII/MEII.hpp>
#include <Mahi/Com.hpp>
#include <Mahi/Util.hpp>
#include <Mahi/Daq.hpp>
#include <Mahi/Robo.hpp>
#include <Mahi/Com.hpp>
#include <Mahi/Fes.hpp>
#include <FESExo/SharedController.hpp>
#include <FESExo/MuscleData.hpp>
#include <FESExo/Utility.hpp>
#include <vector>

using namespace mahi::util;
using namespace mahi::daq;
using namespace mahi::robo;
using namespace mahi::com;
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

enum state {
    to_neutral_0,     // 0
    to_bottom_elbow,  // 1
    to_top_elbow,     // 2
    to_neutral_1,     // 3
    to_pro_forearm,   // 4
    to_sup_forearm,   // 5
    to_neutral_2,     // 6
    to_flex_wrist,    // 7
    to_ext_wrist,     // 8
    to_neutral_3,     // 9
    to_rad_wrist,     // 10
    to_uln_wrist,     // 11
    to_neutral_4      // 12
};

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool handler(CtrlEvent event) {
    stop = true;
    return true;
}

std::atomic<bool> control_started(false);

std::mutex mtx;
std::vector<unsigned int> write_vals;
std::vector<double> shared_controller_ref(4,0.0);
std::vector<double> shared_controller_joint_positions(4,0.0);
std::vector<double> shared_controller_joint_velocities(4,0.0);
std::vector<double> shared_controller_joint_torques(4,0.0);
std::vector<double> shared_controller_fes_torques(4,0.0);
std::vector<unsigned int> shared_controller_fes_pws(8,0);
void update_stim(std::vector<Channel> channels, Stimulator* stim, SharedController sc, std::vector<double> kp_kd){
    

    std::vector<double> local_joint_torques(4,0.0);
    std::vector<double> local_joint_positions(4,0.0);
    std::vector<double> local_joint_velocities(4,0.0);
    std::vector<double> local_ref(4,0.0);

    double kp_factor = kp_kd[0];
    double kd_factor = kp_kd[1];

    std::vector<std::vector<double>> pd_vals = {{100.0*kp_factor, 1.25*kd_factor}, // elbow fe
                                                {100.0*kp_factor,  0.2*kd_factor}, // forearm ps
                                                { 28.0*kp_factor, 0.01*kd_factor}, // wrist fe
                                                { 15.0*kp_factor, 0.01*kd_factor}};// wrist ru

    std::vector<PdController> fes_controllers;
    for (const auto &pd : pd_vals){
        fes_controllers.emplace_back(PdController(pd[0],pd[1]));
    }    
    
    Timer stim_timer(25_ms);

    while(!stop && control_started){
        {
            // copy mutexed file-scoped input variables to local values as quickly as possible to release mutex
            std::lock_guard<std::mutex> guard(mtx);
            std::copy(shared_controller_joint_torques.begin(),shared_controller_joint_torques.end(),local_joint_torques.begin());
            std::copy(shared_controller_joint_positions.begin(),shared_controller_joint_positions.end(),local_joint_positions.begin());
            std::copy(shared_controller_joint_velocities.begin(),shared_controller_joint_velocities.end(),local_joint_velocities.begin());
            std::copy(shared_controller_ref.begin(),shared_controller_ref.end(),local_ref.begin());
        }

        
        
        // START ACTUALLY SHARING TORQUE WITH EXO
        // sharedTorques shared_torques = sc.share_torque(local_joint_torques, local_joint_positions);
        // END ACTUALLY SHARING TORQUE WITH EXO

        // START LOCAL FES TORQUE CALCS
        std::vector<double> local_fes_torque(4,0.0);
        for (auto i = 0; i < fes_controllers.size(); i++){
            local_fes_torque[i] = fes_controllers[i].calculate(local_ref[i],shared_controller_joint_positions[i],0.0,shared_controller_joint_velocities[i]);
        }
        fesPulseWidth shared_torques = sc.calculate_pulsewidths(local_joint_positions,local_fes_torque);
        // END LOCAL FES TORQUE CALCS

        {
            // copy mutexed local output variables to file-scoped values as quickly as possible to release mutex
            std::lock_guard<std::mutex> guard(mtx);
            std::copy(shared_torques.pulsewidth.begin(),shared_torques.pulsewidth.end(),shared_controller_fes_pws.begin());
            std::copy(shared_torques.fes_torque.begin(),shared_torques.fes_torque.end(),shared_controller_fes_torques.begin());
        }
        
        stim->write_pws(channels,shared_torques.pulsewidth); 
        stim->update();
        stim_timer.wait();
    }
    stim->write_pws(channels,std::vector<unsigned int>(channels.size(),0)); 
    stim->update();
}

void to_state(state& current_state_, const state next_state_, WayPoint current_position_, WayPoint new_position_, Time traj_length_, MinimumJerk& mj_, Clock& ref_traj_clock_) {
    current_position_.set_time(seconds(0));
    new_position_.set_time(traj_length_);
    mj_.set_endpoints(current_position_, new_position_);
    
    if (!mj_.trajectory().validate()) {
        LOG(Warning) << "Minimum Jerk trajectory invalid.";
        stop = true;
    }
    current_state_ = next_state_;
    ref_traj_clock_.restart();
}

int main(int argc, char* argv[]) {
    // register ctrl-c handler
    register_ctrl_handler(handler);

    // make options
    Options options("ex_pos_control_nathan.exe", "Nathan's Position Control Demo");
    options.add_options()
		("c,calibrate", "Calibrates the MAHI Exo-II")
        ("n,no_torque", "trajectories are generated, but not torque provided")
        ("v,virtual", "example is virtual and will communicate with the unity sim")
        ("f,fes_share", "share of torque fes is responsible for", cxxopts::value<double>())
        ("e,exo_share", "share of torque exo is responsible for", cxxopts::value<double>())
        ("s,subject", "subject number, ie. -s 1001",cxxopts::value<int>(), "N")
        ("p,kp_fes", "kp value to use for fes", cxxopts::value<double>())
        ("d,kd_fes", "kd value to use for fes", cxxopts::value<double>())
		("h,help", "./single_dof_poc.exe -f 0.4 -s 9001 (-e 0.8) (-v)");

    auto result = options.parse(argc, argv);

    // if -h, print the help option
    if (result.count("help") > 0) {
        print_var(options.help());
        return 0;
    }

    if (!result.count("subject")){
          LOG(Error) << "No subject number input. Exiting program.";
          return 0;
    }

    /////////////////////////////////
    // construct and config MEII   //
    /////////////////////////////////
    std::shared_ptr<MahiExoII> meii = nullptr;
    std::shared_ptr<Q8Usb> q8 = nullptr;
    
    if(result.count("virtual") > 0){
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

    Time Ts = milliseconds(1);  // sample period for DAQ

    bool rps_is_init = false;

    //////////////////////////////////////////////

    // calibrate - manually zero the encoders (right arm supinated)
    if (result.count("calibrate") > 0) {
        meii->calibrate(stop);
        LOG(Info) << "MAHI Exo-II encoders calibrated.";
        return 0;
    }

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
    
    Stimulator stim("UECU Board", channels, "COM4", "COM5");
    stim.create_scheduler(0xAA, 40); // 40 hz frequency 
    stim.add_events(channels);       // add all channels as events

    // std::vector<unsigned int> stim_amplitudes = {65, 65, 25, 25, 25, 25, 25, 40};

    // initializing chared controller
    double fes_share = (result.count("fes_share") > 0) ? result["fes_share"].as<double>() : 0.5;
    double exo_share = (result.count("exo_share") > 0) ? result["exo_share"].as<double>() : 1.0 - fes_share;
    double kp_fes = (result.count("kp_fes") > 0) ? result["kp_fes"].as<double>() : 0.5;
    double kd_fes = (result.count("kd_fes") > 0) ? result["kd_fes"].as<double>() : kp_fes*0.25;
    std::vector<double> fes_kp_kd = {kp_fes, kd_fes};
    print("exo: {}, fes: {}", exo_share, fes_share);
    int subject_num = result["subject"].as<int>();
    size_t num_muscles = channels.size();
    size_t num_joints = meii->n_aj - 1;

    MuscleData muscle_data(get_muscle_info(subject_num));
    std::vector<bool> muscles_enabled = muscle_data.get_actives();
    std::vector<unsigned int> stim_amplitudes = muscle_data.get_amplitudes();

    // std::vector<bool> muscles_enabled = {true,  // Bicep
    //                                      true, // Tricep
    //                                      true,  // Pronator Teres
    //                                      true,  // Brachioradialis
    //                                      true,  // Flexor Carpi Radialis
    //                                      false,  // Palmaris Longus
    //                                      false,  // Flexor Carpi Radialis
    //                                      true}; // Extensor Carpi Radialis

    std::string model_filepath = "C:/Git/FES_Exo/data/S" + std::to_string(subject_num);  
    SharedController sc(num_joints, muscles_enabled, model_filepath, fes_share, exo_share);

    std::vector<double> local_shared_fes_torques(num_joints,0.0);
    std::vector<unsigned int> local_fes_pws(num_muscles,0);

    // make MelShares
    // MelShare ms_pos("ms_pos");
    // MelShare ms_vel("ms_vel");
    MelShare ms_trq("ms_trq");
    MelShare ms_trq_adj("ms_trq_adj");
    MelShare ms_ref("ms_ref");

    // create ranges for saturating trajectories for safety  MIN            MAX
    std::vector<std::vector<double>> setpoint_rad_ranges = {{-90 * DEG2RAD, 0 * DEG2RAD},
                                                            {-90 * DEG2RAD, 90 * DEG2RAD},
                                                            {-15 * DEG2RAD, 15 * DEG2RAD},
                                                            {-15 * DEG2RAD, 15 * DEG2RAD},
                                                            {0.08, 0.115}};

                                     
    std::vector<Time> state_times = {seconds(2.0), seconds(2.0), seconds(4.0), // elbow    (neutral,   extended,    flexed)
                                     seconds(2.0), seconds(1.5), seconds(3.0), // forearm  (neutral,   pronated, supinated)
                                     seconds(2.0), seconds(1.5), seconds(3.0), // wrist fe (neutral,     flexed,  extended)
                                     seconds(2.0), seconds(1.5), seconds(3.0), // wrist ru (neutral, radial dev, ulnar dev)
                                     seconds(2.0)};                            // return to neutral

    // setup trajectories

    double t = 0;

    Time mj_Ts = milliseconds(50);

    std::vector<double> ref;

    // waypoints                                   Elbow F/E       Forearm P/S    Wrist F/E       Wrist R/U     LastDoF
    WayPoint neutral_point = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD,  00  * DEG2RAD,  00 * DEG2RAD, 0.10});
    WayPoint bottom_elbow  = WayPoint(Time::Zero, {-65 * DEG2RAD,  00 * DEG2RAD,  00  * DEG2RAD,  00 * DEG2RAD, 0.10});
    WayPoint top_elbow     = WayPoint(Time::Zero, { -5 * DEG2RAD,  00 * DEG2RAD,  00  * DEG2RAD,  00 * DEG2RAD, 0.10});
    WayPoint pro_forearm   = WayPoint(Time::Zero, {-35 * DEG2RAD,  30 * DEG2RAD,  00  * DEG2RAD,  00 * DEG2RAD, 0.10});
    WayPoint sup_forearm   = WayPoint(Time::Zero, {-35 * DEG2RAD, -30 * DEG2RAD,  00  * DEG2RAD,  00 * DEG2RAD, 0.10});
    WayPoint flex_wrist    = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD,  15  * DEG2RAD,  00 * DEG2RAD, 0.10});
    WayPoint ext_wrist     = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD, -15  * DEG2RAD,  00 * DEG2RAD, 0.10});
    WayPoint rad_wrist     = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD,  00  * DEG2RAD,  15 * DEG2RAD, 0.10});
    WayPoint uln_wrist     = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD,  00  * DEG2RAD, -15 * DEG2RAD, 0.10});

    // construct timer in hybrid mode to avoid using 100% CPU
    Timer timer(Ts, Timer::Hybrid);
    timer.set_acceptable_miss_rate(0.05);

    // construct clock for regulating keypress
    Clock keypress_refract_clock;
    Time keypress_refract_time = seconds(0.5);

    sharedTorques shared_torques{std::vector<double>(4,0),std::vector<double>(4,0),std::vector<unsigned int>(8,0)};

    ///// DATA COLLECTION /////
    std::string filepath = "C:/Git/FES_Exo/data/data_collection/S" + std::to_string(subject_num) + "/single/f" + std::to_string(std::lround(fes_share*100)) + 
                            "_e" + std::to_string(std::lround(exo_share*100)) + "_kp" + std::to_string(kp_fes) + "_kd" + std::to_string(kd_fes) + "_" + currentDateTime() + ".csv";
    print("filepath: {}",filepath);
    std::vector<std::string> header = {"time [s]", "fes_share []", "exo_share []", "kp_val []", "kd_val []",
                                       "com_elbow_fe [rad]", "com_forearm_ps [rad]", "com_wrist_fe [rad]", "com_wrist_ru [rad]", 
                                       "act_elbow_fe [rad]", "act_forearm_ps [rad]", "act_wrist_fe [rad]", "act_wrist_ru [rad]", 
                                       "elbow_fe_trq [Nm]", "forearm_ps_trq [Nm]", "wrist_fe_trq [Nm]", "wrist_ru_trq [Nm]",
                                       "elbow_fe_fes_trq [Nm]", "forearm_ps_fes_trq [Nm]", "wrist_fe_fes_trq [Nm]", "wrist_ru_fes_trq [Nm]",
                                       "pw_0 [us]", "pw_1 [us]", "pw_2 [us]", "pw_3 [us]", "pw_4 [us]", "pw_5 [us]", "pw_6 [us]", "pw_7 [us]",
                                       "amp_0 [mA]", "amp_1 [mA]", "amp_2 [mA]", "amp_3 [mA]", "amp_4 [mA]", "amp_5 [mA]", "amp_6 [mA]", "amp_7 [mA]"};
    
    std::vector<std::vector<double>> data;

    // std::cout << "here";

    ////////////////////////////////////////////////
    //////////// State Manager Setup ///////////////
    ////////////////////////////////////////////////

    state current_state = to_neutral_0;
    WayPoint current_position;
    WayPoint new_position;
    Time traj_length;
    WayPoint dummy_waypoint = WayPoint(Time::Zero, {-35 * DEG2RAD,  00 * DEG2RAD, 00  * DEG2RAD, 00 * DEG2RAD, 0.09});
    MinimumJerk mj(mj_Ts, dummy_waypoint, neutral_point.set_time(state_times[to_neutral_0]));
	std::vector<double> traj_max_diff = { 50 * DEG2RAD, 50 * DEG2RAD, 35 * DEG2RAD, 35 * DEG2RAD, 0.1 };
	mj.set_trajectory_params(Trajectory::Interp::Linear, traj_max_diff);
    Clock ref_traj_clock;

    std::vector<double> aj_positions(5,0.0);
    std::vector<double> aj_velocities(5,0.0);

    std::vector<double> command_torques(5,0.0);
    std::vector<double> command_torques_adjusted(5,0.0);
    std::vector<double> rps_command_torques(5,0.0);

    ref_traj_clock.restart();

    if (!stim.begin())
        return 0;

    stim.set_amps(channels, stim_amplitudes);

    std::thread viz_thread([&stim]() {
        Visualizer visualizer(&stim);
        visualizer.run();
    });
	
	// enable DAQ and exo
	meii->daq_enable();
	
    meii->enable();

    // std::cout << "here";
	
	// meii->daq_watchdog_start();    

    // trajectory following
    LOG(Info) << "Starting Movement.";

    //initialize kinematics
    meii->daq_read_all();
    meii->update_kinematics();

    WayPoint start_pos(Time::Zero, meii->get_anatomical_joint_positions());

    mj.set_endpoints(start_pos, neutral_point.set_time(state_times[to_neutral_0]));

    // enable Windows realtime
    enable_realtime();
    
    std::thread stim_thread(update_stim, channels, &stim, sc, fes_kp_kd);

    // std::cout << "here";

    while (!stop) {
        // update all DAQ input channels
        meii->daq_read_all();

        // update MahiExoII kinematics
        meii->update_kinematics();
        aj_positions  = meii->get_anatomical_joint_positions();
        aj_velocities = meii->get_anatomical_joint_velocities();

        // update reference from trajectory
        ref = mj.trajectory().at_time(ref_traj_clock.get_elapsed_time());
        
        // calculate anatomical command torques
        if (result.count("no_torque") > 0){
            command_torques = {0.0, 0.0, 0.0, 0.0, 0.0};
            meii->set_anatomical_raw_joint_torques(command_torques);
        }
        else{
            
            command_torques = meii->set_anat_pos_ctrl_torques(ref);
            
            {
                std::lock_guard<std::mutex> guard(mtx);
                std::copy(aj_positions.begin(), aj_positions.end()-1,shared_controller_joint_positions.begin());
                std::copy(aj_velocities.begin(), aj_velocities.end()-1,shared_controller_joint_velocities.begin());
                std::copy(ref.begin(), ref.end()-1,shared_controller_ref.begin());
                std::copy(command_torques.begin(), command_torques.end()-1,shared_controller_joint_torques.begin());
                std::copy(shared_controller_fes_torques.begin(),shared_controller_fes_torques.end(),local_shared_fes_torques.begin());
                std::copy(shared_controller_fes_pws.begin(),shared_controller_fes_pws.end(),local_fes_pws.begin());
                
            }

            control_started = true;

            command_torques_adjusted = command_torques;

            // for (size_t i = 0; i < num_joints; i++){
            //     command_torques_adjusted[i] -= local_shared_fes_torques[i];
            // }
            
            meii->set_anatomical_raw_joint_torques(command_torques_adjusted);
        }

        // record data
        std::vector<double> data_line;
        data_line.push_back(t); // time      
        data_line.push_back(fes_share); // fes share      
        data_line.push_back(exo_share); // exo share        
        data_line.push_back(kp_fes); // fes kp value    
        data_line.push_back(kd_fes); // fes kd value
        // reference
        for (auto i = 0; i < num_joints; i++)
            data_line.push_back(ref[i]); 
        // anatomical joint positions
        for (auto i = 0; i < num_joints; i++)
            data_line.push_back(aj_positions[i]);
        // anatomical robot torque (adjusted for fes)
        for (auto i = 0; i < num_joints; i++)
            data_line.push_back(command_torques_adjusted[i]);
        // anatomical fes torque
        for (auto i = 0; i < num_joints; i++)
            data_line.push_back(local_shared_fes_torques[i]);
        // fes pulsewidths
        for (auto i = 0; i < num_muscles; i++)
            data_line.push_back(static_cast<unsigned int>(local_fes_pws[i]));
        // fes amplitudes
        for (auto i = 0; i < num_muscles; i++)
            data_line.push_back(static_cast<unsigned int>(stim_amplitudes[i]));
        
        data.push_back(data_line);

        // if enough time has passed, continue to the next state. See to_state function at top of file for details
        if (ref_traj_clock.get_elapsed_time() > state_times[current_state]) {

            switch (current_state) {
                //// ELBOW ////
                case to_neutral_0:
                    to_state(current_state, to_bottom_elbow, neutral_point, bottom_elbow, state_times[to_bottom_elbow], mj, ref_traj_clock);
                    break;
                case to_bottom_elbow:
                    to_state(current_state, to_top_elbow, bottom_elbow, top_elbow, state_times[to_top_elbow], mj, ref_traj_clock);
                    break;
                case to_top_elbow:
                    to_state(current_state, to_neutral_1, top_elbow, neutral_point, state_times[to_neutral_1], mj, ref_traj_clock);
                    break;
                //// FOREARM ////
                case to_neutral_1:
                    to_state(current_state, to_pro_forearm, neutral_point, pro_forearm, state_times[to_pro_forearm], mj, ref_traj_clock);
                    break;
                case to_pro_forearm:
                    to_state(current_state, to_sup_forearm, pro_forearm, sup_forearm, state_times[to_sup_forearm], mj, ref_traj_clock);
                    break;
                case to_sup_forearm:
                    to_state(current_state, to_neutral_2, sup_forearm, neutral_point, state_times[to_neutral_2], mj, ref_traj_clock);
                    break;
                //// WRIST ////
                case to_neutral_2:
                    to_state(current_state, to_flex_wrist, neutral_point, flex_wrist, state_times[to_flex_wrist], mj, ref_traj_clock);
                    break;
                case to_flex_wrist:
                    to_state(current_state, to_ext_wrist, flex_wrist, ext_wrist, state_times[to_ext_wrist], mj, ref_traj_clock);
                    break;
                case to_ext_wrist:
                    to_state(current_state, to_neutral_3, ext_wrist, neutral_point, state_times[to_neutral_3], mj, ref_traj_clock);
                    break;
                //// WRIST ////
                case to_neutral_3:
                    to_state(current_state, to_rad_wrist, neutral_point, rad_wrist, state_times[to_rad_wrist], mj, ref_traj_clock);
                    break;
                case to_rad_wrist:
                    to_state(current_state, to_uln_wrist, rad_wrist, uln_wrist, state_times[to_uln_wrist], mj, ref_traj_clock);
                    break;
                case to_uln_wrist:
                    to_state(current_state, to_neutral_4, uln_wrist, neutral_point, state_times[to_neutral_4], mj, ref_traj_clock);
                    break;
                //// NEUTRAL //// 
                case to_neutral_4:
                    stop = true;
                    break;
            }
        }

        // kick watchdog
        // if (!meii->daq_watchdog_kick() || meii->any_limit_exceeded()) {
        if (meii->any_limit_exceeded()) {
            stop = true;
        }
        
        // update all DAQ output channels (only if no limit was exceeded)
        if (!stop) meii->daq_write_all();

        ms_ref.write_data(ref);
        ms_trq.write_data(command_torques);
        ms_trq_adj.write_data(command_torques_adjusted);
        // ms_pos.write_data(meii->get_robot_joint_velocities());

        // wait for remainder of sample period
        t = timer.wait().as_seconds();
    }
    stim_thread.join();

    meii->disable();
    meii->daq_disable();

    stim.disable();

    disable_realtime();

    csv_write_row(filepath, header);
	csv_append_rows(filepath, data);

    viz_thread.join();

    return 0;
}