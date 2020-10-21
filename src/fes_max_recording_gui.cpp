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

std::vector<double> torques(4,0.0);
std::vector<unsigned int> amps(8,0);
std::vector<unsigned int> pws(8,0);
std::vector<std::string> names;
std::mutex mtx;

class FesMaxRecordingGui : public Application
{
private:
    Clock m_elapse_clock;

    struct ScrollingData {
        int MaxSize = 1000;
        int Offset  = 0;
        ImVector<ImVec2> Data;
        ScrollingData() { Data.reserve(MaxSize); }
        void AddPoint(float x, float y) {
            if (Data.size() < MaxSize)
                Data.push_back(ImVec2(x,y));
            else {
                Data[Offset] = ImVec2(x,y);
                Offset =  (Offset + 1) % MaxSize;
            }
        }
    };

    std::vector<ScrollingData> m_torque_data;
    std::vector<double> m_torques;
    std::vector<double> m_zero_torques;
    bool m_open = true;

    uint16 muscle_num;
    std::string muscle_name;
    int muscle_pw;
    int muscle_amp;

    void zero();
    void reset();

    std::vector<std::string> m_joint_names = {"Elbow F/E", "Forearm P/S", "Wrist F/E", "Wrist R/U"};

public:
    FesMaxRecordingGui();
    ~FesMaxRecordingGui();
    void update() override;
};

FesMaxRecordingGui::FesMaxRecordingGui():
    Application(500,500,"Max Recording GUI"),
    m_torques(4,0.0),
    m_zero_torques(4,0.0),
    muscle_amp(0),
    muscle_pw(0)
{
    ImGui::StyleColorsDark();
    m_elapse_clock.restart();
    
    for (size_t i = 0; i < 4; i++){
        m_torque_data.emplace_back();
    }
}

FesMaxRecordingGui::~FesMaxRecordingGui()
{
}

void FesMaxRecordingGui::zero(){
    m_zero_torques = m_torques;
}

void FesMaxRecordingGui::reset(){
    m_zero_torques = std::vector<double>(4,0.0);
    muscle_amp = 0;
    muscle_pw  = 0;

    for (auto &&torque_data : m_torque_data){
        torque_data.Data.clear();
    }
   
    {
        std::lock_guard<std::mutex> lock_guard(mtx);
        amps = std::vector<unsigned int>(8,0);
        pws = std::vector<unsigned int>(8,0);
    }
    
}

void FesMaxRecordingGui::update(){
    ImGui::Begin("FES Stim", &m_open);

    float t = (float)m_elapse_clock.get_elapsed_time().as_seconds();

    {
        std::lock_guard<std::mutex> lock(mtx);
        std::copy(torques.begin(),torques.end(),m_torques.begin());
    }

    for (auto i = 0; i < 4; i++) {
        m_torque_data[i].AddPoint(t,m_torques[i]-m_zero_torques[i]);
    }
    
    ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 3);
    
    ImPlot::SetNextPlotLimitsX(t - 10, t, ImGuiCond_Always);
    ImPlot::SetNextPlotLimitsY(-2,2);
    if(ImPlot::BeginPlot("##FES Plot", "Time (s)", "Amplitude(mA)", {400,400}, 0, 0, 0)){
        for (size_t i = 0; i < 4; i++) {
            ImPlot::PlotLine((m_joint_names[i]).c_str(), &m_torque_data[i].Data[0].x, &m_torque_data[i].Data[0].y, m_torque_data[i].Data.size(), m_torque_data[i].Offset, 2 * sizeof(float));
        }
        ImPlot::EndPlot();
    }
    // ImPlot::PopStyleVar();

    ImGui::SameLine();
    ImGui::BeginGroup();

    // dropdown menu for selecting muscle
    ImGui::TextUnformatted("Select Muscle: ");
    ImGui::SameLine();
    if (ImGui::Button(names[muscle_num].c_str()))
            ImGui::OpenPopup("Muscle Options");
    // ImGui::TextUnformatted(names[muscle_num].c_str());
    if (ImGui::BeginPopup("Muscle Options"))
    {
        ImGui::Text("Muscles");
        ImGui::Separator();
        for (int i = 0; i < names.size(); i++)
            if (ImGui::Selectable(names[i].c_str())){
                if (muscle_num != i) reset();
                muscle_num = i;
            }
        ImGui::EndPopup();
    }
    ImGui::PushItemWidth(100);
    ImGui::InputInt("amplitude", &muscle_amp, 1, 70);
    ImGui::InputInt("pulsewidth", &muscle_pw, 1, 70);
    ImGui::PopItemWidth();

    if (ImGui::Button("Zero")) zero();
    ImGui::SameLine();
    if (ImGui::Button("Reset")) reset();
    ImGui::EndGroup();

    ImGui::End();

    muscle_amp = clamp(muscle_amp,0,70);
    muscle_pw = clamp(muscle_pw,0,60);

    {
        std::lock_guard<std::mutex> lock(mtx);
        amps[muscle_num] = muscle_amp;
        pws[muscle_num] = muscle_pw;
    }

    if (!m_open || stop) {
        reset();
        stop = true;
        quit();
    }
}

void update_stim(Stimulator* stim){
    std::vector<unsigned int> local_amps(8,0);
    std::vector<unsigned int> local_pws(8,0);

    Timer stim_timer(25_ms);
    
    Clock func_clock;
    while (!stop){
        {
            std::lock_guard<std::mutex> lock(mtx);
            std::copy(amps.begin(),amps.end(),local_amps.begin());
            std::copy(pws.begin(),pws.end(),local_pws.begin());
        }
        
        stim->set_amps(stim->get_channels(),local_amps);
        stim->write_pws(stim->get_channels(),local_pws);
        stim->update();

        stim_timer.wait();
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
    Stimulator stim("UECU Board", channels, "COM5", "COM8");

    // Initialize scheduler with the sync character and frequency of scheduler in hertz
    stim.create_scheduler(0xAA, 40);

    // Input which events will be added to the scheduler for updates
    stim.add_events(channels);

    for (auto &channel : channels){
        names.push_back(channel.get_channel_name() + " (" + std::to_string(channel.get_channel_num()) + ")");
    }

    std::thread gui_thread([]() {
        FesMaxRecordingGui gui;
        gui.run();
    });

    // start the visualization thread to run the gui. This is optional, but allows the stimulators to be updated through
    // the gui rather than in code. The code will overwrite the gui, so if gui control is desired, remove updates in the
    // while loop below
    // std::thread viz_thread([&stim]() {
    //     Visualizer visualizer(&stim);
    //     visualizer.run();
    // });

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
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    std::copy(anat_command_torques.begin(),anat_command_torques.end()-1,torques.begin());
                }
                
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

    // viz_thread.join();
    gui_thread.join();
    stim_thread.join();

    disable_realtime();

    

    return 0;
}
