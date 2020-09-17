// #define MAHI_GUI_NO_CONSOLE
#include <Mahi/Fes.hpp>
#include <Mahi/Gui.hpp>
#include <Mahi/Util.hpp>
// #include <MahiExoII/MahiExoII.hpp>
#include <atomic>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>

using namespace mahi::util;
using namespace mahi::gui;
using namespace mahi::fes;
// using namespace meii;

// create global stop variable CTRL-C handler function
ctrl_bool stop(false);
bool      handler(CtrlEvent event) {
    stop = true;
    return true;
}

int main(int argc, char const *argv[]) {
    // register ctrl-c handler
    register_ctrl_handler(handler);

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

    // start the visualization thread to run the gui. This is optional, but allows the stimulators to be updated through
    // the gui rather than in code. The code will overwrite the gui, so if gui control is desired, remove updates in the
    // while loop below
    std::thread viz_thread([&stim]() {
        Visualizer visualizer(&stim);
        visualizer.run();
    });

    // start sending stimulation to the board
    stim.begin();

    // set timer for our control loop
    Timer timer(milliseconds(50), Timer::WaitMode::Hybrid);
    timer.set_acceptable_miss_rate(0.05);
    
    // variable to keep track of our current time
    double t(0.0);

    while (!stop) {
        stim.update();
        t = timer.wait().as_seconds();
    }

    viz_thread.join();

    return 0;
}
