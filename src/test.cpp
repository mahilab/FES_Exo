/* 
 *  This is the default license template.
 *  
 *  File: test.cpp
 *  Author: Nathan Dunkelberger
 *  Copyright (c) 2020 Nathan Dunkelberger
 *  
 *  To edit this license information: Press Ctrl+Shift+P and press 'Create new License Template...'.
 */

#include <Mahi/Util.hpp>
#include <FESExo/SharedController.hpp>
// #include <Eigen/Dense>
#include <filesystem>

using namespace mahi::util;

// std::vector<double> test_func(){
//     return std::vector<double>();
// }

int main(int argc, char* argv[]) {

    Options options("ex_pos_control_nathan.exe", "Nathan's Position Control Demo");
    options.add_options()
		("t,torque",    "Calibrates the MAHI Exo-II", cxxopts::value<std::vector<double>>());

    auto result = options.parse(argc, argv);

    const size_t num_joints = 4;
    const size_t num_muscles = 8;

    std::string model_filepath = "C:/Git/FES_Exo/data/S9002";
    // std::cout << "here0";
    SharedController sc(num_muscles, num_joints, model_filepath);
    // std::cout << "here1";

    std::vector<double> predict_point  = {-37.5*DEG2RAD, 0, 0, 0};
    std::vector<double> desired_torque = {0.0, 0.0, 0, 0.05};
    std::vector<double> prev_activations(num_muscles, 0.5);
    // std::vector<double> prev_activations = {0.5, 0.5, 0, 0, 0, 0, 0, 0};

    if (result.count("torque")){
        desired_torque = result["torque"].as<std::vector<double>>();
    }

    Clock funcTimer;
    // fesActivation fes_activations = sc.calculate_activations(predict_point, desired_torque);
    // fesPulseWidth pulse_width = sc.calculate_pulsewidths(predict_point, desired_torque);
    sharedTorques st = sc.share_torque(desired_torque, predict_point);
    // std::cout << funcTimer.get_elapsed_time().as_microseconds() << std::endl;

    std::cout << "exo torque:\n" << st.exo_torque;
    std::cout << "\nfes torque:\n" << st.fes_torque;
    std::cout << "\npulsewidth:\n" << st.pulsewidth;

    // std::cout << "activations: \n" << fes_activations.activations << std::endl;
    // std::cout << "torques: \n" << fes_activations.torques << std::endl;

    // std::cout << "\npulsewidths: \n" << pulse_width.pulseWidth << std::endl;
    // std::cout << "torques: \n" /<< pulse_width.torques << std::endl;

    return 0;
}