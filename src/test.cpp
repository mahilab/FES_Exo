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
#include <FESExo/FesGprModel.hpp>
#include <FESExo/SharedController.hpp>
#include <Eigen/Dense>
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

    std::string model_filepath = "C:/Git/FES_Exo/data/S104/GPR_Cal/Models/";
    // std::cout << "here0";
    SharedController sc(num_muscles, num_joints, model_filepath);
    // std::cout << "here1";

    std::vector<double> predict_point  = {0, 0, 0, 0};
    std::vector<double> desired_torque = {0.1, 0, 0, 0};
    std::vector<double> prev_activations(num_muscles, 0.5);

    if (result.count("torque")){
        desired_torque = result["torque"].as<std::vector<double>>();
    }

    Clock funcTimer;
    fesActivation activations = sc.calculateActivations(predict_point, desired_torque, prev_activations);
    std::cout << funcTimer.get_elapsed_time().as_microseconds() << std::endl;

    for (const auto &activation : activations.activations){
        std::cout << activation << ", ";
    }

    std::cout << std::endl;

    for (const auto &torque : activations.torques){
        std::cout << torque << ", ";
    }

    return 0;
}