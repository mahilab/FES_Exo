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

std::vector<double> test_func(){
    return std::vector<double>();
}

int main(int argc, char* argv[]) {

    std::string model_filepath = "C:/Git/FES_Exo/data/S104/GPR_Cal/Models/";
    // std::cout << "here0";
    SharedController sc(model_filepath);
    // std::cout << "here1";

    std::vector<double> predict_point = {0, 0, 0, 0};

    size_t num_muscles = sc.models.size();
    // std::cout << "here2";
    size_t num_joints = sc.models[0].size();
    
    double prediction;

    std::vector<std::vector<std::vector<double>>> prediction_points;
    
    for (size_t i = 0; i < num_muscles; i++){
        std::vector<std::vector<double>> d1;
        // std::cout << "here1";
        for (size_t j = 0; j < num_joints; j++){
            // std::cout << "here2";
            std::vector<double> d2(4,0);
            for (size_t k = 0; k < 4; k++){
                // std::cout << "here3";
                d2[k] = (random_range(-1.0,1.0));
            }
            d1.push_back(d2);
        }
        prediction_points.push_back(d1);
    }
    // std::cout << "here4";
    int run_times = 10000;
    Clock func_timer;

    for (size_t times = 0; times < run_times; times++){
        for (auto i = 0; i < num_muscles; i++){
            for (auto j = 0; j < num_joints; j++){
                // std::cout << j;
                prediction = sc.models[i][j].predict(prediction_points[i][j]);
            }
        }
    }    

    std::cout << (double)func_timer.get_elapsed_time().as_microseconds()/double(run_times) << std::endl;
    
    prediction = sc.models[0][0].predict(predict_point);
    
    std::cout << prediction << std::endl;

    std::cout << test_func().size();

    return 0;
}