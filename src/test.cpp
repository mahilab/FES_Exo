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
#include <FESExo/MuscleData.hpp>
#include <filesystem>

using namespace mahi::util;

// std::vector<double> test_func(){
//     return std::vector<double>();
// }

int main(int argc, char* argv[]) {

  // make options
  Options options("ex_pos_control_nathan.exe", "Nathan's Position Control Demo");
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
  ("h,help",         "Possible syntaxes: {recruitement_curve.exe (-m) (-v) -s 1001 -i 2}\n"
                                        "{recruitement_curve.exe (-m) (-v) -s 1001 -n 0,1,4 -i 2,3,2}"
                                        "{recruitement_curve.exe (-m) (-v) -s 1001 -n 0,2,4,5 -i 2}"
                                        "{recruitement_curve.exe (-m) (-v) -s 1001 -n 1 -i 3}"
                                        "{recruitement_curve.exe (-m) (-v) -s 1001 -n 1 -p 10,25 -a 60}");

  auto result = options.parse(argc, argv);

  // if -h, print the help option
  if (result.count("help") > 0) {
      print_var(options.help());
      return 0;
  }

  // required parameters. Exit if not provided
  if (!result.count("subject")){
      LOG(Error) << "No subject number input. Exiting program.";
      return 0;
  }
  if (!result.count("muscle_nums")){
      LOG(Error) << "No muscle number input. Exiting program.";
      return 0;
  }
  
  std::vector<int> muscle_nums = result["muscle_nums"].as<std::vector<int>>();

  if (result.count("pulsewidth") + result.count("amplitude") == 1){
    LOG(Error) << "If pulsewidth or amplitude is entered, the other must also be. Exiting program";
    return 0;
  }
  if (result.count("pulsewidth") && muscle_nums.size() != 1){
    LOG(Error) << "If pulsewidth and amplitude are entered, the muscle_nums parameter must be size 1. Exiting program";
    return 0;
  }
  if (result.count("pulsewidth") && result["pulsewidth"].as<std::vector<int>>().size() != 2){
    LOG(Error) << "If the pulsewidth values must be size 2 {pw_min, pw_max}. Exiting program";
    return 0;
  }

  MuscleData muscle_data;

  if (result.count("pulsewidth")){
    std::vector<MuscleInfo> muscle_info;
    auto pulsewidths = result["pulsewidth"].as<std::vector<int>>();
    auto amplitude = result["amplitude"].as<int>();
    muscle_info.emplace_back(muscle_nums[0],"default",amplitude,pulsewidths[0],pulsewidths[1]);
    muscle_data = MuscleData(muscle_info);
  }


  return 0;
}