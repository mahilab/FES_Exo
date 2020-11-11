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

struct RcCalData{
  MuscleData muscle_data;
  std::vector<int> iterations;
};

RcCalData parse_args(cxxopts::ParseResult result, Options options){
  int default_iters = 3;

  MuscleData muscle_data_default;
  RcCalData default_data{muscle_data_default, std::vector<int>(0,0)};

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

    std::ifstream file2("my_file.json");

    file2 >> subject_json;
    file2.close();

    auto muscle_info = subject_json["muscle_info"].get<std::vector<MuscleInfo>>();    
    std::vector<MuscleInfo> muscle_info_final;

    for (auto i = 0; i < muscle_info.size(); i++){
      if (muscle_info[i].active && (std::find(muscle_nums.begin(), muscle_nums.end(), i) != muscle_nums.end() || muscle_nums.empty())){
        muscle_info_final.push_back(muscle_info[i]);
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

  return RcCalData{muscle_data, iterations};
}

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
  ("h,help",         "Possible syntaxes: {recruitement_curve.exe (-m) (-v) -s 1001 -i 2}"
                                        "{recruitement_curve.exe (-m) (-v) -s 1001 -n 0,1,4 -i 2,3,2}"
                                        "{recruitement_curve.exe (-m) (-v) -s 1001 -n 0,2,4,5 -i 2}"
                                        "{recruitement_curve.exe (-m) (-v) -s 1001 -n 1 -i 3}"
                                        "{recruitement_curve.exe (-m) (-v) -s 1001 -n 1 -p 10,25 -a 60}");

  auto result = options.parse(argc, argv);

  RcCalData rc_cal_data = parse_args(result, options);

  for (int i = 0; i < rc_cal_data.muscle_data.get_num_muscles(); i++){
    print("num: {}, name: {}, amp: {}, min_pw {}, max_pw {}, active: {}", rc_cal_data.muscle_data.get_muscle_num(i),
                                                                          rc_cal_data.muscle_data.get_muscle_name(i),
                                                                          rc_cal_data.muscle_data.get_amplitude(i),
                                                                          rc_cal_data.muscle_data.get_min_pulsewidth(i),
                                                                          rc_cal_data.muscle_data.get_max_pulsewidth(i),
                                                                          rc_cal_data.muscle_data.get_active(i) );
  }

  print_var(rc_cal_data.iterations);
  
  if (rc_cal_data.iterations.empty()){
    print("iterations vector empty. Exiting program");
    return 0;
  }
  
  return 0;
}