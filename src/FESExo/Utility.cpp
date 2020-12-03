#include <FESExo/Utility.hpp>
#include <Mahi/Util/Print.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

using namespace mahi::util;

// import subject-specific parameters
std::vector<MuscleInfo> get_muscle_info(const int &subject_num){
    nlohmann::json subject_json;

    std::string import_filepath = "C:/Git/FES_Exo/data/S" + std::to_string(subject_num) + "/Params/S" +  std::to_string(subject_num) + "_params.json";

    std::ifstream param_file(import_filepath);

    param_file >> subject_json;
    param_file.close();

    return subject_json["muscle_info"].get<std::vector<MuscleInfo>>();      
}

void set_tight_pds(std::shared_ptr<meii::MahiExoII> meii){
    //elbow pd
    meii->anatomical_joint_pd_controllers_[0].kp = 120.0; // normally 100.0
    meii->anatomical_joint_pd_controllers_[0].kd = 1.75;  // normally 1.25
    // forearm pd
    meii->anatomical_joint_pd_controllers_[1].kp = 40.0;  // normally 28.0
    meii->anatomical_joint_pd_controllers_[1].kd = 0.40;  // normally 0.20
    // wrist fe pd
    meii->anatomical_joint_pd_controllers_[2].kp = 25.0;  // normally 15.0
    meii->anatomical_joint_pd_controllers_[2].kd = 0.02;  // normally 0.01
    // wrist ru pd
    meii->anatomical_joint_pd_controllers_[3].kp = 25.0;  // normally 15.0
    meii->anatomical_joint_pd_controllers_[3].kd = 0.02;  // normally 0.01
}

void set_normal_pds(std::shared_ptr<meii::MahiExoII> meii){
    //elbow pd
    meii->anatomical_joint_pd_controllers_[0].kp = 100.0; // normally 100.0
    meii->anatomical_joint_pd_controllers_[0].kd = 1.25;  // normally 1.25
    // forearm pd
    meii->anatomical_joint_pd_controllers_[1].kp = 28.0;  // normally 28.0
    meii->anatomical_joint_pd_controllers_[1].kd = 0.20;  // normally 0.20
    // wrist fe pd
    meii->anatomical_joint_pd_controllers_[2].kp = 15.0;  // normally 15.0
    meii->anatomical_joint_pd_controllers_[2].kd = 0.01;  // normally 0.01
    // wrist ru pd
    meii->anatomical_joint_pd_controllers_[3].kp = 15.0;  // normally 15.0
    meii->anatomical_joint_pd_controllers_[3].kd = 0.01;  // normally 0.01
}

mahi::robo::Trajectory get_trajectory(std::string traj_filepath, int n_rows, int n_cols, 
                                      std::vector<std::vector<double>> clamps_rad, 
                                      bool convertdeg2rad, double slop_pos){
    std::vector<std::vector<double>> file_data(n_rows, std::vector<double>(n_cols));

    mahi::util::csv_read_rows(traj_filepath, file_data, 1, 0);

    int path_dimensions = n_cols;
    
    std::vector<mahi::robo::WayPoint> traj_waypoints;
    for (const auto &waypoint : file_data){
        std::vector<double> positions(waypoint.begin()+1,waypoint.end());
        for (size_t i = 0; i < positions.size(); i++){
            if (convertdeg2rad) positions[i]*=DEG2RAD;    
            positions[i] = clamp(positions[i],clamps_rad[i][0],clamps_rad[i][1]);
        }
        positions.push_back(slop_pos);
        
        traj_waypoints.emplace_back(mahi::util::seconds(waypoint[0]), positions);
    }
    
    mahi::robo::Trajectory new_trajectory(path_dimensions, traj_waypoints);
    return new_trajectory;
}