#include <Mahi/FesExo/Utility.hpp>
#include <Mahi/Util/Print.hpp>
#include <Mahi/Util/Math/Functions.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

using namespace mahi::util;

// import subject-specific parameters
std::vector<MuscleInfo> get_muscle_info(std::string import_filepath){
    nlohmann::json subject_json;

    std::ifstream param_file(import_filepath);

    param_file >> subject_json;
    param_file.close();

    return subject_json["muscle_info"].get<std::vector<MuscleInfo>>();      
}

void set_tight_pds(std::shared_ptr<meii::MahiExoII> meii){
    //elbow pd
    meii->anatomical_joint_pd_controllers_[0].kp = 200.0; // normally 100.0
    meii->anatomical_joint_pd_controllers_[0].kd = 2.5;  // normally 1.25
    // forearm pd
    meii->anatomical_joint_pd_controllers_[1].kp = 56.0;  // normally 28.0
    meii->anatomical_joint_pd_controllers_[1].kd = 0.40;  // normally 0.20
    // wrist fe pd
    meii->anatomical_joint_pd_controllers_[2].kp = 30.0;  // normally 15.0
    meii->anatomical_joint_pd_controllers_[2].kd = 0.02;  // normally 0.01
    // wrist ru pd
    meii->anatomical_joint_pd_controllers_[3].kp = 30.0;  // normally 15.0
    meii->anatomical_joint_pd_controllers_[3].kd = 0.02;  // normally 0.01

    print("Tight PDs set");
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

    print("Normal PDs set");
}

void damp_exo(std::shared_ptr<meii::MahiExoII> meii){
    double damp_gain = 5.0;
    //elbow pd
    meii->anatomical_joint_pd_controllers_[0].kp = 100.0; // normally 100.0
    meii->anatomical_joint_pd_controllers_[0].kd = 1.25*damp_gain;  // normally 1.25
    // forearm pd
    meii->anatomical_joint_pd_controllers_[1].kp = 28.0;  // normally 28.0
    meii->anatomical_joint_pd_controllers_[1].kd = 0.20*damp_gain;  // normally 0.20
    // wrist fe pd
    meii->anatomical_joint_pd_controllers_[2].kp = 15.0;  // normally 15.0
    meii->anatomical_joint_pd_controllers_[2].kd = 0.01*damp_gain;  // normally 0.01
    // wrist ru pd
    meii->anatomical_joint_pd_controllers_[3].kp = 15.0;  // normally 15.0
    meii->anatomical_joint_pd_controllers_[3].kd = 0.01*damp_gain;  // normally 0.01
}

mahi::robo::Trajectory get_trajectory(std::string traj_filepath, int n_rows, int n_cols, mahi::util::Time traj_time,
                                      std::vector<std::vector<double>> clamps_rad, 
                                      bool convertdeg2rad, int dof, std::vector<double> zero_positions){
    std::vector<std::vector<double>> file_data(n_rows, std::vector<double>(n_cols));

    mahi::util::csv_read_rows(traj_filepath, file_data, 1, 0);

    int path_dimensions = n_cols-1;

    std::vector<double> min_pos(4,INF);
    std::vector<double> max_pos(4,-INF);

    for (const auto &waypoint : file_data){
        std::vector<double> positions(waypoint.begin()+1,waypoint.end());
        for (auto i = 0; i < positions.size(); i++){
            min_pos[i] = positions[i] < min_pos[i] ? positions[i] : min_pos[i];
            max_pos[i] = positions[i] > max_pos[i] ? positions[i] : max_pos[i];
        }
    }
    
    for (size_t i = 0; i < min_pos.size(); i++)
    {
        if (convertdeg2rad){
            min_pos[i]*=DEG2RAD;
            max_pos[i]*=DEG2RAD;    
        }
    }
    
    std::vector<mahi::robo::WayPoint> traj_waypoints;
    for (const auto &waypoint : file_data){
        std::vector<double> positions(waypoint.begin()+1,waypoint.end());
        for (size_t i = 0; i < positions.size(); i++){
            if (convertdeg2rad) positions[i]*=DEG2RAD;    
            // positions[i] = clamp(positions[i],clamps_rad[i][0],clamps_rad[i][1]);
            double interp_min = min_pos[i] > clamps_rad[i][0] ? min_pos[i] : clamps_rad[i][0];
            double interp_max = max_pos[i] < clamps_rad[i][1] ? max_pos[i] : clamps_rad[i][1];
            positions[i] = interp(positions[i],
                                  min_pos[i],
                                  max_pos[i],
                                  interp_min,
                                  interp_max);

            if ((dof != -1) && (i != dof)){
                positions[i] = zero_positions[i];
            }
        }
        // positions.push_back(slop_pos);
        // print_var(positions);

        auto adjusted_time = interp(waypoint[0], 0.0, 100.0, 0.0, traj_time.as_seconds());
        // std::cout << adjusted_time << std::endl;
        
        traj_waypoints.emplace_back(mahi::util::seconds(adjusted_time), positions);
    }
    
    mahi::robo::Trajectory new_trajectory(path_dimensions, traj_waypoints);
    return new_trajectory;
}