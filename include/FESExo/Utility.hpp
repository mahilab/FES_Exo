#pragma once

#include <FESExo/MuscleData.hpp>
#include <Mahi/Robo/Trajectories/Trajectory.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <Mahi/Util/Logging/Csv.hpp>

using mahi::util::INF;

std::vector<MuscleInfo> get_muscle_info(const int &subject_num);

void set_tight_pds(std::shared_ptr<meii::MahiExoII> meii);
void set_normal_pds(std::shared_ptr<meii::MahiExoII> meii);

void damp_exo(std::shared_ptr<meii::MahiExoII> meii);

mahi::robo::Trajectory get_trajectory(std::string traj_filepath, int n_rows, int n_cols, 
                                      std::vector<std::vector<double>> clamps_rad = {{-INF,INF},{-INF,INF},{-INF,INF},{-INF,INF}}, 
                                      bool convertdeg2rad = false, double slop_pos = 0.1);