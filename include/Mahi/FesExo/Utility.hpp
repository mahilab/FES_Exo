#pragma once

#include <Mahi/FesExo/MuscleData.hpp>
#include <Mahi/Robo/Trajectories/Trajectory.hpp>
#include <MEII/MahiExoII/MahiExoII.hpp>
#include <Mahi/Util/Logging/Csv.hpp>

using mahi::util::INF;

std::vector<MuscleInfo> get_muscle_info(std::string import_filepath);

void set_tight_pds(std::shared_ptr<meii::MahiExoII> meii);
void set_normal_pds(std::shared_ptr<meii::MahiExoII> meii);

void damp_exo(std::shared_ptr<meii::MahiExoII> meii);

// trajectories MUST be input with 0-100 on the first column
mahi::robo::Trajectory get_trajectory(std::string traj_filepath, int n_rows, int n_cols, 
                                      mahi::util::Time traj_time = mahi::util::seconds(20),
                                      std::vector<std::vector<double>> clamps_rad = {{-INF,INF},{-INF,INF},{-INF,INF},{-INF,INF}}, 
                                      bool convertdeg2rad = false, int dof = -1, std::vector<double> = {0,0,0,0});