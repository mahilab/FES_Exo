#include <Mahi/Util.hpp>
#include <FESExo/Utility.hpp>

using namespace mahi::robo;
using namespace mahi::util;

int main(){
    std::string filepath = "C:/Git/FES_Exo/images/full_traj.csv";

    std::vector<std::vector<double>> min_max = {{-91.5 * DEG2RAD,  3.0 * DEG2RAD},
                                                {-80.0 * DEG2RAD, 80.0 * DEG2RAD},
                                                {-20.0 * DEG2RAD, 20.0 * DEG2RAD},
                                                {-20.0 * DEG2RAD, 20.0 * DEG2RAD}};

    mahi::robo::Trajectory my_traj = get_trajectory(filepath,201,5,min_max,true);
    for (size_t i = 0; i < my_traj.size(); i++){
        print_var(my_traj[i]);
    }
}