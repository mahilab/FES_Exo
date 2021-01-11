#include <Mahi/Util.hpp>
#include <FESExo/Utility.hpp>
#include <FESExo/SharedController.hpp>

using namespace mahi::robo;
using namespace mahi::util;

int main(){
    std::string filepath = "C:/Git/FES_Exo/data/S9007";

    // MuscleData muscle_data(get_muscle_info(9004));
    // std::vector<bool> muscles_enabled = muscle_data.get_actives();
    // std::vector<unsigned int> stim_amplitudes = muscle_data.get_amplitudes();

    // SharedController sc(4, muscles_enabled, filepath, 0.5, 0.5);

    // std::vector<double> position = {-0.593732, 0.0265899, 0.0902366, -0.0388611};
    // std::vector<double> torque = {-0.386424, -0.171031, -0.182397, 0.175859};

    // auto activations = sc.calculate_activations(position,torque);
    // auto pws         = sc.calculate_pulsewidths(position,torque);

    // print_var(activations.activations);
    // print_var(pws.pulsewidth);

    auto test_model = RCModel(filepath + "/RC_Cal/Models/2_mdl.json");

    print_var(test_model.inverse_predict(0.0));
    print_var(test_model.inverse_predict(1.0));
    print_var(test_model.forward_predict(16));
    print_var(test_model.forward_predict(23));
}