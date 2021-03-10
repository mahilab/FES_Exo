#include <Mahi/Util.hpp>
#include <FESExo/Utility.hpp>
#include <FESExo/SharedController.hpp>

using namespace mahi::robo;
using namespace mahi::util;

int main(){
    int subject_num = 9010;
    std::string filepath = "C:/Git/FES_Exo/data/S9010";
    MuscleData muscle_data(get_muscle_info(subject_num));
    std::string model_filepath = "C:/Git/FES_Exo/data/S" + std::to_string(subject_num);  
    auto num_joints = 4;


    std::vector<bool> muscles_enabled = muscle_data.get_actives();
    std::vector<unsigned int> stim_amplitudes = muscle_data.get_amplitudes();

    SharedController sc(num_joints, muscles_enabled, model_filepath, 0.0, 0.0);
    

    // std::vector<double> positions   = {-0.184262,-0.32162,-0.231542,-0.202164};
    // std::vector<double> positions_d = {-0.192116,-0.313603,-0.242093,-0.166599};

    // std::vector<double> velocities  = {-0.25, 0.0, 0.05, 0.061};

    // double Kp_scale = 1.0;
    // double Kd_scale = 0.25;
    // std::vector<double> Kps = {100.0, 100.0, 28.0, 15.0};
    // std::vector<double> Kds = {1.25, 0.2, 0.01, 0.01};


    // std::vector<double> torques(4,0);
    // for (auto i = 0; i < 4; i++){
    //     auto controller = PdController(Kps[i]*Kp_scale,Kds[i]*Kd_scale);
    //     torques[i] = controller.calculate(positions_d[i],positions[i],0.0,velocities[i]);
    // }
    // print_var(torques);
    // torques = {0.275719,0.224492,-0.160185,0.530847};
    // torques = {1.1236,-0.26266,-0.187465,0.321077};

    std::vector<double> test_vec = {-1.0,-0.219219219219229,-0.21721721721722698,1.0};
    // linspace(-1.0,1.0,test_vec);
    // test_vec.push_back(DBL_MAX);
    // test_vec.push_back(DBL_MIN);

    for (size_t i = 0; i < test_vec.size(); i++){
        print("pw: {}, activation: {}", sc.rc_models[0].inverse_predict(test_vec[i]),test_vec[i]);
    }
    
    // auto result = sc.calculate_pulsewidths(positions,torques);
    // print_var(result.fes_torque);
    // print_var(result.pulsewidth);

    float x = 2;
    std::cout << "hello";

    std::cout << "hi";
}