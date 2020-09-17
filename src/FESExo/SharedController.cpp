#include <Mahi/Util.hpp>
#include <FESExo/SharedController.hpp>
#include <filesystem>

using namespace mahi::util;

SharedController::SharedController(std::string model_filepath, double fes_share_amt, double exo_share_amt):
    m_model_filepath(model_filepath),
    m_fes_share_amt(fes_share_amt),
    m_exo_share_amt(exo_share_amt)
    {
        m_valid = verify_share_amts();
        build_model();
    }

SharedController::~SharedController()
{
}

bool SharedController::build_model(){

    std::vector<FesGprModel<ardsqexpKernel>> joint_vector;
    bool through_1 = false;
    int last_muscle = -1;
    
    for (const auto & entry : std::filesystem::directory_iterator(m_model_filepath)){
        size_t last_pos = entry.path().string().find_last_of("/");
        std::string filename = entry.path().string().substr(last_pos+1);
        int muscle_num = std::stoi(filename.substr(1,1));
        int joint_num = std::stoi(filename.substr(3,1));

        if (muscle_num != last_muscle){
            if (last_muscle != -1) models.push_back(joint_vector);
            joint_vector.clear();
            last_muscle = muscle_num;
        }
        joint_vector.push_back(FesGprModel<ardsqexpKernel>(entry.path().string()));
    }
    models.push_back(joint_vector);
    
    return true;
}

bool SharedController::verify_share_amts(){
    if (m_fes_share_amt < 0.0 || m_exo_share_amt < 0.0){
        LOG(Error) << "Share amount cannot have a value less than 0.0";
        return false;
    }
    if (m_fes_share_amt + m_exo_share_amt > 1.0){
        LOG(Error) << "Combined share amounts cannot have a value greater than 1.0";
        return false;
    }
    return true;
}
