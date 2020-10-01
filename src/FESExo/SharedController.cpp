#include <Mahi/Util.hpp>
#include <FESExo/SharedController.hpp>
#include <filesystem>

using namespace mahi::util;
using Eigen::MatrixXd;
using Eigen::VectorXd;

SharedController::SharedController(size_t num_muscles_, size_t num_joints_, std::string model_filepath, double fes_share_amt, double exo_share_amt):
    num_muscles(num_muscles_),
    num_joints(num_joints_),
    m_model_filepath(model_filepath),
    m_fes_share_amt(fes_share_amt),
    m_exo_share_amt(exo_share_amt),
    m_alpha_prev(num_muscles,0.5),
    m_M(MatrixXd::Zero(num_joints,num_muscles)),
    amplitudes(num_muscles,0)
    {
        m_valid = verify_share_amts();
        build_both_models();
    }

SharedController::~SharedController()
{
}

bool SharedController::build_both_models(){
    return (build_gpr_models() && build_rc_models());
}

bool SharedController::build_gpr_models(){

    std::vector<FesGprModel<ardsqexpKernel>> joint_vector;
    bool through_1 = false;
    int last_muscle = -1;

    // get the strings into a readable format and sort them by alphabetical (and numerical) order
    std::vector<std::string> file_strings;
    for (const auto & entry : std::filesystem::directory_iterator(m_model_filepath + "/GPR_Cal/Models/"))
        file_strings.push_back(entry.path().string());
    std::sort(file_strings.begin(),file_strings.end());
    
    for (const auto & file : file_strings){
        size_t last_pos = file.find_last_of("/");
        std::string filename = file.substr(last_pos+1);
        int muscle_num = std::stoi(filename.substr(1,1));
        // int joint_num = std::stoi(filename.substr(3,1));

        if (muscle_num != last_muscle){
            if (last_muscle != -1) gpr_models.push_back(joint_vector);
            joint_vector.clear();
            last_muscle = muscle_num;
        }
        joint_vector.push_back(FesGprModel<ardsqexpKernel>(file));
    }
    gpr_models.push_back(joint_vector);
    
    return !gpr_models.empty();
}

bool SharedController::build_rc_models(){
    // get the strings into a readable format and sort them by alphabetical (and numerical) order
    std::vector<std::string> file_strings;
    for (const auto & entry : std::filesystem::directory_iterator(m_model_filepath +  "/RC_Cal/Models/"))
        file_strings.push_back(entry.path().string());
    std::sort(file_strings.begin(),file_strings.end());

    for (const auto & file : file_strings){        
        rc_models.push_back(RCModel(file));
        amplitudes.push_back(rc_models.back().get_amplitude());
    }
    return !rc_models.empty();
}

bool SharedController::verify_share_amts(){
    if (m_fes_share_amt < 0.0 || m_exo_share_amt < 0.0){
        LOG(Error) << "Share amount cannot have a value less than 0.0";
        return false;
    }
    return true;
}

MatrixXd SharedController::predict_models(std::vector<double> positions){
    MatrixXd M = MatrixXd::Zero(num_joints,num_muscles);

    for (auto i = 0; i < num_joints; i++){
        for (auto j = 0; j < num_muscles; j++){
            M(i,j) = gpr_models[j][i].predict(positions);
        }
    }
    return M;
}

fesActivation SharedController::calculate_activations(std::vector<double> positions, std::vector<double> torque_desired_vec){
    // MatrixXd M = MatrixXd::Zero(num_joints,num_muscles);
    m_M = predict_models(positions);

    VectorXd alpha;

    VectorXd alpha_baseline = VectorXd::Zero(num_muscles);
    for (auto i = 0; i < num_muscles; i++){
        alpha_baseline(i) = 0.5;
    }
    // Use desired activation if previous activations were too high or low
    // This should help stop the solution from running away
    if (max_element(m_alpha_prev)>1.2||min_element(m_alpha_prev)<-0.2){
        alpha = alpha_baseline;
    }
    // Use previous time step. Faster solution and smoother
    else{
        alpha = Eigen::Map<Eigen::VectorXd>(&m_alpha_prev[0], m_alpha_prev.size());
    }

    auto torque_desired = Eigen::Map<Eigen::VectorXd>(&torque_desired_vec[0], torque_desired_vec.size());

    VectorXd alpha_last;
    VectorXd g_last;

    Eigen::MatrixXd Bk = Eigen::MatrixXd::Identity(num_muscles,num_muscles);
    
    auto penalty_result = penaltyFunc(alpha, m_M, torque_desired);
    // std::cout << "loss: " << penalty_result.loss << std::endl;
    // std::cout << "gradient: " << penalty_result.gradient.transpose() << std::endl;
    // std::cin.get();
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(num_muscles,num_muscles);

    size_t it=0;
    while(penalty_result.gradient.norm()>1e-3 && it<120){
        auto p         = -Bk*penalty_result.gradient;
        auto alpha2    = armijo(alpha, penalty_result.loss, penalty_result.gradient, p, m_M, torque_desired);
        alpha_last     = alpha;
        alpha          = alpha + alpha2 * p;
        auto sk        = alpha - alpha_last;
        g_last         = penalty_result.gradient;
        penalty_result = penaltyFunc(alpha, m_M, torque_desired);
        auto yk        = penalty_result.gradient - g_last;
        double rho     = 1 / (yk.transpose()*sk);
        // https://en.wikipedia.org/wiki/Broyden%E2%80%93Fletcher%E2%80%93Goldfarb%E2%80%93Shanno_algorithm
        Bk = (identity - (rho * (sk * yk.transpose()).array()).matrix()) * Bk * (identity - (rho * (yk * sk.transpose()).array()).matrix()) + rho * sk * sk.transpose();
        it++;
        // std::cout << "loss: " << penalty_result.loss << std::endl;
        // std::cout << "gradient: " << penalty_result.gradient.transpose() << std::endl;
        // std::cin.get();
    }

    if (max_element(alpha)>1) alpha=(alpha.array()/max_element(alpha)).matrix();

    std::vector<double> alpha_stdvec(&alpha[0], alpha.data()+alpha.cols()*alpha.rows());

    m_alpha_prev = alpha_stdvec;
    
    VectorXd torque_outputs = m_M*alpha;
    std::vector<double> torque_outputs_stdvec(&torque_outputs[0], torque_outputs.data()+torque_outputs.cols()*torque_outputs.rows());
    // print("after {} iterations...", it);
    return fesActivation {alpha_stdvec, torque_outputs_stdvec};
}

penaltyResult SharedController::penaltyFunc(VectorXd alpha, MatrixXd M, VectorXd torque_desired){
    VectorXd gradient = VectorXd::Zero(num_muscles);
    
    double c  = 1.0;
    double c2 = 500.0;
    double c3 = 5000.0;
    
    double K  = 0.0;
    for(auto i = 0; i < alpha.size(); i++){
        if( alpha(i) < 0) {
            K += alpha(i)*alpha(i);
        }
    }
    
    double penalty = c*alpha.squaredNorm()+c2*(M*alpha-torque_desired).squaredNorm()+c3*K;

    // Define gradient
    VectorXd K_vec = VectorXd::Zero(num_muscles);
    for (size_t i = 0; i < num_muscles; i++){
        if (alpha(i) < 0){
            K_vec(i) = 2 * alpha(i);
        }
    }
    
    gradient = c*2*alpha + c2*2*M.transpose()*(M*alpha - torque_desired) + c3*K_vec;
    
    return penaltyResult {penalty, gradient};
}

double SharedController::armijo(VectorXd alphak, double lossk, VectorXd gradk, VectorXd pk, MatrixXd M, VectorXd torque_desired){
    double alpha1  = 1;
    double c       = 0;
    double rho     = 0.5;

    size_t n = 0;
    // while sufficient descent condition is not met
    while (lossk + c*alpha1*gradk.transpose()*pk < penaltyFunc(alphak+alpha1*pk,M,torque_desired).loss){
        alpha1 = rho*alpha1;
        n=n+1;
    }
    return alpha1;
}

fesPulseWidth SharedController::calculate_pulsewidths(std::vector<double> positions, std::vector<double> torque_desired_vec){
    fesActivation fes_activations = calculate_activations(positions, torque_desired_vec);

    std::vector<unsigned int> pulse_widths_out(num_muscles,0);
    std::vector<double> activations_out(num_muscles,0);

    for (auto i = 0; i < num_muscles; i++){
        pulse_widths_out[i] = rc_models[i].inverse_predict(fes_activations.activations[i]);
        activations_out[i]  = rc_models[i].forward_predict(pulse_widths_out[i]);
    }

    auto activations_out_eig = Eigen::Map<Eigen::VectorXd>(&activations_out[0], activations_out.size());
    Eigen::VectorXd torque_out = m_M*activations_out_eig;

    std::vector<double> torque_out_stdvec(&torque_out[0], torque_out.data()+torque_out.cols()*torque_out.rows());

    return fesPulseWidth {pulse_widths_out, torque_out_stdvec};
}

sharedTorques SharedController::share_torque(std::vector<double> unshared_torques, std::vector<double> current_position){
    sharedTorques shared_torques{std::vector<double>(num_joints,0.0), std::vector<double>(num_joints,0.0), std::vector<unsigned int>(num_muscles,0)};

    // return zeros if torque size is wrong
    if(unshared_torques.size() != num_joints){
        LOG(Error) << "Incorrect number of torques input";
        return shared_torques;
    } 
    // return zeros if torque size is wrong
    if(current_position.size() != num_joints){
        LOG(Error) << "Incorrect number of position inputs";
        return shared_torques;
    }

    std::vector<double> fes_torque_desired(num_joints, 0.0);
    for (auto i = 0; i < num_joints; i++){
        fes_torque_desired[i] = unshared_torques[i]*m_fes_share_amt;
    }

    fesPulseWidth pulse_width = calculate_pulsewidths(current_position, fes_torque_desired);

    shared_torques.fes_torque = pulse_width.torques;
    shared_torques.pulsewidth = pulse_width.pulseWidth;

    for (auto i = 0; i < num_joints; i++){
        shared_torques.exo_torque[i] = unshared_torques[i] - shared_torques.fes_torque[i];
    }
    
    return shared_torques;
}