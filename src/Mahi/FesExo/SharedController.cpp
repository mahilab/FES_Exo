#include <Mahi/Util.hpp>
#include <Mahi/FesExo/SharedController.hpp>
#include <filesystem>

using namespace mahi::util;
using Eigen::MatrixXd;
using Eigen::VectorXd;

SharedController::SharedController(std::vector<bool> joints_enable, std::vector<bool> muscles_enable, std::string model_filepath, double fes_share_amt, double exo_share_amt):
    m_joints_enable(joints_enable),
    num_joints(std::count(joints_enable.begin(),joints_enable.end(),true)),    
    m_muscle_enable(muscles_enable),
    num_muscles(std::count(muscles_enable.begin(),muscles_enable.end(),true)),
    m_model_filepath(model_filepath),
    m_fes_share_amt(fes_share_amt),
    m_exo_share_amt(exo_share_amt),
    m_alpha_prev(num_muscles,0.5),
    m_M(MatrixXd::Zero(num_joints,num_muscles)),
    amplitudes(num_muscles,0)
    {
        print("number of muscles: {}", num_muscles);
        
        m_valid = verify_share_amts();
        build_both_models();
        print("gpr size: {}x{}, rc size: {}", gpr_models.size(), gpr_models[0].size(), rc_models.size());
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

    for (auto i = 0; i < m_muscle_enable.size(); i++){
        if (m_muscle_enable[i]){
            for (auto j = 0; j < m_joints_enable.size(); j++){
                    if (m_joints_enable[j]){
                        std::string filename = m_model_filepath + "/GPR_Cal/Models/m" + std::to_string(i+1) +               // muscle
                                                                                  "j" + std::to_string(j+1) + "model.json"; // joint
                                            joint_vector.push_back(filename);

                        print_var(filename);
                    }
            }
            
            gpr_models.push_back(joint_vector);
            joint_vector.clear();
        }
    }
    
    return !gpr_models.empty();
}

bool SharedController::build_rc_models(){
    for (auto i = 0; i < m_muscle_enable.size(); i++){
        if (m_muscle_enable[i]){
            std::string filepath = m_model_filepath +  "/RC_Cal/Models/" + std::to_string(i+1) + "_mdl.json";
            rc_models.emplace_back(filepath);
            amplitudes.push_back(rc_models.back().get_amplitude());
        }
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
    }

    if (max_element(alpha)>1) alpha=(alpha.array()/max_element(alpha)).matrix();

    std::vector<double> alpha_stdvec(&alpha[0], alpha.data()+alpha.cols()*alpha.rows());

    m_alpha_prev = alpha_stdvec;
    
    VectorXd torque_outputs = m_M*alpha;
    std::vector<double> torque_outputs_stdvec(&torque_outputs[0], torque_outputs.data()+torque_outputs.cols()*torque_outputs.rows());

    return fesActivation {alpha_stdvec, torque_outputs_stdvec};
}

penaltyResult SharedController::penaltyFunc(VectorXd alpha, MatrixXd M, VectorXd torque_desired){
    VectorXd gradient = VectorXd::Zero(num_muscles);
    
    double c  = 1.0; // 1 before 12/18/2020
    double c2 = 1500.0; // 500 before 12/18/2020
    double c3 = 5000.0; // 5000 before 12/18/2020

    // VectorXd dof_weights;
    // dof_weights << 0.25, 1.75, 1.0, 1.0;
    
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
    std::vector<double> remapped_torque_desired;
    for (size_t i = 0; i < m_joints_enable.size(); i++){
        remapped_torque_desired.push_back(torque_desired_vec[i]);
    }    
    fesActivation fes_activations = calculate_activations(positions, remapped_torque_desired);

    std::vector<unsigned int> pulse_widths_out(num_muscles,0);
    std::vector<double> activations_out(num_muscles,0);

    for (auto i = 0; i < num_muscles; i++){
        pulse_widths_out[i] = rc_models[i].inverse_predict(fes_activations.activations[i]);
        activations_out[i]  = rc_models[i].forward_predict(pulse_widths_out[i]);
    }

    auto activations_out_eig = Eigen::Map<Eigen::VectorXd>(&activations_out[0], activations_out.size());
    Eigen::VectorXd torque_out = m_M*activations_out_eig;

    std::vector<double> torque_out_stdvec(&torque_out[0], torque_out.data()+torque_out.cols()*torque_out.rows());

    // remap enabled pulsewidths to all pulsewidths
    std::vector<unsigned int> pulse_widths_out_remapped;
    int muscle_num = 0;
    for (const auto &enabled : m_muscle_enable){
        unsigned int remapped_pw = (enabled) ? pulse_widths_out[muscle_num++] : 0;
        pulse_widths_out_remapped.push_back(remapped_pw);
    }

    // remap enabled joints to all joints
    std::vector<double> joint_torques_out_remapped;
    int joint_num = 0;
    for (const auto &enabled : m_joints_enable){
        double remapped_torque = (enabled) ? torque_out_stdvec[joint_num++] : 0;
        joint_torques_out_remapped.push_back(remapped_torque);
    }

    return fesPulseWidth {pulse_widths_out_remapped, joint_torques_out_remapped};
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

    shared_torques.fes_torque = pulse_width.fes_torque;
    shared_torques.pulsewidth = pulse_width.pulsewidth;

    for (auto i = 0; i < num_joints; i++){
        shared_torques.exo_torque[i] = unshared_torques[i] - shared_torques.fes_torque[i];
    }
    
    return shared_torques;
}