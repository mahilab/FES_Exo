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

MatrixXd SharedController::predictModels(std::vector<double> positions){
    MatrixXd M = MatrixXd::Zero(num_joints,num_muscles);

    for (auto i = 0; i < num_joints; i++){
        for (auto j = 0; j < num_muscles; j++){
            M(i,j) = models[j][i].predict(positions);
        }
    }
    return M;
}

fesActivation SharedController::calculateActivations(std::vector<double> positions, std::vector<double> torque_desired_vec, std::vector<double> alpha_prev){
    MatrixXd M = predictModels(positions);

    VectorXd alpha;

    VectorXd alpha_baseline = VectorXd::Zero(num_muscles);
    for (auto i = 0; i < num_muscles; i++){
        alpha_baseline(i) = 0.5;
    }
    // Use desired activation if previous activations were too high or low
    // This should help stop the solution from running away
    if (max_element(alpha_prev)>1.2||min_element(alpha_prev)<-0.2){
        alpha = alpha_baseline;
    }
    // Use previous time step. Faster solution and smoother
    else{
        alpha = Eigen::Map<Eigen::VectorXd>(&alpha_prev[0], alpha_prev.size());
    }

    auto torque_desired = Eigen::Map<Eigen::VectorXd>(&torque_desired_vec[0], torque_desired_vec.size());

    VectorXd alpha_last;
    VectorXd g_last;

    Eigen::MatrixXd Bk = Eigen::MatrixXd::Identity(num_muscles,num_muscles);
    
    auto penalty_result = penaltyFunc(alpha,M,torque_desired);

    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(num_muscles,num_muscles);

    size_t it=0;
    while(penalty_result.gradient.norm()>1e-3 && it<120){
        auto p         = -Bk*penalty_result.gradient;
        auto alpha2    = armijo(alpha, penalty_result.loss, penalty_result.gradient, p, M, torque_desired);
        alpha_last     = alpha;
        alpha          = alpha + alpha2 * p;
        auto sk        = alpha - alpha_last;
        g_last         = penalty_result.gradient;
        penalty_result = penaltyFunc(alpha,M,torque_desired);
        auto yk        = penalty_result.gradient - g_last;
        double rho     = 1 / (yk.transpose()*sk);
        // https://en.wikipedia.org/wiki/Broyden%E2%80%93Fletcher%E2%80%93Goldfarb%E2%80%93Shanno_algorithm
        Bk = (identity - (rho * (sk * yk.transpose()).array()).matrix()) * Bk * (identity - (rho * (yk * sk.transpose()).array()).matrix()) + rho * sk * sk.transpose();
        it++;
    }

    if (max_element(alpha)>1) alpha=(alpha.array()/max_element(alpha)).matrix();

    std::vector<double> alpha_stdvec(&alpha[0], alpha.data()+alpha.cols()*alpha.rows());
    
    VectorXd torque_outputs = M*alpha;
    std::vector<double> torque_outputs_stdvec(&torque_outputs[0], torque_outputs.data()+torque_outputs.cols()*torque_outputs.rows());

    return fesActivation {alpha_stdvec, torque_outputs_stdvec};
}

penaltyResult SharedController::penaltyFunc(VectorXd alpha, MatrixXd M, VectorXd torque_desired){
    VectorXd gradient = VectorXd::Zero(num_muscles);
    
    double c  = 1.0;
    double c2 = 5000.0;
    
    double K  = 0.0;
    for(auto i = 0; i < alpha.size(); i++){
        if( alpha(i) < 0) {
            K += alpha(i)*alpha(i);
        }
    }

    double penalty = c*alpha.squaredNorm()+(M*alpha-torque_desired).squaredNorm()+c2*K;

    // Define gradient
    VectorXd K_vec = VectorXd::Zero(num_muscles);
    for (size_t i = 0; i < num_muscles; i++){
        if (alpha(i) < 0){
            K_vec(i) = 2 * alpha(i);
        }
    }
    
    gradient = c*2*alpha + 2*M.transpose()*(M*alpha - torque_desired) + c2*K_vec;
    
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
