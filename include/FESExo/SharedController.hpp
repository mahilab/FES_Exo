#pragma once

#include<FESExo/FesGprModel.hpp>
#include<FESExo/RCModel.hpp>
#include<Eigen/Dense>

// custom struct to help manage the penalty and gradient from the loss function
struct penaltyResult{
    double loss;
    Eigen::VectorXd gradient;
};

// custom struct to get the split torque from FES and from the Exo
struct sharedTorques{
    std::vector<double> exo_torque;
    std::vector<double> fes_torque;
    std::vector<unsigned int> pulsewidth;
};

// custom struct to get the result of activation count
struct fesActivation{
    std::vector<double> activations;
    std::vector<double> torques;
};

// custom struct to get the result of activation count
struct fesPulseWidth{
    std::vector<unsigned int> pulseWidth;
    std::vector<double> torques;
};

class SharedController
{
    
public:
    size_t num_muscles;
    size_t num_joints;
    SharedController(size_t num_muscles, size_t num_joints, std::vector<bool> muscles_enable, std::string model_filepath, double fes_share_amt = 0.5, double exo_share_amt = 0.5);
    ~SharedController();
    std::vector<std::vector<FesGprModel<ardsqexpKernel>>> gpr_models;
    std::vector<RCModel> rc_models;
    fesActivation calculate_activations(std::vector<double> positions, std::vector<double> torque_desired);
    fesPulseWidth calculate_pulsewidths(std::vector<double> positions, std::vector<double> torque_desired);
    Eigen::MatrixXd predict_models(std::vector<double> positions);

    sharedTorques share_torque(std::vector<double> unshared_torques, std::vector<double> current_position);

    std::vector<unsigned int> get_amplitudes(){return amplitudes;};

private:
    /// function that ensures that the share amounts given are valid
    bool verify_share_amts();
    /// executes build_gpr_models and build_rc_models
    bool build_both_models();
    /// builds the gpr models based on the folder input for the class
    bool build_gpr_models();
    /// builds the rc models based on the folder input for the class
    bool build_rc_models();
    /// penalty function to use when choosing activations based on desired torques and gpr models
    penaltyResult penaltyFunc(Eigen::VectorXd alpha, Eigen::MatrixXd M, Eigen::VectorXd 
    torque_desired);
    /// constraint function to make sure that our model decreases
    double armijo(Eigen::VectorXd alphak, double lossk, Eigen::VectorXd gradk, Eigen::VectorXd pk, Eigen::MatrixXd M, Eigen::VectorXd torque_desired);
    
    std::vector<bool> m_muscle_enable;
    bool m_valid = false;
    double m_fes_share_amt = 0;
    double m_exo_share_amt = 0;
    std::string m_model_filepath;

    std::vector<double> m_alpha_prev;
    Eigen::MatrixXd m_M;

    std::vector<unsigned int> amplitudes;
};




