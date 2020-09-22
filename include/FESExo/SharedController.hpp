#pragma once

#include<FESExo/FesGprModel.hpp>
#include<Eigen/Dense>

// custom struct to help manage the penalty and gradient from the loss function
struct penaltyResult{
    double loss;
    Eigen::VectorXd gradient;
};

// custom struct to get the split torque from FES and from the Exo
struct SharedTorques{
    double exo_torque = 0;
    double fes_torque = 0;
};

// custom struct to get the result of activation count
struct fesActivation{
    std::vector<double> activations;
    std::vector<double> torques;
};

class SharedController
{
private:

    bool verify_share_amts();
    bool build_model();
    penaltyResult penaltyFunc(Eigen::VectorXd alpha, Eigen::MatrixXd M, Eigen::VectorXd 
    torque_desired);
    double armijo(Eigen::VectorXd alphak, double lossk, Eigen::VectorXd gradk, Eigen::VectorXd pk, Eigen::MatrixXd M, Eigen::VectorXd torque_desired);
    bool m_valid = false;
    double m_fes_share_amt = 0;
    double m_exo_share_amt = 0;
    std::string m_model_filepath;
    
public:
    size_t num_muscles;
    size_t num_joints;
    SharedController(size_t num_muscles, size_t num_joints, std::string model_filepath, double fes_share_amt = 0.5, double exo_share_amt = 0.5);
    ~SharedController();
    std::vector<std::vector<FesGprModel<ardsqexpKernel>>> models;
    fesActivation calculateActivations(std::vector<double> positions, std::vector<double> torque_desired, std::vector<double> alpha_prev);
    Eigen::MatrixXd predictModels(std::vector<double> positions);
};




