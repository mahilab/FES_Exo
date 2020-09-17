#pragma once

#include<FESExo/FesGprModel.hpp>

class SharedController
{
private:
    bool verify_share_amts();
    bool build_model();
    bool m_valid = false;
    double m_fes_share_amt = 0;
    double m_exo_share_amt = 0;
    std::string m_model_filepath;

public:
    SharedController(std::string model_filepath, double fes_share_amt = 0.5, double exo_share_amt = 0.5);
    ~SharedController();
    std::vector<std::vector<FesGprModel<ardsqexpKernel>>> models;
};

struct SharedTorques
{
    double exo_torque = 0;
    double fes_torque = 0;
};