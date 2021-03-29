#pragma once

#include <nlohmann/json.hpp>
#include <Mahi/FesExo/Kernels.hpp>
#include <Eigen/Dense>

template <typename T>
class FesGprModel
{
public:
    /// constructor
    FesGprModel(std::string json_filename);
    ~FesGprModel();
    std::string name; // name of the model detailing muscle and joint
    /// prediction using the kernel of the template type. K * alpha + [1 x_new] * beta
    double predict(const std::vector<double>& predict_point);
private:
    T m_kernel;                     // kernel struct to use of type T (template) instantiated with m_theta and m_train_inputs
    std::vector<std::vector<double>> m_train_inputs; // training inputs for prediction
    std::vector<double> m_alpha;        // value of alpha for prediction
    std::vector<double> m_beta;         // value of beta for prediction
    std::vector<double> m_theta;        // value of theta for the kernel
    nlohmann::json m_js;            // json object to read from
    std::string m_json_filename;    // filename of the json with parameters
    

    /// pulls relevant info from json file m_json_filename and populates member variables parameters 
    void populate_parameters();
};