#pragma once

#include <nlohmann/json.hpp>

class RCModel
{
public:
    /// constructor
    RCModel(std::string json_filename);
    ~RCModel();

    /// forward_prediction return the activation given a pule_width
    double forward_predict(unsigned int pulse_width);

    /// inverse prediction returning the required PW given a desired activation
    unsigned int inverse_predict(double desired_activation);

    unsigned int get_amplitude(){return static_cast<unsigned int>(m_amp);};
    
    std::string name; // name of the model detailing muscle

private:
    std::vector<std::vector<double>> m_train_inputs; // training inputs for prediction
    double m_a;                  // value of "a" parameter for prediction
    double m_b;                  // value of "b" parameter for prediction
    double m_c;                  // value of "c" parameter for prediction
    double m_amp;                // value of amplitude to use for muscle
    double m_pw_min;             // min value of pw to use for control
    double m_pw_max;             // max value of pw to use for control
    double m_torque_max;         // max value of torque used to scale for activation
    nlohmann::json m_js;         // json object to read from
    std::string m_json_filename; // filename of the json with parameters

    /// pulls relevant info from json file m_json_filename and populates member variables parameters 
    void populate_parameters();
};