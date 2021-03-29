#include <Mahi/FesExo/RCModel.hpp>
#include <fstream>
#include <math.h>
#include <Mahi/Util/Math/Functions.hpp>
#include <Mahi/Util/Print.hpp>

RCModel::RCModel(std::string json_filename):
    m_json_filename(json_filename),
    m_js()
    {
        populate_parameters();
    }

RCModel::~RCModel(){
}

void RCModel::populate_parameters(){
    std::ifstream file(m_json_filename);
    file >> m_js;
    file.close();
    m_a = m_js["a"].get<double>();
    m_b = m_js["b"].get<double>();
    m_c = m_js["c"].get<double>();

    m_amp    = m_js["amp"].get<double>();
    m_pw_min = m_js["pw_low"].get<double>();
    m_pw_max = m_js["pw_high"].get<double>();

    m_torque_max = m_a/(1.0 + exp(-m_b*((m_pw_max-m_pw_min) - m_c))) - m_a/(1.0 + exp(m_b*m_c));
}

double RCModel::forward_predict(unsigned int pulse_width){
    double adjusted_input = static_cast<double>(pulse_width) - m_pw_min;
    double torque_output = m_a/(1.0 + exp(-m_b*(adjusted_input - m_c))) - m_a/(1.0 + exp(m_b*m_c));
    double activation = torque_output / m_torque_max;
    return mahi::util::clamp01(activation);
}

unsigned int RCModel::inverse_predict(double desired_activation){
    desired_activation = mahi::util::clamp(desired_activation,0.0,1.0);
    double torque_out = m_torque_max*desired_activation;
    double pw = log(-(1.0-(m_a+m_a*exp(m_b*m_c))/(torque_out+torque_out*exp(m_b*m_c)+m_a)))/-m_b+m_c;
    unsigned int pw_out = static_cast<unsigned int>(mahi::util::clamp(pw + m_pw_min,0.0,m_pw_max) + 0.5);
    // mahi::util::print("double: {}, clamped unsigned int: {}",pw + m_pw_min, pw_out);
    // if the desired (zeroed) pulsewidth is < 0, command 0 pw
    // if (pw < 0) return 0;   

    // otherwise, add the min pw and clamp it to the max allowable pw
    return (pw_out <= m_pw_min) ? 0 : pw_out;
}

