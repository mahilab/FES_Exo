#include <FESExo/FesGprModel.hpp>
#include <fstream>
#include <iostream>

template <typename T>
FesGprModel<T>::FesGprModel(std::string json_filename):
    m_json_filename(json_filename),
    m_js()
    {
        populate_parameters();
        // std::cout << "here" << std::endl;
        // std::cout << name;
        // std::cout << m_theta.size();
        // std::cout << m_theta.size();
        // std::cout << m_theta.size();
         
        
        m_kernel = T(m_train_inputs, m_theta);
        // std::cout << "post kernel";
    }

template <typename T>
FesGprModel<T>::~FesGprModel(){
}

template <typename T>
void FesGprModel<T>::populate_parameters(){
    std::ifstream file(m_json_filename);
    file >> m_js;
    file.close();
    m_beta  = m_js["beta"].get<std::vector<double>>();
    m_theta = m_js["theta"].get<std::vector<double>>();
    m_alpha = m_js["alpha"].get<std::vector<double>>();
    m_train_inputs = m_js["train_inputs"].get<std::vector<std::vector<double>>>();
    name = m_js["name"].get<std::string>();  
}

template<typename T>
double FesGprModel<T>::predict(const std::vector<double>& predict_point){
    std::vector<double> modified_prediction(predict_point.size()+1,1);
    for (auto i = 0; i < predict_point.size(); i++){
        modified_prediction[i+1] = predict_point[i];
    }
    
    double dot_prod = 0;
    std::vector<double> K = m_kernel(predict_point);
    for (size_t i = 0; i < m_alpha.size(); i++)
    {
        dot_prod += K[i]*m_alpha[i];
    }

    for (size_t i = 0; i < m_beta.size(); i++)
    {
        dot_prod += modified_prediction[i]*m_beta[i];
    }
    
    return dot_prod;
}

template class FesGprModel<ardsqexpKernel>;