#pragma once

#include <math.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Mahi/Util/Timing/Clock.hpp>
        
struct ardsqexpKernel{
    ardsqexpKernel():
    length_scales(1,1),
    length_scales_2(1,1),
    train(1,std::vector<double>(1,0)),
    sigma_f(1.0),
    sigma_f_2(1.0),
    K(train.size(),0)
    {
    }

    ardsqexpKernel(std::vector<std::vector<double>> train, std::vector<double> theta):
    train(train),
    sigma_f(theta[theta.size()-1]),
    sigma_f_2(sigma_f*sigma_f),
    K(train.size(),0)
    {
        length_scales = std::vector<double>(theta.begin(),theta.end()-1);
        length_scales_2 = std::vector<double>(length_scales.size(),0);
        for (size_t i = 0; i < length_scales.size(); i++){
            length_scales_2[i] = length_scales[i]*length_scales[i];
        }   
    }

    std::vector<double> operator()(const std::vector<double>& New){       
        for (auto i = 0; i < train.size(); i++){
            double ard_sum = 0;
            for(auto j = 0; j < train[0].size(); j++) {
                double diff = train[i][j]-New[j];
                ard_sum += diff*diff/(length_scales_2[j]);
            }
            K[i] = sigma_f_2*exp(-0.5*ard_sum);
        }
        return K;
    }

    std::vector<std::vector<double>> train;
    std::vector<double> length_scales;
    std::vector<double> length_scales_2;
    double sigma_f;
    double sigma_f_2;
    std::vector<double> K;
};
