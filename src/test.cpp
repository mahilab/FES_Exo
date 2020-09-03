/* 
 *  This is the default license template.
 *  
 *  File: test.cpp
 *  Author: Nathan Dunkelberger
 *  Copyright (c) 2020 Nathan Dunkelberger
 *  
 *  To edit this license information: Press Ctrl+Shift+P and press 'Create new License Template...'.
 */

// #include <MEII/MEII.hpp>
#include <Mahi/Util.hpp>
#include <thread>

// using namespace meii;
using namespace mahi::util;

int main(int argc, char* argv[]) {
    std::vector<std::vector<int>> stim_order( 54 , std::vector<int> (4, 0));
    csv_read_rows("C:\\Git\\FES_Exo\\data\\CollectionPointsRandomized.csv",stim_order);

    for (auto stim: stim_order)
    {
        for (auto j: stim) {
            std::cout << j << ", ";
        }
        std::cout << std::endl;
    }
    

    return 0;
}