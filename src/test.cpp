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

void do_nothing(std::vector<std::vector<double>> data){
    int x = 4;
}

int main(int argc, char* argv[]) {
    std::vector<std::vector<double>> data;

    std::vector<double> data_line;

    size_t line_length = 10;
    size_t num_lines = 2000*8;

    for (size_t i = 0; i < num_lines; i++){
        for (size_t i = 0; i < line_length; i++){
            data_line.push_back(random_range(0.0,10.0));
        }
        data.push_back(data_line);
        data_line.clear();
    }
    

    Clock timer;
    int count = 1000;
    for (size_t i = 0; i < count; i++){
        std::thread new_thread(do_nothing, data);
        new_thread.detach();
    }

    print_var((double)timer.get_elapsed_time().as_microseconds()/(double)count);

    return 0;
}