#include <Mahi/Util.hpp>
#include <FESExo/Utility.hpp>
#include <FESExo/SharedController.hpp>

using namespace mahi::robo;
using namespace mahi::util;

int main(){
    std::vector<double> my_vec(100,0.0);

    double new_number = 20.0;

    linspace(0.0,100.0,my_vec);
    for (auto i = 0; i < my_vec.size(); i++){
        std::cout << interp(my_vec[i],0.0,100.0,0.0, new_number) << std::endl;
    }
    
}