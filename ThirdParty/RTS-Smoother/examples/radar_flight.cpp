#include "rts-smoother.hpp"
#include <ADEKF/ADEKF.h>
#include <Eigen/Core>
#include <ADEKF/types/SO3.h>
#include <ADEKF/ManifoldCreator.h>


ADEKF_MANIFOLD(Test,((adekf::SO3,orient)),(3, position))
using namespace adekf;
int main(int argc, char *argv[]) {
    
    RTS_Smoother smoother{ADEKF{Test<double>{},Eigen::Matrix<double,6,6>::Identity()}};
    std::cout << smoother.sigma<< std::endl;
    smoother.storeEstimation();
    std::cout << smoother.old_sigmas.back()<< std::endl;

    //smoother.smoothAll();
    

}