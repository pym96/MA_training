#include <eigen3/Eigen/Dense>
#include<iostream>


int main(){

    Eigen::Matrix<double, 6, 1> X;
    
    X << 0, 0, 0, 0, 0, 0;

    std::cout << X;


    return 0;
}