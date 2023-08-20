#include <iostream>
#include <Eigen/Dense>
#include <vector>

int main () {
    Eigen::Matrix<double, 3, 3> R;
    R << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    std::cout << "R(3,1): " << R(3,1) << std::endl;
    std::cout << "R(3,2): " << R(3,2) << std::endl;
    return 0;
}