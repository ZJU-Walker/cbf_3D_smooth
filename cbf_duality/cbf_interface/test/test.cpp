#include <iostream>
#include <Eigen/Dense>
#include <vector>

int main () {

    std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> dists_(10, Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(9));
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 9; ++j) {
            dists_[i][j] = 1;
        }
    }
    return 0;
}