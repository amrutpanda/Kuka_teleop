#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace std;
using namespace Eigen;

int main(int argc, char const *argv[])
{
    VectorXd v1, v2, v;
    v1 << 1,2,3;
    v2 << -1,-2,-3;

    v << 3,4,5;
    // v.unaryExpr([](int i){cout << i << "\n";});

    return 0;
}
