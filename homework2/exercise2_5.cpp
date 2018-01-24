#include <iostream>
#include <cmath>
using namespace std;
#include <ctime>

#include <Eigen/Core>
#include <Eigen/Geometry>

int main (int argc, char** argv)
{
    int mx_size = 100;
    Eigen::MatrixXd dynamic = Eigen::MatrixXd::Random(mx_size, mx_size);
    Eigen::MatrixXd b = Eigen::MatrixXd::Random(mx_size, 1);

    clock_t time_start = clock();
    Eigen::MatrixXd x = dynamic.inverse()*b;
    cout <<"time use in normal inverse is " << 1000* (clock() - time_start)/(double)CLOCKS_PER_SEC << "ms"<< endl;

    time_start = clock();
    x = dynamic.colPivHouseholderQr().solve(b);
    cout <<"time use in Qr decomposition is " << 1000* (clock() - time_start)/(double)CLOCKS_PER_SEC << "ms"<< endl;

}
