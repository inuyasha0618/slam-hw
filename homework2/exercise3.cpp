#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

int main (int argc, char** argv)
{
    Eigen::Isometry3d Tc1w = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q1(0.35, 0.2, 0.3, 0.1);
    Eigen::Vector3d t1(0.3, 0.1, 0.1);
    q1.normalize();
    Tc1w.rotate(q1);
    Tc1w.pretranslate(t1);

    Eigen::Isometry3d Tc2w = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q2(-0.5, 0.4, -0.1, 0.2);
    Eigen::Vector3d t2(-0.1, 0.5, 0.3);
    q2.normalize();
    Tc2w.rotate(q2);
    Tc2w.pretranslate(t2);

    Eigen::Vector3d Xc1 (0.5, 0, 0.2);
    Eigen::Vector3d Xw;
    Xw = Tc1w.inverse() * Xc1;

    Eigen::Vector3d Xc2;
    Xc2 = Tc2w * Xw;

    cout << "In robot2\'s cordinate system: "<< endl <<Xc2 << endl;
}
