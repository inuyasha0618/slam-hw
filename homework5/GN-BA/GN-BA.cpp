//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

void setVecVector(VecVector2d& v, string filename) {
    ifstream iss(filename);
    string currLineStr;
    while (getline(iss, currLineStr)) {
        Vector2d currLineVec;
        istringstream iss(currLineStr);
        iss >> currLineVec(0);
        iss >> currLineVec(1);
        v.push_back(currLineVec);
    }
}

void setVecVector(VecVector3d& v, string filename) {
    ifstream iss(filename);
    string currLineStr;
    while (getline(iss, currLineStr)) {
        Vector3d currLineVec;
        istringstream iss(currLineStr);
        iss >> currLineVec(0);
        iss >> currLineVec(1);
        iss >> currLineVec(2);
        v.push_back(currLineVec);
    }
}

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    setVecVector(p2d, p2d_file);
    setVecVector(p3d, p3d_file);
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Vector3d t(0, 0, 0);
    Matrix3d R = Eigen::Matrix3d::Identity();
    Sophus::SE3 T_esti ; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
            Vector3d P = p3d.at(i);
            Vector2d u = p2d.at(i);
            Vector3d Pc = T_esti * P;
            Vector3d u_calcu_homo = K * Pc;

            Vector2d e;
            e(0) = u(0) - u_calcu_homo(0) / u_calcu_homo(2);
            e(1) = u(1) - u_calcu_homo(1) / u_calcu_homo(2);

            cost += e.transpose() * e;

            // END YOUR CODE HERE
            // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE
            Matrix<double, 2, 3> de_dPc;
            de_dPc << -fx / Pc(2), 0, fx * Pc(0) / (Pc(2) * Pc(2)),
                    0, -fy / Pc(2), fy * Pc(1) / (Pc(2) * Pc(2));

            Matrix<double , 3, 6> dPc_dpose;
            dPc_dpose << 1, 0, 0, 0, Pc(2), -Pc(1),
                         0, 1, 0, -Pc(2), 0, Pc(0),
                         0, 0, 1, Pc(1), -Pc(0), 0;

            J = de_dPc * dPc_dpose;
	        // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	    // solve dx
        Vector6d dx;

        // START YOUR CODE HERE
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (std::isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE
        T_esti = Sophus::SE3::exp(dx) * T_esti;
        // END YOUR CODE HERE

        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
