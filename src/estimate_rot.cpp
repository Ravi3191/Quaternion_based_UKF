#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <math.h>


std::vector<Eigen::Quaterniond> axis_to_quat(std::vector<Eigen::Vector3d>& axes){

    std::vector<Eigen::Quaterniond> quats;
    int norm{1};
    int angle{0};

    for (Eigen::Vector3d vector :  axes){

        Eigen::Quaterniond quat;
        norm = vector.norm();

        vector.normalize();
        quat = Eigen::AngleAxisd(norm,vector); //be careful of norm/2
        quats.push_back(quat);
    }

    return quats;
}
