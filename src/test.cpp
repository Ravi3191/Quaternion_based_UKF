#include <iostream>
#include <Eigen/Dense>
#include <string>

int main(){
    
    Eigen::Quaterniond test_q = Eigen::Quaterniond::Identity();
    test_q.w() = 0.999985;
    test_q.x() = -4.40361e-08;
    test_q.y() = 0.0054506;
    test_q.z() = -9.45204e-07;

    auto euler = test_q.toRotationMatrix().eulerAngles(2,1,0);

    Eigen::Matrix3d mat;

    mat << 9.999263298991553084e-01, -4.539906266821729444e-03, -1.125717662178053226e-02,
    4.557965814180941176e-03, 9.999883655753890599e-01, 1.579133147901947126e-03,
    1.124987653453348758e-02, -1.630326639209955884e-03,9.999353890692174174e-01;
    
    auto rot_euler = mat.eulerAngles(2,1,0);

    std::cout << euler << "\n";
    std::cout << rot_euler << "\n";
    return 0;
}