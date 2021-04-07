#include <iostream>
#include <Eigen/Dense>
#include <vector>
// #include <math>

int main(){

    Eigen::Quaterniond q;
    // 0.999985,-4.40361e-08,0.0054506,-9.45204e-07

    q.w() = 0.999985;
    q.x() = -4.40361e-08;
    q.y() = 0.0054506;
    q.z() = -9.45204e-07;

    double roll,pitch,yaw;
    
    roll = std::atan2( 2*(q.w() * q.x() + q.y() * q.z()) ,
       1 - 2* (std::pow(q.x(),2)+ std::pow(q.y(),2)));

    pitch = std::asin(2*(q.w()*q.y() - q.x()*q.z()));

    yaw = std::atan2( 2*(q.w() * q.z() + q.x() * q.y()) ,
       1 - 2* (std::pow(q.y(),2)+ std::pow(q.z(),2)));

   for(int i = 0; i < 3; i++){
      for(int j = 0; j < 3; j++){
         for(int k = 0; k < 3; k++){

            std::cout << i << "," << j << "," << k << "\n";
            Eigen::Vector3d vector = q.toRotationMatrix().eulerAngles(i,j,k);
            std::cout << vector[0] << "," << vector[1] << "," << vector[2] << "\n";

         }
      }
   }
      
   std::cout << "printing gt values" << "\n";
    std::cout << roll << "\n";
    std::cout << pitch << "\n";
    std::cout << yaw << "\n";

    return 0;
}