#ifndef DATA_HANDLER
#define DATA_HANDLER

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>

const double PI  =3.141592653589793238463;

class Data_handler{

    public:

        Data_handler(int data_file);

        Eigen::Matrix<double,1,Eigen::Dynamic> imu_ts_raw;
        Eigen::Matrix<double,6,Eigen::Dynamic> imu_vals_raw;

        Eigen::Matrix<double,3,Eigen::Dynamic> predictions;

        int file_no{0};

        void euler_angles(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw);

    private:


        const std::string imu_path = "../data/imu/imuRaw";
        const std::string vicon_path = "../data/vicon/viconRaw";

        const std::vector<double> scale_acc {(3300.0/(1023.0*34.2)),(3300.0/(1023.0*34.2)),(3300.0/(1023.0*34.2))};
        const std::vector<double> bias_acc {510,501,502};
        const std::vector<double> scale_omega {(PI/180)*(3300.0/(1023.0*390)) , (PI/180)*(3300.0/(1023.0*195)) , (PI/180)*(3300.0/(1023.0*200))};
        const std::vector<double> bias_omega {373.7,375.8,369.8};

        Eigen::Matrix<double,1,Eigen::Dynamic> vicon_ts_raw;
        Eigen::Matrix<double,9,Eigen::Dynamic> vicon_rots_raw;

        void preprocess_data();
};

#endif