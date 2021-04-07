#ifndef DATA_PARSESR
#define DATA_PARSESR

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>

// #include "TCanvas.h"
// #include "TROOT.h"
// #include "TGraphErrors.h"
// #include "TF1.h"
// #include "TLegend.h"
// #include "TArrow.h"
// #include "TLatex.h"
// #include "matplotlibcpp.h"

const double PI  =3.141592653589793238463;

class data_parser{

    public:

        data_parser(int data_file,bool gt_available);

        Eigen::Matrix<double,1,Eigen::Dynamic> imu_ts_raw;
        Eigen::Matrix<double,6,Eigen::Dynamic> imu_vals_raw;

        Eigen::Matrix<double,1,Eigen::Dynamic> roll_pred;
        Eigen::Matrix<double,1,Eigen::Dynamic> pitch_pred;
        Eigen::Matrix<double,1,Eigen::Dynamic> yaw_pred;

    private:


        const std::string imu_path = "../data/imu/imuRaw";
        const std::string vicon_path = "../data/vicon/viconRaw";

        const bool gt_available_;

        const std::vector<double> scale_acc {(3300.0/(1023.0*34.2)),(3300.0/(1023.0*34.2)),(3300.0/(1023.0*34.2))};
        const std::vector<double> bias_acc {510,501,502};
        const std::vector<double> scale_omega {(PI/180)*(3300.0/(1023.0*390)) , (PI/180)*(3300.0/(1023.0*195)) , (PI/180)*(3300.0/(1023.0*200))};
        const std::vector<double> bias_omega {373.7,375.8,369.8};

        Eigen::Matrix<double,1,Eigen::Dynamic> vicon_ts_raw;
        Eigen::Matrix<double,9,Eigen::Dynamic> vicon_rots_raw;

        void preprocess_data();
        void print_data();


};

#endif