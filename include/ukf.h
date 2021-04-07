#ifndef UKF
#define UKF

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include "data_parser.h"

struct state
{   
    state(){
        quats = Eigen::Quaterniond::Identity();
        omegas = Eigen::Vector3d::Zero();
    }
    Eigen::Quaterniond quats;
    Eigen::Vector3d omegas;
};


class ukf{

    public:

        ukf(int data_num,bool vicon_available);
        void run();
        
    private:

        data_parser Data;

        Eigen::Quaterniond g_quat;
        Eigen::Quaterniond avg_quat;
        Eigen::Matrix<double,6,6> Q;
        Eigen::Matrix<double,6,6> R;
        Eigen::Matrix<double,6,6> cov_prev_k;
        Eigen::Matrix<double,6,6> Pzz;
        Eigen::Matrix<double,6,6> Pvv;
        Eigen::Matrix<double,6,6> Pxz;
        Eigen::Matrix<double,6,6> K;
        Eigen::Matrix<double,6,1> gain;

        Eigen::Matrix<double,6,6> cholesky;
        Eigen::Matrix<double,1,12> cholesky_norms;
        
        state  prev_state;
        Eigen::Matrix<double,6,12> W;
        Eigen::Matrix<double,6,13> cov_w;
        Eigen::Matrix<double,6,13> prior_z;
        std::vector<state> sigma_points {std::vector<state>(13,state())};
        // std::vector<Eigen::Vector3d> error_omegas;
        Eigen::Matrix<double,3,13> error_omegas;
        Eigen::Matrix<double,6,1> mean_z;
        Eigen::Matrix<double,6,1> curr_data;
        state mean_state;

        Eigen::Quaterniond error_quat;
        Eigen::Vector3d error_omega_mean;
        Eigen::Quaterniond error_quats_mean;

        //general purpose utility variables
        Eigen::Quaterniond quat;
        Eigen::Vector3d vector;
        double norm{0};

        void calculate_sigma_points();
        void update_step(int index);
        void measurement_step();
        void get_distribution_charectoristics();
        void kalman_update(int index);
        void update_prediction(int index);

        void print_state(Eigen::Quaterniond q);
        void print_vector(Eigen::Vector3d v);

        void euler_angles(Eigen::Quaterniond, double& roll, double& pitch, double& yaw);

        bool debug{0};

};

#endif