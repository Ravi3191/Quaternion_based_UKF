#ifndef UKF_H
#define UKF_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <Eigen/Dense>
#include "data_handler.h"

#define PROCESS_NOISE_COEFF     11
#define MEASUREMENT_NOISE_COEFF 15
#define INIT_COV_COEFF          50

/*
    Data Structure for storing the 7D state of the drone (Quaternion + 3D Vector) 
*/
struct State
{   
    State(){

        quats = Eigen::Quaterniond::Identity();
        omegas = Eigen::Vector3d::Zero();
        
    }

    Eigen::Quaterniond quats;
    Eigen::Vector3d omegas;
};


/*
    Implementation of Quaternion based Unscented Kalman Filter 
*/
class Ukf{

    public:

        Ukf(int data_num, bool gt_available);
        void run();
        
    private:
        
        //Instance of the Data handler
        Data_handler data_;

        Eigen::Quaterniond g_quat;                      //gravity vector quat
        Eigen::Quaterniond avg_quat;                    //stores the mean quaternion of the sigma points

        Eigen::Matrix<double,6,6> Q;                    // process noise
        Eigen::Matrix<double,6,6> R;                    //measurement noise
        Eigen::Matrix<double,6,6> cov_prev_k;           // state covariance
        Eigen::Matrix<double,6,6> Pzz;                  // posterior state covariance
        Eigen::Matrix<double,6,6> Pvv;                  // expected covariance of posterior
        Eigen::Matrix<double,6,6> Pxz;                  // cross corelation matrix
        Eigen::Matrix<double,6,6> K;                    //Kalman gain matrix
        
        Eigen::Matrix<double,6,6> cholesky;             // cholesky decomposition
        Eigen::Matrix<double,1,12> cholesky_norms;      //norms of the decomposed vectors
        
        
        Eigen::Matrix<double,6,12> W;                   //priori 6D state vectores
        Eigen::Matrix<double,6,13> cov_w;               //error matrix
        Eigen::Matrix<double,6,13> posteriori_cov_z;    //posterirori error matrix
        Eigen::Matrix<double,3,13> error_omegas;        //angular velocity errors

        Eigen::Matrix<double,6,1> mean_z;               //mean of the posterirori
        Eigen::Matrix<double,6,1> gain;                 //measurement gain
        Eigen::Matrix<double,6,1> curr_data;            //curr observation
        
        State  prev_state;                              //state at last time step
        State mean_state;                               //avg state of sigma points

        std::vector<State> sigma_points {std::vector<State>(13,State())}; //ukf sigma points

        Eigen::Quaterniond error_quat;                   //quaternion error for mean update
        Eigen::Vector3d error_omega_mean;                //rotation error for mean update
        Eigen::Quaterniond error_quats_mean;             // mean error of sigma points

        //general purpose utility variables
        Eigen::Quaterniond quat;
        Eigen::Vector3d vector;
        double norm{0};

        //update functions
        void calculate_sigma_points();
        void update_step(int index);
        void measurement_step();
        void get_distribution_charectoristics();
        void kalman_update(int index);
        void update_prediction(int index);
};

#endif