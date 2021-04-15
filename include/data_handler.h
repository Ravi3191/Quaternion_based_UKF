#ifndef DATA_HANDLER_H
#define DATA_HANDLER_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <Eigen/Dense>

#define PI 3.141592653589793238463

class Data_handler{

    public:

        Data_handler(int data_file, bool gt_availbale);            //constructor

        const int file_no;                                         //data file value

        void euler_angles(const Eigen::Quaterniond q, double& roll, double& pitch, double& yaw);
        void store_predicionts();
        void post_process_data();
        
        //getter and setter functions
        int get_size(){ return imu_ts_raw.cols(); }              
        double get_prediction(const int rows, const int cols) {return predictions(rows,cols);}
        double get_time(const int index) {  return imu_ts_raw(0,index); }
        Eigen::Matrix<double,6,1> get_imu_col(const int index){ return imu_vals_raw.col(index); }
        void set_predictions(const int index,const double& roll,const double& pitch,const double& yaw);


    private:

        const bool gt_available;                                    // vicon data availability
        const std::string imu_path = "../data/imu/imuRaw";          // imu data path
        const std::string vicon_path = "../data/vicon/viconRaw";    // vicon data path
        const std::string save_path = "../data/predictions/predictions";  // results save path
        const std::string gt_save_path = "../data/predictions/gt";  // gt rpy save path

        const std::vector<double> scale_acc {(3300.0/(1023.0*34.2)),
                        (3300.0/(1023.0*34.2)),
                            (3300.0/(1023.0*34.2))};                //precomputed scale factor for accelataion
        const std::vector<double> bias_acc {510,501,502};           //precomputed bias for accelataion

        const std::vector<double> scale_omega {(PI/180)*(3300.0/(1023.0*390)),
                        (PI/180)*(3300.0/(1023.0*195)),          
                            (PI/180)*(3300.0/(1023.0*200))};        //precomputed scale factor for gyro
        const std::vector<double> bias_omega {373.7,375.8,369.8};   //precomputed bias for gyro

        Eigen::Matrix<double,1,Eigen::Dynamic> imu_ts_raw;         //imu time stamps
        Eigen::Matrix<double,6,Eigen::Dynamic> imu_vals_raw;       //imu values

        Eigen::Matrix<double,1,Eigen::Dynamic> vicon_ts_raw;        //vicon time stamps
        Eigen::Matrix<double,9,Eigen::Dynamic> vicon_rots_raw;      //vicon data

        Eigen::Matrix<double,3,Eigen::Dynamic> predictions;        //matrix to store roll,pitch,yaw predictions
        Eigen::Matrix<double,3,Eigen::Dynamic> vicon_euler;        //matrix to store roll,pitch,yaw angles of vicon data                                   

        int nearest_index{0};                                      //vicon index corresponding to the nearest temporal match of imu data
        double roll_error{0};                                      // appx. roll error
        double pitch_error{0};                                     // appx. pitch error
        double yaw_error{0};                                       // appx. yaw error

        void pre_process_data();
        void euler_angles(const int index);
        void update_error(const int index);
        

        /*
            Returns the difference of angles adjusted for cyclic nature  
        */
        inline double min_diff(const double angle_pred, const double angle_gt){
                
                if(std::abs(angle_gt - angle_pred) < PI) return angle_gt - angle_pred;
                else return 2*PI - std::abs(angle_gt - angle_pred);

        }
};

#endif