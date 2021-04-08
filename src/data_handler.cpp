#include "data_handler.h"

/*
    Extracts and pre-processes IMU and Vicon data from .txt files and stores them in Eigen Matrices
    
    @param Dy, data_file is the index of the dataset that we want to evaluate on. It starts from 1. 
    
*/

Data_handler::Data_handler(int data_file) : file_no(data_file){

    std::ifstream file;
    std::string str,token;
    size_t pos = 0;

    int imu_cols{0},vicon_cols{0},imu_rows{0},vicon_rows{0};

    //get number of cols for the imu data files to resize array
    file.open( imu_path + std::to_string(data_file) + "_ts.txt");
    while(std::getline(file,str)){
        while((pos = str.find(" ")) != std::string::npos){
            token = str.substr(0,pos);
            str.erase(0,pos+1);
            imu_cols++;
        }
        imu_cols++;
    }
    file.close();

    //resize the matrices based on the cols
    imu_ts_raw.resize(1,imu_cols);
    imu_vals_raw.resize(6,imu_cols);

    predictions.resize(3,imu_cols);

    //parse and  store the time stamps of imu
    file.open(imu_path + std::to_string(data_file) + "_ts.txt");
    while(std::getline(file,str)){
        imu_cols = 0;
        while((pos = str.find(" ")) != std::string::npos){
            imu_ts_raw(0,imu_cols) = stod(str.substr(0,pos));
            str.erase(0,pos+1);
            imu_cols++;
        }
        imu_ts_raw(0,imu_cols) = stod(str);
    }
    file.close();

    //parse and store imu data
    file.open(imu_path + std::to_string(data_file) + "_vals.txt");
    while(std::getline(file,str)){
        imu_cols = 0;
        while((pos = str.find(" ")) != std::string::npos){
            //the data is ordered in z,x,y order
            if(imu_rows == 3){
                imu_vals_raw(5,imu_cols) = stod(str.substr(0,pos));
            }else if(imu_rows == 4){
                imu_vals_raw(3,imu_cols) = stod(str.substr(0,pos));
            }else if(imu_rows == 5){
                imu_vals_raw(4,imu_cols) = stod(str.substr(0,pos));
            }else{
                imu_vals_raw(imu_rows,imu_cols) = stod(str.substr(0,pos));
            }
            
            str.erase(0,pos+1);
            imu_cols++;
        }

        if(imu_rows == 3){
            imu_vals_raw(5,imu_cols) = stod(str);
        }else if(imu_rows == 4){
            imu_vals_raw(3,imu_cols) = stod(str);
        }else if(imu_rows == 5){
            imu_vals_raw(4,imu_cols) = stod(str);
        }else{
            imu_vals_raw(imu_rows,imu_cols) = stod(str);
        }

        imu_rows++;
    }
    file.close();

    
    //get number of cols for the vicon data files to resize array
    file.open( vicon_path + std::to_string(data_file) + "_ts.txt");
    while(std::getline(file,str)){
        while((pos = str.find(" ")) != std::string::npos){
            token = str.substr(0,pos);
            str.erase(0,pos+1);
            vicon_cols++;
        }
        vicon_cols++;
    }
    file.close();

    vicon_ts_raw.resize(1,vicon_cols);
    vicon_rots_raw.resize(9,vicon_cols);



    //parse and store vicon time stamps
    file.open( vicon_path + std::to_string(data_file) + "_ts.txt");
    while(std::getline(file,str)){
        vicon_cols = 0;
        while((pos = str.find(" ")) != std::string::npos){
            vicon_ts_raw(0,vicon_cols) = stod(str.substr(0,pos));
            str.erase(0,pos+1);
            vicon_cols++;
        }
        vicon_ts_raw(0,vicon_cols) = stod(str);
    }
    file.close();

    //parse and store vicon data
    file.open( vicon_path + std::to_string(data_file) + "_rots.txt");
    while(std::getline(file,str)){
        vicon_cols = 0;
        while((pos = str.find(" ")) != std::string::npos){
            vicon_rots_raw(vicon_rows,vicon_cols) = stod(str.substr(0,pos));
            str.erase(0,pos+1);
            vicon_cols++;
        }
        vicon_rots_raw(vicon_rows,vicon_cols) = stod(str);
        vicon_rows++;
    }
    file.close();

    preprocess_data();

    std::cout << "Finished Extracting and Preprocessing data!" << "\n";
    std::cout << "Imu Data has "  << imu_vals_raw.cols()  << " data points"<<  "\n";
    std::cout << "Vicon Data has "  << vicon_rots_raw.cols()  << " data points"<<  "\n";

}

/*
    Pre-processes the IMU and Vicon data by scaling and shifting them appropriately
    
*/
void Data_handler::preprocess_data(){

    //accel data along x,y,z
    imu_vals_raw.array().row(0) = -(imu_vals_raw.array().row(0) - bias_acc[0])*scale_acc[0];
    imu_vals_raw.array().row(1) = -(imu_vals_raw.array().row(1) - bias_acc[1])*scale_acc[1];
    imu_vals_raw.array().row(2) =  (imu_vals_raw.array().row(2) - bias_acc[2])*scale_acc[2];

    //gyro data (rotation rates)
    imu_vals_raw.array().row(3) = (imu_vals_raw.array().row(3) - bias_omega[0])*scale_omega[0];
    imu_vals_raw.array().row(4) = (imu_vals_raw.array().row(4) - bias_omega[1])*scale_omega[1];
    imu_vals_raw.array().row(5) = (imu_vals_raw.array().row(5) - bias_omega[2])*scale_omega[2];

    //initialize the first predictions to the imu data
    predictions(0,0) = std::atan2(imu_vals_raw(1,0),imu_vals_raw(2,0));
    predictions(1,0) = std::atan2(-imu_vals_raw(0,0), std::sqrt(std::pow(imu_vals_raw(2,0),2) + std::pow(imu_vals_raw(1,0),2)));
    predictions(2,0) = 0; 

}

void Data_handler::euler_angles(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw){

    roll = std::atan2( 2*(q.w() * q.x() + q.y() * q.z()) ,
       1 - 2* (std::pow(q.x(),2)+ std::pow(q.y(),2)));

    pitch = std::asin(2*(q.w()*q.y() - q.x()*q.z()));

    yaw = std::atan2( 2*(q.w() * q.z() + q.x() * q.y()) ,
       1 - 2* (std::pow(q.y(),2)+ std::pow(q.z(),2)));

}