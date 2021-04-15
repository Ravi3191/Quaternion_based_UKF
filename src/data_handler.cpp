#include "data_handler.h"

/*
    Extracts and pre-processes IMU and Vicon data from .txt files and stores them in Eigen Matrices
    
    @param Dy, data_file is the index of the dataset that we want to evaluate on. It starts from 1. 
    
*/

Data_handler::Data_handler(int data_file) : file_no(data_file){

    //initialize data stream
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

    std::cout << "Imu Data has "  << imu_cols<< " data points"<<  "\n";

    //resize the matrices based on the cols
    imu_ts_raw.resize(1,imu_cols);
    imu_vals_raw.resize(6,imu_cols);
    predictions.resize(3,imu_cols);

    //parse and  store the time stamps of imu
    int cols{0};
    file.open(imu_path + std::to_string(data_file) + "_ts.txt");
    while(std::getline(file,str)){

        cols = 0;
        while((pos = str.find(" ")) != std::string::npos){

            imu_ts_raw(0,cols) = stod(str.substr(0,pos));
            str.erase(0,pos+1);
            cols++;

        }
        imu_ts_raw(0,cols) = stod(str);
    }
    file.close();

    //parse and store imu data
    cols = 0;
    file.open(imu_path + std::to_string(data_file) + "_vals.txt");
    while(std::getline(file,str)){

        if(imu_rows >= 6){
            std::cout << "Imu Data should only have 6 rows Ax,Ay,Az,Wz,Wx,Wy\n";
            file.close();
            exit(1);
        }

        cols = 0;
        while((pos = str.find(" ")) != std::string::npos){
            
            if(cols > imu_cols - 2){
                std::cout << "Imu Data points size != Time Stamp Size\n";
                file.close();
                exit(1);
            }

            //the data is ordered in z,x,y order
            if(imu_rows == 3){
                imu_vals_raw(5,cols) = stod(str.substr(0,pos));
            }else if(imu_rows == 4){
                imu_vals_raw(3,cols) = stod(str.substr(0,pos));
            }else if(imu_rows == 5){
                imu_vals_raw(4,cols) = stod(str.substr(0,pos));
            }else{
                imu_vals_raw(imu_rows,cols) = stod(str.substr(0,pos));
            }
            
            str.erase(0,pos+1);
            cols++;
        }

        if(imu_rows == 3){
            imu_vals_raw(5,cols) = stod(str);
        }else if(imu_rows == 4){
            imu_vals_raw(3,cols) = stod(str);
        }else if(imu_rows == 5){
            imu_vals_raw(4,cols) = stod(str);
        }else{
            imu_vals_raw(imu_rows,cols) = stod(str);
        }

        if(cols != imu_cols - 1){
            std::cout << "Imu Data points size != Time Stamp Size\n";
            file.close();
            exit(1);
        }

        imu_rows++;
    }

    file.close();
    if(imu_rows < 6){
        std::cout << "Imu Data should have 6 rows Ax,Ay,Az,Wz,Wx,Wy, able to read only " << imu_rows + 1 << "\n";
        exit(1);
    }
 
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

    std::cout << "Vicon Data has "  << vicon_cols<< " data points"<<  "\n";

    //resize the matrices based on vicon_cols
    vicon_ts_raw.resize(1,vicon_cols);
    vicon_rots_raw.resize(9,vicon_cols);
    vicon_euler.resize(3,vicon_cols);


    //parse and store vicon time stamps
    cols = 0;
    file.open( vicon_path + std::to_string(data_file) + "_ts.txt");
    while(std::getline(file,str)){

        cols = 0;
        while((pos = str.find(" ")) != std::string::npos){

            vicon_ts_raw(0,cols) = stod(str.substr(0,pos));
            str.erase(0,pos+1);
            cols++;

        }
        vicon_ts_raw(0,cols) = stod(str);
    }
    file.close();


    //parse and store vicon data
    file.open( vicon_path + std::to_string(data_file) + "_rots.txt");
    while(std::getline(file,str)){

        if(vicon_rows >= 9){
            std::cout << "Imu Data should only have 9 rows\n";
            file.close();
            exit(1);
        }

        cols = 0;
        while((pos = str.find(" ")) != std::string::npos){

            if(cols > vicon_cols - 1){
                std::cout << "Vicon Data points size != Time Stamp Size\n";
                file.close();
                exit(1);
            }

            vicon_rots_raw(vicon_rows,cols) = stod(str.substr(0,pos));
            str.erase(0,pos+1);
            cols++;

        }

        if(cols != vicon_cols - 1){
            std::cout << "Vicon Data points size != Time Stamp Size\n";
            file.close();
            exit(1);
        }

        vicon_rots_raw(vicon_rows,cols) = stod(str);
        vicon_rows++;
    }
    file.close();

    if(vicon_rows < 9){
        std::cout << "Imu Data should have 9 rows, able to read only " << imu_rows + 1 << "\n";
        exit(1);
    }

    //adjust data based on pre computed biases and scale factors
    pre_process_data();

    std::cout << "Finished Extracting and Preprocessing data!" << "\n";
    std::cout << "Now running Unscented Kalman Filter\n";

}

/*
    Pre-processes the IMU and Vicon data by scaling and shifting them appropriately
    
*/
void Data_handler::pre_process_data(){

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

    //store r,p,y of vicon and save them.
    std::ofstream file;
    file.open( gt_save_path + std::to_string(file_no) + ".txt");
    if(file.is_open()){

        double roll_gt{0},pitch_gt{0},yaw_gt{0};
        for(int i = 0 ; i < vicon_rots_raw.cols(); i++){

            euler_angles(i);
            file << vicon_euler(0,i) << "," << vicon_euler(1,i) << "," << vicon_euler(2,i);
            if(i != vicon_rots_raw.cols() - 1) file << "\n";

        }

    }
    file.close();

}

/*
    Convert Quaternion to Euler angles  
*/
void Data_handler::euler_angles(const Eigen::Quaterniond q, double& roll, double& pitch, double& yaw){

    roll = std::atan2( 2*(q.w() * q.x() + q.y() * q.z()) ,
       1 - 2* (std::pow(q.x(),2)+ std::pow(q.y(),2)));

    pitch = std::atan2(-2*(q.x()*q.z() - q.w()*q.y()) , 
       std::sqrt(std::pow(2 * (q.w()*q.x() + q.y()*q.z()), 2) + std::pow( 1 - 2 *(std::pow(q.x(),2) + std::pow(q.y(),2)) , 2)));

    yaw = std::atan2( 2*(q.w() * q.z() + q.x() * q.y()) ,
       1 - 2* (std::pow(q.y(),2)+ std::pow(q.z(),2)));

}

/*
    Convert Rotation to Euler angles  
*/
void Data_handler::euler_angles(const int index){

    vicon_euler(0,index) = std::atan2( vicon_rots_raw(7,index), vicon_rots_raw(8,index));

    vicon_euler(1,index) = std::atan2(-vicon_rots_raw(6,index) , std::sqrt(std::pow(vicon_rots_raw(7,index),2) + std::pow(vicon_rots_raw(8,index),2)));

    vicon_euler(2,index) = std::atan2( vicon_rots_raw(3,index), vicon_rots_raw(0,index));

}

/*
    Updates the latest predictions  
*/
void Data_handler::set_predictions(const int index,const double& roll,const double& pitch,const double& yaw){
    
    predictions(0,index) = roll;
    predictions(1,index) = pitch;
    predictions(2,index) = yaw;

    update_error(index);
}

/*
    Updates the error metrics  
*/
void Data_handler::update_error(const int index){

    while(nearest_index < vicon_euler.cols() - 1){

        if(std::abs(imu_ts_raw(0,index) - vicon_ts_raw(0,nearest_index)) < std::abs(imu_ts_raw(0,index) - vicon_ts_raw(0,nearest_index + 1))){
                
                //do slerp
                roll_error += std::pow(min_diff(predictions(0,index) , vicon_euler(0,nearest_index)),2);
                pitch_error += std::pow(min_diff(predictions(1,index) , vicon_euler(1,nearest_index)),2);
                yaw_error += std::pow(min_diff(predictions(2,index) , vicon_euler(2,nearest_index)),2);
                break;

        }else nearest_index++;

    }

}

/*
    Post-processes the Error metrics and displays them
    
*/
void Data_handler::post_process_data(){

    roll_error /= imu_ts_raw.cols();
    pitch_error /= imu_ts_raw.cols();
    yaw_error /= imu_ts_raw.cols();

    std::cout << "MSE for roll, picth and yaw angles in rad are: " << roll_error << ", " << pitch_error << ", " << yaw_error << "\n";

}

/*
    Stores predictions in .txt file  
*/
void Data_handler::store_predicionts(){

    std::ofstream file;

    file.open( save_path + std::to_string(file_no) +".txt" );
    if(file.is_open()){

        for(int i = 0; i < predictions.cols(); i++){

            file << predictions(0,i) << "," << predictions(1,i) << "," << predictions(2,i);
            if(i != predictions.cols() - 1) file << "\n";

        }
    }
    file.close();

    std::cout << "Prediction is complete and is stored in " << save_path << std::to_string(file_no) << ".txt" << "\n";
}