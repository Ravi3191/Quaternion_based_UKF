#include "ukf.h"

/*
    Implementation of Quaternion based Unscented Kalman Filter 
    @param data_num,  data_file is the index of the dataset that we want to evaluate on. It starts from 1
*/
Ukf::Ukf(int data_num,bool gt_available) : data_(data_num,gt_available) {

    //initialize vectors
    g_quat.w() = 0; g_quat.x() = 0; g_quat.y() = 0; g_quat.z() = 9.8;

    Q = PROCESS_NOISE_COEFF * Eigen::MatrixXd::Identity(Q.rows(),Q.cols());
    R = MEASUREMENT_NOISE_COEFF * Eigen::MatrixXd::Identity(R.rows(),R.cols());
    cov_prev_k = INIT_COV_COEFF * Eigen::MatrixXd::Identity(cov_prev_k.rows(),cov_prev_k.cols());

    //set prev_state based on the first data sample
    prev_state.quats = Eigen::AngleAxisd(data_.get_prediction(0,0),Eigen::Vector3d::UnitX())  
                        * Eigen::AngleAxisd(data_.get_prediction(1,0),Eigen::Vector3d::UnitY()) 
                        * Eigen::AngleAxisd(data_.get_prediction(2,0),Eigen::Vector3d::UnitZ());

    // prev_state.omegas = data_.imu_vals_raw.col(0).bottomRows(3);
    prev_state.omegas = data_.get_imu_col(0).bottomRows(3);

    //set other vectors to zero
    W.setZero();
    K.setZero();
    Pzz.setZero();
    Pvv.setZero();
    Pxz.setZero();
    gain.setZero();
    cov_w.setZero();
    mean_z.setZero();
    cholesky.setZero();
    error_omegas.setZero();
    posteriori_cov_z.setZero();
};

/*
    Extracts Quaternions based on process noise and state covariance 
*/
void Ukf::calculate_sigma_points(){

    //calculate sigma points using cholesky decomposition
    cholesky = (Q + cov_prev_k).llt().matrixL();
    W.leftCols(cholesky.cols()) = std::sqrt(12) * cholesky;
    W.rightCols(cholesky.cols()) = -std::sqrt(12) * cholesky;

    cholesky_norms = W.topRows(3).colwise().norm();
    
    // convert axis representation to quaternion
    for(int i = 0; i < W.cols(); i++){

        vector = Eigen::Map<Eigen::Vector3d>(W.col(i).topRows(3).data());
        vector.normalize();
        quat = Eigen::AngleAxisd(cholesky_norms(0,i),vector);
        quat = prev_state.quats * quat;

        sigma_points[i].quats = quat;
        sigma_points[i].omegas = Eigen::Map<Eigen::Vector3d>(W.col(i).bottomRows(3).data()) + prev_state.omegas;

    }

    sigma_points[sigma_points.size()-1].quats = prev_state.quats;
    sigma_points[sigma_points.size()-1].omegas = prev_state.omegas;

}

/*
    Calculates priori's of the sigma points using update model and dt of the next measurment.
*/
void Ukf::update_step(int index){

    for(int i = 0; i < sigma_points.size(); i++){

        norm = sigma_points[i].omegas.norm() * (data_.get_time(index) - data_.get_time(index-1));
        vector = sigma_points[i].omegas;
        vector.normalize();

        quat = Eigen::AngleAxisd(norm,vector);

        sigma_points[i].quats = sigma_points[i].quats * quat;
    }
}

/*
    Calculates the distribution mean, covariance of the priori's
*/
void Ukf::get_distribution_charectoristics(){


    double error{MAXFLOAT};
    double mag{0};

    
    // calcuate the mean rotatin based on gradient descent
    avg_quat = Eigen::Quaterniond::Identity();

    while(error > 0.9){

        error_omega_mean.setZero();
        for(int i = 0; i < sigma_points.size(); i++){
            
            //calculate current error quaternion and then the error omega
            error_quat = sigma_points[i].quats * avg_quat.inverse();
            error_quat.normalize();

            mag = 2 * std::acos(error_quat.w());
            mag /= std::sin(mag/2);
            
            error_omegas.col(i) =  mag * error_quat.vec();
            error_omega_mean += error_omegas.col(i);

        }

        //calculate mean and noramilze it
        error_omega_mean /= double(sigma_points.size());
        error = error_omega_mean.norm();
        error_omega_mean.normalize();

        //update the avg_quat
        error_quats_mean = Eigen::AngleAxisd(error,error_omega_mean);
        avg_quat = error_quats_mean * avg_quat;
        
    }

    mean_state.quats = avg_quat;
    
    //calculate mean omega value and populate the error matrix
    mean_state.omegas.setZero();
    for (int i = 0; i < sigma_points.size();i++){

        mean_state.omegas += sigma_points[i].omegas;
        cov_w.bottomRows(3).col(i) = sigma_points[i].omegas;

    }
    mean_state.omegas /= sigma_points.size();

    //calculate the covariance matrix
    cov_w.topRows(3) = error_omegas;
    cov_w.bottomRows(3).colwise() -= mean_state.omegas;

    cov_prev_k = (cov_w.matrix() * cov_w.matrix().transpose() )/13;

}

/*
    Calculates posterior's charectoristics using measurement model.
*/
void Ukf::measurement_step(){
    
    //transform state and populate the posterior error matrix
    for(int i = 0; i < sigma_points.size(); i++){

        sigma_points[i].quats.normalize();
        posteriori_cov_z.topRows(3).col(i) = (sigma_points[i].quats.inverse() * g_quat * sigma_points[i].quats).vec();
        posteriori_cov_z.bottomRows(3).col(i) = sigma_points[i].omegas;

    }

    //subtract mean from posterior covariance matrix
    mean_z = posteriori_cov_z.rowwise().mean();
    posteriori_cov_z = posteriori_cov_z.colwise() - mean_z;

    //calcuate the posterior covariance matrix and cross corelation matrix
    Pzz = (posteriori_cov_z.matrix() * posteriori_cov_z.matrix().transpose())/13;
    Pvv = Pzz + R;
    Pxz = (cov_w.matrix() * posteriori_cov_z.matrix().transpose()) /13;

}

/*
    Calculates Kalman gain and updates the mean estimate and state covariance.
*/
void Ukf::kalman_update(int index){
    
    //Measure kalman gain and the gain vector
    K = Pxz.matrix() * Pvv.matrix().inverse();
    curr_data = data_.get_imu_col(index);
    gain = K.matrix() * (curr_data - mean_z).matrix();

    //update state covariance matrix
    cov_prev_k -= ((K.matrix() * Pvv.matrix()) * K.matrix().transpose()); 

    //convert gain to quaternion and update the mean state
    vector = gain.topRows(3);
    norm = vector.norm();
    vector.normalize();
    quat = Eigen::AngleAxisd(norm,vector);

    mean_state.quats = mean_state.quats * quat;
    mean_state.omegas += gain.bottomRows(3);

}

/*
    Convert final prediction quat to euler representation and store them in Data handler
*/
void Ukf::update_prediction(int index){

    vector = mean_state.quats.toRotationMatrix().eulerAngles(0, 1, 2);    

    double roll,pitch,yaw;

    data_.euler_angles(mean_state.quats,roll,pitch,yaw);
    data_.set_predictions(index,roll,pitch,yaw);

    prev_state.quats = mean_state.quats;
    prev_state.omegas = mean_state.omegas;

}

/*
    Loops through all the points in the data by calling functions sequentially and saves the final predictions.
*/
void Ukf::run(){

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for(int i = 1; i < data_.get_size(); i++){

        calculate_sigma_points();
        update_step(i);
        get_distribution_charectoristics();
        measurement_step();
        kalman_update(i);
        update_prediction(i);

    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Execution Time = " << std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() << "[ms]" << "\n";

    data_.post_process_data();

    std::cout << "Storing Predictions\n";

    //save predictions
    data_.store_predicionts();
}