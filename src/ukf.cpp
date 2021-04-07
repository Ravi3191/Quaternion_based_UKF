#include "ukf.h"

ukf::ukf(int data_num, bool vicon_available) : Data(data_num,vicon_available) {

    //initialize vectors
    g_quat.w() = 0; g_quat.x() = 0; g_quat.y() = 0; g_quat.z() = 9.8;

    Q = 11 * Eigen::MatrixXd::Identity(Q.rows(),Q.cols());
    R = 15 * Eigen::MatrixXd::Identity(R.rows(),R.cols());
    cov_prev_k = 50 * Eigen::MatrixXd::Identity(cov_prev_k.rows(),cov_prev_k.cols());

    prev_state.quats = Eigen::AngleAxisd(Data.roll_pred(0,0),Eigen::Vector3d::UnitX())  
                        * Eigen::AngleAxisd(Data.pitch_pred(0,0),Eigen::Vector3d::UnitY()) 
                        * Eigen::AngleAxisd(Data.yaw_pred(0,0),Eigen::Vector3d::UnitZ());

    prev_state.omegas = Data.imu_vals_raw.bottomRows(3).col(0);

    W.setZero();
    prior_z.setZero();
    cov_w.setZero();
    Pzz.setZero();
    Pvv.setZero();
    Pxz.setZero();
    mean_z.setZero();
    K.setZero();
    gain.setZero();
    cholesky.setZero();
    //initialize prev_state to the first roll, pitch, yaw value and also sigma points

    // error_omegas (sigma_points.size(),Eigen::Vector3d::Identity());
    error_omegas.setZero();
};

void ukf::calculate_sigma_points(){

    //calculate sigma points by cholesky decomposition
    cholesky = (Q + cov_prev_k).llt().matrixL();
    W.leftCols(cholesky.cols()) = std::sqrt(12) * cholesky;
    W.rightCols(cholesky.cols()) = -std::sqrt(12) * cholesky;

    cholesky_norms = W.topRows(3).colwise().norm();

    if(debug){
        std::cout << "cholesky matrix:" << "\n";
        std::cout << std::sqrt(12) *cholesky << "\n";

        std::cout << "Printing Sigma Points" << "\n";
    }

    for(int i = 0; i < W.cols(); i++){

        vector = Eigen::Map<Eigen::Vector3d>(W.col(i).topRows(3).data());
        vector.normalize();
        quat = Eigen::AngleAxisd(cholesky_norms(0,i),vector);
        quat = prev_state.quats * quat;

        sigma_points[i].quats = quat;
        sigma_points[i].omegas = Eigen::Map<Eigen::Vector3d>(W.col(i).bottomRows(3).data()) + prev_state.omegas;

        if(debug){
            print_state(sigma_points[i].quats);
            print_vector(sigma_points[i].omegas);
        }
    }

    sigma_points[sigma_points.size()-1].quats = prev_state.quats;
    sigma_points[sigma_points.size()-1].omegas = prev_state.omegas;

    if(debug){
        print_state(sigma_points[sigma_points.size()-1].quats);
        print_vector(sigma_points[sigma_points.size()-1].omegas);
    }

}

void ukf::update_step(int index){

    if(debug){
        std::cout << "Printing Updated Sigma Points" << "\n";
    }

    for(int i = 0; i < sigma_points.size(); i++){

        norm = sigma_points[i].omegas.norm() * (Data.imu_ts_raw(0,index) - Data.imu_ts_raw(0,index-1));
        vector = sigma_points[i].omegas;
        vector.normalize();

        quat = Eigen::AngleAxisd(norm,vector);

        sigma_points[i].quats = sigma_points[i].quats * quat;

        if(debug){
            print_state(sigma_points[i].quats);
            print_vector(sigma_points[i].omegas);
        }
    }
}

void ukf::get_distribution_charectoristics(){


    double error{MAXFLOAT};
    double mag{0};

    avg_quat = Eigen::Quaterniond::Identity();

    while(error > 0.9){

        error_omega_mean.setZero();
        for(int i = 0; i < sigma_points.size(); i++){

            error_quat = sigma_points[i].quats * avg_quat.inverse();
            error_quat.normalize();
            mag = 2 * std::acos(error_quat.w());
            mag /= std::sin(mag/2);
            error_omegas.col(i) =  mag * error_quat.vec();
            error_omega_mean += error_omegas.col(i);

        }

        error_omega_mean /= double(sigma_points.size());
        error = error_omega_mean.norm();
        error_omega_mean.normalize();
        error_quats_mean = Eigen::AngleAxisd(error,error_omega_mean);
        avg_quat = error_quats_mean * avg_quat;
        

    }

    if(debug){
            
            std::cout << "error omegas:" << "\n";
            std::cout << error_omegas << "\n";        
    }


    mean_state.quats = avg_quat;
    mean_state.omegas.setZero();
    for (int i = 0; i < sigma_points.size();i++){
        mean_state.omegas += sigma_points[i].omegas;
        cov_w.bottomRows(3).col(i) = sigma_points[i].omegas;
    }
    mean_state.omegas /= sigma_points.size();

    if(debug){
        std::cout << "Printing Average Transformed State" << "\n";
        print_state(mean_state.quats);
        print_vector(mean_state.omegas);
    }

    cov_w.topRows(3) = error_omegas;
    cov_w.bottomRows(3).colwise() -= mean_state.omegas;

    cov_prev_k = (cov_w.matrix() * cov_w.matrix().transpose() )/13;


    if(debug){
            
            std::cout << "cov_w:" << "\n";
            std::cout << cov_w << "\n";        

            std::cout << "cov_prev_k:" << "\n";
            std::cout << cov_prev_k << "\n";        
    }

}


void ukf::measurement_step(){
    //try normalizing quat it if error is high
    for(int i = 0; i < sigma_points.size(); i++){
        sigma_points[i].quats.normalize();
        prior_z.topRows(3).col(i) = (sigma_points[i].quats.inverse() * g_quat * sigma_points[i].quats).vec();
        prior_z.bottomRows(3).col(i) = sigma_points[i].omegas;
    }

    mean_z = prior_z.rowwise().mean();
    prior_z = prior_z.colwise() - mean_z;

    Pzz = (prior_z.matrix() * prior_z.matrix().transpose())/13;

    Pvv = Pzz + R;
    Pxz = (cov_w.matrix() * prior_z.matrix().transpose()) /13;

    if(debug){
            
            std::cout << "prior_z:" << "\n";
            std::cout << prior_z << "\n";        

            std::cout << "mean_z:" << "\n";
            std::cout << mean_z << "\n";

            std::cout << "Pzz:" << "\n";
            std::cout << Pzz << "\n";        

            std::cout << "Pvv:" << "\n";
            std::cout << Pvv << "\n";        

            std::cout << "Pxz:" << "\n";
            std::cout << Pxz << "\n";        
    }

}

void ukf::kalman_update(int index){
    
    K = Pxz.matrix() * Pvv.matrix().inverse();
    curr_data = Data.imu_vals_raw.col(index);
    gain = K.matrix() * (curr_data - mean_z).matrix();

    cov_prev_k -= ((K.matrix() * Pvv.matrix()) * K.matrix().transpose()); 

    if(debug){
            
            std::cout << "cov_prev_k:" << "\n";
            std::cout << cov_prev_k << "\n";        

            std::cout << "K:" << "\n";
            std::cout << K << "\n";        

            std::cout << "gain:" << "\n";
            std::cout << gain << "\n";        
    }

    vector = gain.topRows(3);
    norm = vector.norm();
    vector.normalize();
    quat = Eigen::AngleAxisd(norm,vector);

    mean_state.quats = mean_state.quats * quat;
    mean_state.omegas += gain.bottomRows(3);

    if(debug){
        std::cout << "Printing Average Transformed State" << "\n";
        print_state(mean_state.quats);
        print_vector(mean_state.omegas);
    }
}

void ukf::update_prediction(int index){

    vector = mean_state.quats.toRotationMatrix().eulerAngles(0, 1, 2);    

    double roll,pitch,yaw;

    euler_angles(mean_state.quats,roll,pitch,yaw);

    Data.roll_pred(0,index) = roll;
    Data.pitch_pred(0,index) = pitch;
    Data.yaw_pred(0,index) = yaw;

    prev_state.quats = mean_state.quats;
    prev_state.omegas = mean_state.omegas;

    if(debug){
        std::cout << "Printing roll, pitch, yaw:" << "\n";
        std::cout << roll << "," << pitch << "," << yaw << "\n";
    }

}

void ukf::run(){

    for(int i = 1; i < Data.imu_ts_raw.cols(); i++){

        if(debug){
            // std::cout << "Q matrix:" << "\n";
            // std::cout << Q << "\n";

            // std::cout << "R matrix:" << "\n";
            // std::cout << R << "\n";

            // std::cout << "g_qauat:" << "\n";
            // std::cout << g_quat.w() << "," << g_quat.vec() << "\n";

            std::cout << "cov_prev_k:" << "\n";
            std::cout << cov_prev_k << "\n";

            std::cout << "prev_state:" << "\n";
            print_state(prev_state.quats);
            print_vector(prev_state.omegas);
        }

        calculate_sigma_points();
        update_step(i);
        get_distribution_charectoristics();
        measurement_step();
        kalman_update(i);
        update_prediction(i);

        if(debug){
            if(i == 2) break;
        }

    }

    std::ofstream file;

    file.open("predictions.txt");
    if(file.is_open()){
        for(int i = 0; i < Data.roll_pred.cols(); i++){
            file << Data.roll_pred(0,i) << "," << Data.pitch_pred(0,i) << "," << Data.yaw_pred(0,i);
            if(i != Data.roll_pred.cols() - 1) file << "\n";
        }
    }
    file.close();


    file.open("roll_predictions.txt");
    if(file.is_open()){
        for(int i = 0; i < Data.roll_pred.cols(); i++){
            file << Data.roll_pred(0,i);
            if(i != Data.roll_pred.cols()-1) file << ",";
        }
        
    }
    file.close();

    file.open("pitch_predictions.txt");
    if(file.is_open()){
        for(int i = 0; i < Data.pitch_pred.cols(); i++){
            file << Data.pitch_pred(0,i);
            if(i != Data.pitch_pred.cols()-1) file << ",";
        }
        
    }
    file.close();

    file.open("yaw_predictions.txt");
    if(file.is_open()){
        for(int i = 0; i < Data.yaw_pred.cols(); i++){
            file << Data.yaw_pred(0,i);
            if(i != Data.yaw_pred.cols()-1) file << ",";
        }
        
    }
    file.close();

    std::cout << Data.roll_pred.rows() << "," << Data.roll_pred.cols() << "\n";
}

void ukf::print_state(Eigen::Quaterniond q){

    std::cout << q.w() << "," << q.x()<< "," << q.y()<< "," << q.z() << "\n";

}

void ukf::print_vector(Eigen::Vector3d v){
    std::cout << v(0) << "," << v(1) << "," << v(2) << "\n";
}

void ukf::euler_angles(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw){

    roll = std::atan2( 2*(q.w() * q.x() + q.y() * q.z()) ,
       1 - 2* (std::pow(q.x(),2)+ std::pow(q.y(),2)));

    pitch = std::asin(2*(q.w()*q.y() - q.x()*q.z()));

    yaw = std::atan2( 2*(q.w() * q.z() + q.x() * q.y()) ,
       1 - 2* (std::pow(q.y(),2)+ std::pow(q.z(),2)));

}