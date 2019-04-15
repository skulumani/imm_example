#include "kalman.hpp"

KalmanFilter::KalmanFilter( void ) {
    dim_x = 2;
    dim_z = 2;
    dim_u = 1;
    init();
}

KalmanFilter::KalmanFilter(const size_t& dim_x_in, const size_t& dim_z_in, 
        const size_t& dim_u_in) {
    
    dim_x = dim_x_in;
    dim_z = dim_z_in;
    dim_u = dim_u_in;

    init();
}

void KalmanFilter::init( void ) {
// initialize all our matrices and sizes

    x = Eigen::VectorXd::Zero(dim_x);
    P = Eigen::MatrixXd::Identity(dim_x, dim_x);
    Q = Eigen::MatrixXd::Identity(dim_x, dim_x);
    R = Eigen::MatrixXd::Identity(dim_z, dim_z);

    F = Eigen::MatrixXd::Identity(dim_x, dim_x);
    B = Eigen::MatrixXd::Zero(dim_x, dim_u);

    H = Eigen::MatrixXd::Identity(dim_z, dim_x);

    z = Eigen::VectorXd::Zero(dim_z);
    
    y_pre = Eigen::VectorXd::Zero(dim_z);
    y_post = Eigen::VectorXd::Zero(dim_z);

    S = Eigen::MatrixXd::Zero(dim_z, dim_z);
    K = Eigen::MatrixXd::Zero(dim_x, dim_z);

    x_prior = Eigen::VectorXd::Zero(dim_x);
    x_post = Eigen::VectorXd::Zero(dim_x);

    P_prior = Eigen::MatrixXd::Identity(dim_x, dim_x);
    P_post = Eigen::MatrixXd::Identity(dim_x, dim_x);
}
