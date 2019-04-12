#ifndef KALMAN_HPP
#define KALMAN_HPP
#include <Eigen/Dense>

/** @class KalmanFilter

  @brief Discrete time kalman filter

  extended description

  @author Shankar Kulumani
  @version 20190412
*/  
class KalmanFilter {
    public:
        KalmanFilter( void );
        KalmanFilter( const Eigen::Ref<const Eigen::MatrixXd>& F,
                const Eigen::Ref<const Eigen::MatrixXd>& B,
                const Eigen::Ref<const Eigen::MatrixXd>& Q,
                const Eigen::Ref<const Eigen::MatrixXd>& H,
                const Eigen::Ref<const Eigen::MatrixXd>& R);

        virtual ~KalmanFilter( void ) {};


    private:
        
        // initialize everything else
        void init( void );
        
        Eigen::VectorXd x; // state estimate
        Eigen::MatrixXd P; // covariance
        Eigen::MatrixXd Q, R; // process and measurement covariance
        Eigen::MatrixXd F, B; // state and control transition matrices
        Eigen::MatrixXd H; // measurement function

        Eigen::VectorXd z; // measurement vector

        // Kalman gain and measurement update data
        Eigen::VectorXd y_pre, y_post; // innovation/measurement pre post fit residual
        Eigen::MatrixXd S; // innovation (pre fit) residual
        Eigen::MatrixXd K; // kalman gain

        // State and covariance after prediction is called (a priori)
        Eigen::VectorXd x_prior;
        Eigen::MatrixXd P_prior;

        // state and covariance after update is called (a posteriori)
        Eigen::VectorXd x_post;
        Eigen::MatrixXd P_post;


};


#endif


