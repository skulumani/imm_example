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
        KalmanFilter( const size_t& dim_x_in,
                const size_t& dim_z_in,
                const size_t& dim_u_in);
        KalmanFilter( const Eigen::Ref<const Eigen::MatrixXd>& F_in,
                const Eigen::Ref<const Eigen::MatrixXd>& B_in,
                const Eigen::Ref<const Eigen::MatrixXd>& Q_in,
                const Eigen::Ref<const Eigen::MatrixXd>& H_in,
                const Eigen::Ref<const Eigen::MatrixXd>& R_in);

        virtual ~KalmanFilter( void ) {};
        
        void set_state(const Eigen::Ref<const Eigen::VectorXd>& x_in);
        void set_cov(const Eigen::Ref<const Eigen::MatrixXd>& P_in);
        
        Eigen::VectorXd get_state( void ) const;
        Eigen::MatrixXd get_cov( void ) const;

        void predict( void ); // predict one step
        void update( const Eigen::Ref<const Eigen::VectorXd>& z_in );
    private:
        
        // initialize all the member variables so we don't get in trouble
        void init( void );
        
        size_t dim_x, dim_z, dim_u; // size of arrays

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


