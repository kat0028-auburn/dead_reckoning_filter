#include <eigen3/Eigen/Dense>
#include <iostream>

class inertial_filter
{
    private:
    Eigen::Matrix<double, 7, 7> P;
    Eigen::Matrix<double, 2, 2> R_imu;
    Eigen::Matrix<double, 2, 2> R_wheel;
    Eigen::Matrix<double, 2, 2> Q;
    Eigen::Matrix<double, 7, 7> Q_d;

    Eigen::Matrix<double, 7, 7> A;
    Eigen::Matrix<double, 7, 2> Bw;
    Eigen::Matrix<double, 7, 2> L_imu;
    Eigen::Matrix<double, 7, 2> L_wheel;
    Eigen::Matrix<double, 2, 7> Cd;
    Eigen::Matrix<double, 7, 7> I;

    Eigen::Matrix<double, 7, 1> x_hat;

    Eigen::Matrix<double, 2, 1> y_hat;
    Eigen::Matrix<double, 2, 1> y_wheel;
    Eigen::Matrix<double, 2, 1> y_imu;

    Eigen::Matrix<double, 2, 2> temp;

    bool initialized;

    double dt;
    double time;
    double N;
    double E;
    double dN;
    double dE;
    double yaw;
    double yaw_rate;
    double vx;

    double yaw_old;

    double wheel_counter;
    double imu_counter;

    double vx_wheel;
    double vx_imu;
    double yaw_rate_wheel;
    double yaw_rate_imu;
    double dt_imu;
    double dt_wheel;
    double wheel_time;
    double imu_time;
    
    double gyro_z_bias;
    double accel_x_bias;
    double R_rr;
    double R_rl;
    double tw;

    public:
    void initialize_filter(double dt_in, double R_rr_in, double R_rl_in, double tw_in, Eigen::Matrix<double, 7, 7> P_in, Eigen::Matrix<double, 2, 2> R_imu_in, Eigen::Matrix<double, 2, 2> R_wheel_in, Eigen::Matrix<double, 2, 2> Q_in);
    void set_bias(double accel_x_bias_in, double gyro_z_bias_in, double heading_in, bool initialized_in);
    void wheel_update(double omega_rr, double omega_rl, double wheel_time_new);
    void imu_update(double accel_x_in, double gyro_z_in, double imu_time_new);
    void time_update(double time_new);

    Eigen::Matrix<double, 7, 1> get_states();
    Eigen::Matrix<double, 7, 7> get_covar();

    inertial_filter();
    ~inertial_filter();
};