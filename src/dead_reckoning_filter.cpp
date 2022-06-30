#include <dead_reckoning_filter/dead_reckoning_filter.h>

inertial_filter::inertial_filter()
{
    wheel_counter = 0;
    imu_counter = 0;

    imu_time = 0;
    wheel_time = 0;
    time = 0;
    x_hat<<0, 0, 0, 0, 0, 0, 0;
    E = 0;
    N = 0;
    dN = 0;
    dE = 0;
    yaw = 0;
    yaw_rate = 0;
    vx = 0;   
    initialized = false; 

    yaw_old = 0;

    Cd<<0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 1;

    I<< 1, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 
        0, 0, 0, 1, 0, 0, 0, 
        0, 0, 0, 0, 1, 0, 0, 
        0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 1;
    
    Bw<<0, 0,
        0, 0,
        0, 0, 
        0, 0, 
        0, 0, 
        1, 0,
        0, 1; 
    
    x_hat<<N, E, dN, dE, yaw, yaw_rate, vx;

    std::cout<<"Dead Reckoning Filter Started"<<std::endl;
}

inertial_filter::~inertial_filter()
{
    std::cout<<"Wheel Counter: "<<wheel_counter<<std::endl;
    std::cout<<"Imu Counter: "<<imu_counter<<std::endl;
    std::cout<<"Filter Shutdown"<<std::endl;
}

void inertial_filter::initialize_filter(double dt_in, double R_rr_in, double R_rl_in, double tw_in, Eigen::Matrix<double, 7, 7> P_in, Eigen::Matrix<double, 2, 2> R_imu_in, Eigen::Matrix<double, 2, 2> R_wheel_in, Eigen::Matrix<double, 2, 2> Q_in)
{
    dt = dt_in; //For time update
    R_rr = R_rr_in;
    R_rl = R_rl_in; //Wheel Radius
    tw = tw_in; //track width
    P = P_in;
    R_imu = R_imu_in;
    R_wheel = R_wheel_in;
    Q = Q_in; //Initialize Covariance Matrices
    Q_d = Bw*Q*Bw.transpose()*dt;
    std::cout<<P<<std::endl;
    std::cout<<"Dead Reckoning Filter Initialized"<<std::endl;
}

void inertial_filter::set_bias(double accel_x_bias_in, double gyro_z_bias_in, double heading_in, bool initialized_in)
{
    accel_x_bias = accel_x_bias_in;
    gyro_z_bias = gyro_z_bias_in;
    yaw = heading_in;
    yaw_old = yaw;
    initialized = initialized_in;

    x_hat<<N, E, dN, dE, yaw, yaw_rate, vx;
    std::cout<<"Bias Corrected, Reset Heading"<<std::endl;
    std::cout<<accel_x_bias<<"\t"<<gyro_z_bias<<"\t"<<yaw<<std::endl;

}

void inertial_filter::wheel_update(double omega_rr, double omega_rl, double wheel_time_new)
{
    wheel_counter = wheel_counter + 1;

    if(wheel_time==0)
    {
        dt_wheel = 0;
    }
    else
    {
        dt_wheel = wheel_time_new - wheel_time;
    }
    wheel_time = wheel_time_new;

    vx_wheel = 0.5*(omega_rl*R_rl) + 0.5*(omega_rr*R_rr);
    yaw_rate_wheel = (omega_rl*R_rl - omega_rr*R_rr)/tw;

    y_hat = Cd*x_hat;
    y_wheel<<yaw_rate_wheel, vx_wheel;

    temp = Cd*P*Cd.transpose() + R_wheel;
    L_wheel = P*Cd.transpose()*(temp.inverse());

    x_hat = x_hat + L_wheel*(y_wheel - y_hat);

    N = x_hat(0,0);
    E = x_hat(1,0);
    dN = x_hat(2,0);
    dE = x_hat(3,0);
    yaw = x_hat(4,0);
    yaw_rate = x_hat(5,0);
    vx = x_hat(6,0);

    P = (I - L_wheel*Cd)*P;
}

void inertial_filter::imu_update(double accel_x_in, double gyro_z_in, double imu_time_new)
{
    imu_counter = imu_counter + 1;

    if(imu_time==0)
    {
        dt_imu = 0;
    }
    else
    {
        dt_imu = imu_time_new - imu_time;
    }
    imu_time = imu_time_new;
    
    if(initialized)
    {
        //std::cout<<"Start"<<std::endl;
        vx_imu = vx + (accel_x_in-accel_x_bias)*dt_imu;
        yaw_rate_imu = (gyro_z_in-gyro_z_bias);

        y_hat = Cd*x_hat;
        y_imu<<yaw_rate_imu, vx_imu;

        temp = Cd*P*(Cd.transpose())+R_imu;
        L_imu = P*Cd.transpose()*(temp.inverse());

        x_hat = x_hat + L_imu*(y_imu-y_hat);

        N = x_hat(0,0);
        E = x_hat(1,0);
        dN = x_hat(2,0);
        dE = x_hat(3,0);
        yaw = x_hat(4,0);
        yaw_rate = x_hat(5,0);
        vx = x_hat(6,0);

        P=(I-L_imu*Cd)*P;
    }
}

void inertial_filter::time_update(double time_new)
{
    if(time==0)
    {
        dt = 0;
    }
    else
    {
        dt = time_new-time;
    }
    time = time_new;
    
    N = x_hat(0,0);
    E = x_hat(1,0);
    dN = x_hat(2,0);
    dE = x_hat(3,0);
    yaw = x_hat(4,0);
    yaw_rate = x_hat(5,0);
    vx = x_hat(6,0);

    if(yaw>2*M_PI)
    {
        yaw = yaw - 2*M_PI;
    }
    else if(yaw<0)
    {
        yaw += yaw+2*M_PI;
    }

    N = N + dN*dt;
    E = E + dE*dt;
    dN = vx * cos(yaw);
    dE = vx * sin(yaw);

    yaw_old = yaw;

    yaw = yaw + yaw_rate*dt;
    //Maybe remove these lines 
    yaw_rate = yaw_rate;
    vx = vx;
    //************************

    x_hat<<N, E, dN, dE, yaw, yaw_rate, vx;

    A<< 1, 0, dt, 0, 0, 0, 0,
        0, 1, 0, dt, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, cos(0.5*(yaw - yaw_old)),
        0, 0, 0, 0, 0, 0, sin(0.5*(yaw - yaw_old)),
        0, 0, 0, 0, 1, dt, 0,
        0, 0, 0, 0, 0, 1, 0, 
        0, 0, 0, 0, 0, 0, 1;
    
    P = A*P*A.transpose() + Q_d;
}

Eigen::Matrix<double, 7, 1> inertial_filter::get_states()
{
    return x_hat;
}

Eigen::Matrix<double, 7, 7> inertial_filter::get_covar()
{
    return P;
}

