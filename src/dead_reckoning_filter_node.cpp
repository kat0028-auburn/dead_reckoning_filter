#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <dead_reckoning_filter/dead_reckoning_filter.h>
#include <sensor_msgs/Imu.h>
#include <dbw_mkz_msgs/WheelSpeedReport.h>
#include <dead_reckoning_filter/InitialData.h>
#include <dead_reckoning_filter/States.h>

class dead_reckoning_filter_node
{
    private:
    ros::NodeHandle n;
    
    ros::Subscriber imu_sub;
    ros::Subscriber wheel_sub;
    ros::Subscriber init_sub;

    ros::Publisher states_pub;

    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void wheel_callback(const dbw_mkz_msgs::WheelSpeedReport::ConstPtr& wheel_msg);
    void init_callback(const dead_reckoning_filter::InitialData::ConstPtr& init_msg);

    Eigen::Matrix<double, 7, 7> P;
    Eigen::Matrix<double, 2, 2> R_imu;
    Eigen::Matrix<double, 2, 2> R_wheel;
    Eigen::Matrix<double, 2, 2> Q; 

    double P11, P22, P33, P44, P55, P66, P77;
    double Ri11, Ri22;
    double Rw11, Rw22;
    double Q11, Q22;

    double accel_x_bias;
    double gyro_z_bias;
    double heading;

    double accel_x;
    double gyro_z;
    double omega_rr;
    double omega_rl;
    double imu_time;
    double wheel_time;

    bool initialized;

    double R_rr;
    double R_rl;
    double tw;

    public:
    dead_reckoning_filter::States states_msg;
    double N, E, dN, dE, yaw, yaw_rate, vx;
    Eigen::Matrix<double, 7, 1> x_hat;

    double dt;

    void publish_states();
    
    inertial_filter mech_filter;
    
    dead_reckoning_filter_node();
    ~dead_reckoning_filter_node();
};

dead_reckoning_filter_node::dead_reckoning_filter_node()
{
    imu_sub = n.subscribe("/kvh/imu", 1, &dead_reckoning_filter_node::imu_callback, this);
    wheel_sub = n.subscribe("/vehicle/wheel_speed_report", 1, &dead_reckoning_filter_node::wheel_callback, this);
    init_sub = n.subscribe("/filter_bias", 1, &dead_reckoning_filter_node::init_callback, this);

    states_pub = n.advertise<dead_reckoning_filter::States>("/filtered_states", 1);

    n.param<double>("/dead_reckoning_filter/P11", P11, 0);
    n.param<double>("/dead_reckoning_filter/P22", P22, 0);
    n.param<double>("/dead_reckoning_filter/P33", P33, 0);
    n.param<double>("/dead_reckoning_filter/P44", P44, 0);
    n.param<double>("/dead_reckoning_filter/P55", P55, 0);
    n.param<double>("/dead_reckoning_filter/P66", P66, 0);
    n.param<double>("/dead_reckoning_filter/P77", P77, 0);
    
    n.param<double>("/dead_reckoning_filter/Ri11", Ri11, 0);
    n.param<double>("/dead_reckoning_filter/Ri22", Ri22, 0);
    n.param<double>("/dead_reckoning_filter/Rw11", Rw11, 0);
    n.param<double>("/dead_reckoning_filter/Rw22", Rw22, 0);

    n.param<double>("/dead_reckoning_filter/Q11", Q11, 0);
    n.param<double>("/dead_reckoning_filter/Q22", Q22, 0);

    n.param<double>("/dead_reckoning_filter/dt", dt, 0);
    n.param<double>("/dead_reckoning_filter/R_rr", R_rr, 0);
    n.param<double>("/dead_reckoning_filter/R_rl", R_rl, 0);
    n.param<double>("/dead_reckoning_filter/tw", tw, 0);

    P <<P11, 0, 0, 0, 0, 0, 0,
        0, P22, 0, 0, 0, 0, 0, 
        0, 0, P33, 0, 0, 0, 0, 
        0, 0, 0, P44, 0, 0, 0, 
        0, 0, 0, 0, P55, 0, 0,
        0, 0, 0, 0, 0, P66, 0,
        0, 0, 0, 0, 0, 0, P77;

    Q <<Q11, 0,
        0, Q22;
    
    R_imu<<Ri11, 0,
            0, Ri22;
    
    R_wheel<<Rw11, 0,
            0, Rw22;
    
    mech_filter.initialize_filter(dt, R_rr, R_rl, tw, P, R_imu, R_wheel, Q);

    n.param<double>("/dead_reckoning_filter/accel_x_bias", accel_x_bias, 0);
    n.param<double>("/dead_reckoning_filter/gyro_z_bias", gyro_z_bias, 0);
    n.param<double>("/dead_reckoning_filter/heading", heading, 0);
    n.param<bool>("/dead_reckoning_filter/initialized", initialized, false);

    heading = heading * M_PI/180.0;

    mech_filter.set_bias(accel_x_bias, gyro_z_bias, heading, initialized);
}

dead_reckoning_filter_node::~dead_reckoning_filter_node()
{
    //do nothing
}

void dead_reckoning_filter_node::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    accel_x = imu_msg->linear_acceleration.x;
    gyro_z = -1*(imu_msg->angular_velocity.z);

    imu_time = imu_msg->header.stamp.toSec();

    mech_filter.imu_update(accel_x, gyro_z, imu_time);
    mech_filter.time_update(imu_time);
    publish_states();
}

void dead_reckoning_filter_node::wheel_callback(const dbw_mkz_msgs::WheelSpeedReport::ConstPtr& wheel_msg)
{
    omega_rl = wheel_msg->rear_left;
    omega_rr = wheel_msg->rear_right;

    wheel_time = wheel_msg->header.stamp.toSec();

    mech_filter.wheel_update(omega_rr, omega_rl, wheel_time);
    mech_filter.time_update(wheel_time);
}

void dead_reckoning_filter_node::init_callback(const dead_reckoning_filter::InitialData::ConstPtr& init_msg)
{
    accel_x_bias = init_msg->accel_bias.x;
    gyro_z_bias = init_msg->gyro_bias.z;
    heading = (init_msg->heading)*M_PI/180;

    initialized = true;
    std::cout<<"Accel x: "<<accel_x_bias<<std::endl;
    std::cout<<"Gyro z: "<<gyro_z_bias<<std::endl;
    std::cout<<"Heading: "<<heading<<std::endl;

    mech_filter.set_bias(accel_x_bias, gyro_z_bias, heading, initialized);
}

void dead_reckoning_filter_node::publish_states()
{
    x_hat = mech_filter.get_states();
    N = x_hat(0,0);
    E = x_hat(1,0);
    dN = x_hat(2,0);
    dE = x_hat(3,0);
    yaw = x_hat(4,0);
    yaw_rate = x_hat(5,0);
    vx = x_hat(6,0);

    states_msg.N = N;
    states_msg.E = E;
    states_msg.dE = dE;
    states_msg.dN = dN;
    states_msg.yaw = yaw;
    states_msg.yaw_rate = yaw_rate;
    states_msg.vx = vx;
    states_msg.header.stamp = ros::Time::now();

    states_pub.publish(states_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dead_reckoning_filter");
    dead_reckoning_filter_node filter_node;

    int rate;
    rate = 1/filter_node.dt;
    ros::Rate loop_rate(rate);

    /*while(ros::ok())
    {
        filter_node.mech_filter.time_update();
        filter_node.publish_states();
        ros::spinOnce();
        loop_rate.sleep();
    }*/
    ros::spin();

    return 0;
}