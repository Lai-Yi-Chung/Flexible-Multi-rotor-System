#include <ros/ros.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "forceest.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"
#include "math.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include "mavros_msgs/RCOut.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/BatteryState.h"
#include "nav_msgs/Odometry.h"

#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include "least_square.h"
#include <random>
#include "time.h"

#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/ModelState.h>
//#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/BodyRequest.h>

#include "rls.h"

int drone_flag;
int bias_flag;
forceest forceest1(statesize,measurementsize);
geometry_msgs::Point euler, euler_ref, force, body_torque,torque_ground, bias, angular_v,
                     pwm,battery_p,pose,thrust, force_nobias, rope_theta_v,least;

//-----------------callback -------------------
sensor_msgs::Imu drone2_imu;
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
    drone2_imu = *msg;
}

sensor_msgs::Imu raw;
void raw_cb(const sensor_msgs::Imu::ConstPtr &msg){
    raw = *msg;
}

geometry_msgs::PoseStamped drone2_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    drone2_pose = *msg;
}

geometry_msgs::TwistStamped drone2_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    drone2_vel = *msg;
}

mavros_msgs::RCOut rc_out;
void rc_cb(const mavros_msgs::RCOut::ConstPtr &msg){
    rc_out = *msg;
}

sensor_msgs::MagneticField mag;
void mag_cb(const sensor_msgs::MagneticField::ConstPtr &msg){
    mag = *msg;
}

sensor_msgs::BatteryState battery;
void battery_cb(const sensor_msgs::BatteryState::ConstPtr &msg){
    battery = *msg;
}

nav_msgs::Odometry odom;
void odom_cb(const nav_msgs::Odometry::ConstPtr &msg){
    odom = *msg;
    forceest1.quat_m << drone2_pose.pose.orientation.x, drone2_pose.pose.orientation.y, drone2_pose.pose.orientation.z, drone2_pose.pose.orientation.w;
}

geometry_msgs::Point acc_bias;
void acc_cb(const geometry_msgs::Point::ConstPtr &msg){
    acc_bias = *msg;
}

geometry_msgs::Point gyro_bias;
void gyro_cb(const geometry_msgs::Point::ConstPtr &msg){
    gyro_bias = *msg;
}
//-----------------force callback----------------------
Eigen::Vector3d  point_0,point_1, point_2, point_3;
geometry_msgs::WrenchStamped wrench_0, wrench_1, wrench_2, wrench_3;
void force_0_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    wrench_0= *msg;
    point_0<< wrench_0.wrench.force.x,
               wrench_0.wrench.force.y,
               wrench_0.wrench.force.z;
}
void force_1_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    wrench_1= *msg;
    point_1<< wrench_1.wrench.force.x,
               wrench_1.wrench.force.y,
               wrench_1.wrench.force.z;
}
void force_2_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    wrench_2= *msg;
    point_2<< wrench_2.wrench.force.x,
                  wrench_2.wrench.force.y,
                  wrench_2.wrench.force.z;
}
void force_3_cb(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    wrench_3= *msg;
    point_3<< wrench_3.wrench.force.x,
               wrench_3.wrench.force.y,
               wrench_3.wrench.force.z;
}

char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "force_estimate");
    ros::NodeHandle nh;

    std::string topic_imu, topic_mocap, topic_thrust, topic_vel, topic_mag,
                topic_raw,topic_battery,topic_odom, topic_acc_bias, topic_gyro_bias;

    ros::param::get("~topic_imu", topic_imu);
    ros::param::get("~topic_mocap", topic_mocap);
    ros::param::get("~topic_thrust", topic_thrust);
    ros::param::get("~topic_vel", topic_vel);
    ros::param::get("~topic_drone", drone_flag);
    ros::param::get("~topic_mag", topic_mag);
    ros::param::get("~topic_raw", topic_raw);
    ros::param::get("~topic_battery", topic_battery);
    ros::param::get("~topic_odom", topic_odom);
    ros::param::get("~topic_acc_bias", topic_acc_bias);
    ros::param::get("~topic_gyro_bias", topic_gyro_bias);

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(topic_imu, 2, imu_cb);
    ros::Subscriber raw_sub = nh.subscribe<sensor_msgs::Imu>(topic_raw, 2, raw_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_mocap, 2, pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(topic_vel, 2, vel_cb);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCOut>(topic_thrust, 2, rc_cb);
    ros::Subscriber mag_sub = nh.subscribe<sensor_msgs::MagneticField>(topic_mag, 2, mag_cb);
    ros::Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>(topic_battery, 2, battery_cb);
    ros::Subscriber acc_sub = nh.subscribe<geometry_msgs::Point>(topic_acc_bias, 2, acc_cb);
    ros::Subscriber gyro_sub = nh.subscribe<geometry_msgs::Point>(topic_gyro_bias, 2, gyro_cb);

    ros::Publisher euler_pub = nh.advertise<geometry_msgs::Point>("euler", 2);
    ros::Publisher euler_ref_pub = nh.advertise<geometry_msgs::Point>("euler_ref", 2);

    ros::Publisher body_torque_pub1 = nh.advertise<geometry_msgs::Point>("body_torque1", 2);
    ros::Publisher torque_ground_pub1 = nh.advertise<geometry_msgs::Point>("torque_ground1", 2);

    ros::Publisher bias_pub = nh.advertise<geometry_msgs::Point>("bias", 2);
    ros::Publisher angular_v_pub = nh.advertise<geometry_msgs::Point>("angular_v", 2);
    ros::Publisher pwm_pub = nh.advertise<geometry_msgs::Point>("pwm", 2);
    ros::Publisher battery_pub = nh.advertise<geometry_msgs::Point>("battery", 2);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Point>("pose", 2);
    ros::Publisher thrust_pub = nh.advertise<geometry_msgs::Point>("thrust", 2);
    ros::Publisher least_square_pub = nh.advertise<geometry_msgs::Point>("least_square", 2);


    ros::Subscriber force_sub_0 = nh.subscribe<geometry_msgs::WrenchStamped>
            ("/uav1/rotor_0_ft", 3, force_0_cb);
    ros::Subscriber force_sub_1 = nh.subscribe<geometry_msgs::WrenchStamped>
            ("/uav1/rotor_1_ft", 3, force_1_cb);
    ros::Subscriber force_sub_2 = nh.subscribe<geometry_msgs::WrenchStamped>
            ("/uav1/rotor_2_ft", 3, force_2_cb);
    ros::Subscriber force_sub_3 = nh.subscribe<geometry_msgs::WrenchStamped>
            ("/uav1/rotor_3_ft", 3, force_3_cb);

    ros::Rate loop_rate(30);


//----------drone arm length-------------
    double l = 0.25;
    double k = 0.02;
//----------least square ------------------
    Least_square least_square;
    std::random_device rd;
    std::default_random_engine generator = std::default_random_engine(rd());
    std::normal_distribution<double> distribution(0.0,2.0);
    Eigen::VectorXd output;
    output.setZero(4,1);
//----------low pass filter parameter ----------------
    double SAMPLE_RATE = 150;
    double CUTOFF = 50;
    double ooutput = 0 , last_output = 0;
//----------recursuve least square ------------------
    rls num_rls;
    Eigen::VectorXd theta;
    theta.setZero(4,1);

//----------use tau = inertia * alpha----------------
    Eigen::Matrix3d J;
    Eigen::Vector3d last_angular,now_angular,alpha_truth,torque_truth;
//----------delay offset ----------------------------
    double prev_torque[10] ;
    double prev_value[10][4];
//----------UKF parameter setting ------------------
    double measure_ex, measure_ey, measure_ez;
    double sum_pwm;

    Eigen::MatrixXd pnoise;
    pnoise.setZero(statesize,statesize);
    pnoise(p_x,p_x) = 1e-2;
    pnoise(p_y,p_y) = 1e-2;
    pnoise(p_z,p_z) = 1e-2;
    pnoise(v_x,v_x) = 1e-2;
    pnoise(v_y,v_y) = 1e-2;
    pnoise(v_z,v_z) = 1e-2;

    pnoise(e_x,e_x) = 0.005;//0.5,調小beta收斂較快
    pnoise(e_y,e_y) = 0.005;
    pnoise(e_z,e_z) = 0.005;

    pnoise(omega_x,omega_x) = 1e-2;
    pnoise(omega_y,omega_y) = 1e-2;
    pnoise(omega_z,omega_z) = 1e-2;

    pnoise(tau_x,tau_x) = 0.05;
    pnoise(tau_y,tau_y) = 0.05;
    pnoise(tau_z,tau_z) = 0.05;

    pnoise(beta_x,beta_x) = 0.05;//調大beta會無法收斂
    pnoise(beta_y,beta_y) = 0.05;
    pnoise(beta_z,beta_z) = 0.05;

    forceest1.set_process_noise(pnoise);

    Eigen::MatrixXd mnoise;
    mnoise.setZero(measurementsize,measurementsize);
    mnoise = 3e-3*Eigen::MatrixXd::Identity(measurementsize , measurementsize);

    mnoise(mp_x,mp_x) = 1e-4;
    mnoise(mp_y,mp_y) = 1e-4;
    mnoise(mp_z,mp_z) = 1e-4;

    mnoise(mv_x,mv_x) = 1e-2;
    mnoise(mv_y,mv_y) = 1e-2;
    mnoise(mv_z,mv_z) = 1e-2;

    mnoise(momega_x,momega_x) = 1e-2;
    mnoise(momega_y,momega_y) = 1e-2;
    mnoise(momega_z,momega_z) = 1e-2;

    mnoise(me_x,me_x) = 1;//1
    mnoise(me_y,me_y) = 1;
    mnoise(me_z,me_z) = 1;

    forceest1.set_measurement_noise(mnoise);

    Eigen::MatrixXd measurement_matrix;
    measurement_matrix.setZero(measurementsize,statesize);

    measurement_matrix(mp_x,p_x) = 1;
    measurement_matrix(mp_y,p_y) = 1;
    measurement_matrix(mp_z,p_z) = 1;

    measurement_matrix(mv_x,v_x) = 1;
    measurement_matrix(mv_y,v_y) = 1;
    measurement_matrix(mv_z,v_z) = 1;

    measurement_matrix(momega_x,omega_x) = 1;
    measurement_matrix(momega_y,omega_y) = 1;
    measurement_matrix(momega_z,omega_z) = 1;

    measurement_matrix(me_x,e_x) = 3.5;//1,調小，beta會劇烈震盪
    measurement_matrix(me_y,e_y) = 3.5;
    measurement_matrix(me_z,e_z) = 3.5;

    forceest1.set_measurement_matrix(measurement_matrix);
//-------------UKF end---------------------------------



    while(ros::ok()){

        double F1, F2, F3, F4;
        double pwm1, pwm2, pwm3, pwm4;
        //double U_x, U_y, U_z;
        Eigen::MatrixXd theta_q(4,3), phi_q(4,3);
        Eigen::Matrix3d A_q;
        Eigen::Vector3d y_k;
        Eigen::Vector3d mag_v;


        pose.x = drone2_pose.pose.position.x;
        pose_pub.publish(pose);


        if(drone2_imu.angular_velocity.x!=0 && drone2_pose.pose.position.x !=0 && drone2_vel.twist.linear.x !=0)
        {
            if(rc_out.channels.size()!=0 && rc_out.channels[0] != 0){
                pwm1 = rc_out.channels[0];
                pwm2 = rc_out.channels[1];
                pwm3 = rc_out.channels[2];
                pwm4 = rc_out.channels[3];
            }

            F1 = point_2(2);
            F2 = point_0(2);
            F3 = point_3(2);
            F4 = point_1(2);
            forceest1.thrust = F1 + F2 + F3 + F4;
            torque_ground.x = l*(F1 - F2 - F3 + F4);
            torque_ground.y = l*(-F1 - F2 + F3 + F4);
            //U_y = -0.35*F1 - 0.15*F2 + 0.45*F3 + 0.25*F4;
            torque_ground.z = k*F1 - k*F2 + k*F3 - k*F4;

//-----------MRP ---------------------------------------------------
            double x = drone2_pose.pose.orientation.x;
            double y = drone2_pose.pose.orientation.y;
            double z = drone2_pose.pose.orientation.z;
            double w = drone2_pose.pose.orientation.w;
            //pwm_pub.publish(torque_ground);
            forceest1.R_IB.setZero();
            forceest1.R_IB << w*w+x*x-y*y-z*z  , 2*x*y-2*w*z        ,  2*x*z+2*w*y,
                              2*x*y +2*w*z     , w*w-x*x+y*y-z*z    ,  2*y*z-2*w*x,
                              2*x*z -2*w*y     , 2*y*z+2*w*x        ,  w*w-x*x-y*y+z*z;
            forceest1.angular_v_measure << drone2_imu.angular_velocity.x, drone2_imu.angular_velocity.y, drone2_imu.angular_velocity.z;

            forceest1.predict();

            Eigen::VectorXd measure;
            measure.setZero(measurementsize);


            measure << drone2_pose.pose.position.x, drone2_pose.pose.position.y, drone2_pose.pose.position.z,
                    drone2_vel.twist.linear.x, drone2_vel.twist.linear.y, drone2_vel.twist.linear.z,
                    measure_ex, measure_ey, measure_ez,
                    drone2_imu.angular_velocity.x, drone2_imu.angular_velocity.y, drone2_imu.angular_velocity.z;
            forceest1.omega_bias(0) = gyro_bias.x;//vio gyro
            forceest1.omega_bias(1) = gyro_bias.y;
            forceest1.omega_bias(2) = gyro_bias.z;
            forceest1.quat_m << drone2_pose.pose.orientation.x, drone2_pose.pose.orientation.y, drone2_pose.pose.orientation.z, drone2_pose.pose.orientation.w;
            forceest1.qk11 = forceest1.qk1;

            forceest1.correct(measure);
            forceest1.x[e_x] = 0;
            forceest1.x[e_y] = 0;
            forceest1.x[e_z] = 0;

            bias.x = forceest1.x[beta_x];
            bias.y = forceest1.x[beta_y];
            bias.z = forceest1.x[beta_z];


            bias_pub.publish(bias);

            euler.x = forceest1.euler_angle(0);//roll:forceest1.euler_angle(0)
            euler.y = forceest1.euler_angle(1);//pitch:forceest1.euler_angle(1)
            euler.z = forceest1.euler_angle(2);//yaw:forceest1.euler_angle(2)
            euler_pub.publish(euler);

            angular_v.x = drone2_imu.angular_velocity.x;
            angular_v.y = drone2_imu.angular_velocity.y;
            angular_v.z = drone2_imu.angular_velocity.z;

            angular_v_pub.publish(angular_v);
            tf::Quaternion quat_transform_ref(drone2_pose.pose.orientation.x,drone2_pose.pose.orientation.y,drone2_pose.pose.orientation.z,drone2_pose.pose.orientation.w);
            double roll_ref,pitch_ref,yaw_ref;
            tf::Matrix3x3(quat_transform_ref).getRPY(roll_ref,pitch_ref,yaw_ref);
            euler_ref.x = roll_ref*180/3.1415926;//roll_ref*180/3.1415926
            euler_ref.y = pitch_ref*180/3.1415926;//pitch_ref*180/3.1415926
            euler_ref.z = yaw_ref*180/3.1415926;//yaw_ref*180/3.1415926
            euler_ref_pub.publish(euler_ref);


//------------use rotation dynamics to calculate torque-----------------
            double delta_t = 0.0333333;
            now_angular(0) = angular_v.x;
            now_angular(1) = angular_v.y;
            now_angular(2) = angular_v.z;
            alpha_truth(0) = (now_angular(0) - last_angular(0))/delta_t;
            alpha_truth(1) = (now_angular(1) - last_angular(1))/delta_t;
            alpha_truth(2) = (now_angular(2) - last_angular(2))/delta_t;
            J << 0.03 , 0   , 0,  //0.0625,0.0625,0.12656
                 0    , 0.05, 0,
                 0    , 0   , 0.1;
            torque_truth = J * alpha_truth + now_angular.cross(J*now_angular);
            last_angular(0) =angular_v.x ;
            last_angular(1) =angular_v.y ;
            last_angular(2) =angular_v.z ;




            body_torque.x = drone2_pose.pose.position.x;
            //body_torque.y = forceest1.x[tau_y];
            //body_torque.y = torque_truth(1);
            body_torque.z = forceest1.x[tau_z];


/*
            for(int i = 0; i < 4 ; i++){

                if(i == 3){
                    prev_torque[3] = body_torque.x;
                    //prev_torque[3] = forceest1.x[tau_y];
                }
                else{
                    prev_torque[i] = prev_torque[i+1];
                }

            }
            body_torque.y = prev_torque[0];
            */
//------------------shift force ----------------------
            for(int i = 0; i < 3 ; i++){
                    if(i == 2){
                        prev_value[2][0] = F1;
                        prev_value[2][1] = F2;
                        prev_value[2][2] = F3;
                        prev_value[2][3] = F4;
                    }
                    else{
                        prev_value[i][0] = prev_value[i+1][0];
                        prev_value[i][1] = prev_value[i+1][1];
                        prev_value[i][2] = prev_value[i+1][2];
                        prev_value[i][3] = prev_value[i+1][3];
                    }
            }


            //----------least square--------------//
            /*
            Eigen::MatrixVd input_force;
            input_force.setZero(1,4);
            input_force(0,0) = F1;
            input_force(0,1) = F2;
            input_force(0,2) = F3;
            input_force(0,3) = F4;
*/
            //rls num_rls;
            //Eigen::VectorXd theta;
            //theta.setZero(4,1);
            //double yy = ;
            //theta = num_rls.update( F1,F2,F3,F4, prev_torque[0] );
            //theta = num_rls.update( prev_value[0][0],prev_value[0][1],prev_value[0][2],prev_value[0][3], forceest1.x[tau_y] );
            theta = num_rls.update( F1,F2,F3,F4, forceest1.x[tau_y]);

            //ROS_INFO("RLS  %lf ; %lf ; %lf ; %lf",theta(0), theta(1), theta(2),theta(3));
            ROS_INFO("RLS  %lf ; %lf ; %lf ; %lf",theta(0), theta(1), theta(2),theta(3) );

            double noise = distribution(generator);
            double RC = 1.0/(CUTOFF*2*3.14);
            double dt = 1.0/SAMPLE_RATE;
            double alpha = dt/(RC+dt);
            //double input = torque_ground.y+0.05*noise;
            //double input = torque_truth(1);
            double input = forceest1.x[tau_y];
            ooutput = last_output + (alpha*(input - last_output));
            last_output = ooutput;
            //body_torque.y = last_output;
            //body_torque.y = forceest1.x[tau_y];
            //body_torque.y = prev_torque[0];
            body_torque.y = forceest1.x[p_x];

            pwm_pub.publish(body_torque);
            //if(torque_ground.y > 0.1){
                Eigen::VectorXd input_force;
                input_force.setZero(4,1);
                input_force(0) = F1;
                input_force(1) = F2;
                input_force(2) = F3;
                input_force(3) = F4;
                //output = least_square.update(input_force ,last_output);
                //output = least_square.update(input_force ,forceest1.x[tau_y]);
            //}


            //ROS_INFO("least_square  %lf ; %lf ; %lf ; %lf", output(0), output(1), output(2),output(3));
            //ROS_INFO("l  %5lf ; %5lf ; %5lf ; %5lf", point_0(2), point_1(2), point_2(2), point_3(2));
            least.x = output(0);
            least_square_pub.publish(least);
            //std::cout << " X "  << least_square.update(input_force,forceest1.x[tau_x]) <<std::endl;
        }
        loop_rate.sleep();
        ros::spinOnce();

    }
}
