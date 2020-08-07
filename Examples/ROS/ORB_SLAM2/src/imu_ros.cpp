
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
//********para of imu*******
Eigen::Vector3d tmp_P=Eigen::Vector3d(0, 0, 0); //t
Eigen::Quaterniond tmp_Q=Eigen::Quaterniond::Identity();//R
Eigen::Vector3d tmp_V=Eigen::Vector3d(0, 0, 0);
double g_last_imu_time = -1.0;
ros::Publisher g_imu_path_pub;
nav_msgs::Path g_imu_path;

using namespace std;
void ShowIMUPositionCallBack(Eigen::Vector3d imu_linear_acc, Eigen::Vector3d&  imu_angular_vel,clock_t t)
{
    std::cout<<imu_angular_vel(2)<<std::endl;
    double dt =(double) (t - g_last_imu_time)/CLOCKS_PER_SEC;  //CLOCKS_PER_SEC=1000000
    g_last_imu_time = t;
    tmp_V+=tmp_Q*imu_linear_acc*dt;

    //  Eigen::Quaterniond wheelR=Eigen::Quaterniond(1,0,0,0.5*wheel_angular_vel(2)*dt);

    //  Eigen::Vector3d wheelt= g_wheel_predicted_rot*wheel_linear_vel*dt;
    Eigen::Vector3d acc=  tmp_Q*imu_linear_acc;
    tmp_P = tmp_P + tmp_Q* tmp_V*dt+0.5*dt*dt*acc ;
    tmp_Q = tmp_Q * Eigen::Quaterniond(1, 0 , 0, 0.5*imu_angular_vel(2)*dt);
    cout<<"tmp_Q eular"<<(180/M_PI)*tmp_Q.matrix().eulerAngles(0,1,2)<<endl;

    //pub path
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x =tmp_P(0);
    this_pose_stamped.pose.position.y = tmp_P(1);
    this_pose_stamped.pose.position.z = tmp_P(2);

    this_pose_stamped.pose.orientation.x = tmp_Q.x();
    this_pose_stamped.pose.orientation.y = tmp_Q.y();
    this_pose_stamped.pose.orientation.z = tmp_Q.z();
    this_pose_stamped.pose.orientation.w = tmp_Q.w();
    this_pose_stamped.header.stamp= ros::Time::now();
    this_pose_stamped.header.frame_id="imu_path";
    std::cout<<" dt: " <<  dt <<"x="<<tmp_P(0)<<" y="<<tmp_P(1)<<" z="<<tmp_P(2)<<std::endl;

    g_imu_path.poses.push_back(this_pose_stamped);
    g_imu_path_pub.publish(g_imu_path);

}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath");
    std::cout<<"start benchmark"<<std::endl;
    ros::NodeHandle ph;
    g_imu_path_pub = ph.advertise<nav_msgs::Path>("imu_path",1, true);
    g_imu_path.header.frame_id="world";
    double imu_angular_vel_rz=0.1;
    double const_imu_linear_acc_x = 0.0;
    double const_imu_linear_acc_y =0.0;
    Eigen::Vector3d imu_angular_vel={ 0, 0, imu_angular_vel_rz};
    Eigen::Vector3d imu_linear_acc={const_imu_linear_acc_x, const_imu_linear_acc_y,0};

    clock_t current_time,init_time;
    init_time=clock();
    current_time=clock();

    bool init_flag=false;
    tmp_V={0.1,0,0};
    cv::RNG rng;                        // OpenCV随机数产生器
    while (1)
    {
        if(false==init_flag)
        {
            current_time=init_time;
            g_last_imu_time=current_time;
            init_flag=true;
        }else{
            current_time = clock();
        }
        //add noise
        // imu_linear_acc(0)+= rng.gaussian ( 0.0001);
        cout<<"imu_angular_vel"<<imu_angular_vel(2)<<std::endl;
        ShowIMUPositionCallBack( imu_linear_acc, imu_angular_vel,current_time);
        sleep(0.1);
    }

    return 0;
}

