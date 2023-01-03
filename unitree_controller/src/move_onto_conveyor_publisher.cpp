#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <string>
#include <stdio.h>  
#include <tf/transform_datatypes.h>
// #include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>
#include <sensor_msgs/Joy.h>

float roll_ang = 0.0;
float pitch_ang = 0.0;
float yaw_ang = -3.142 * 0.5;
float bod_height_ros = 0.0;
float bod_height_ros_max = 0.5;
float bod_height_ros_min = -0.4;

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg) {

  // std::cout << "ha" <<std::endl;
  // bod_height_ros += joy_msg->axes[1]*0.01;
  // if (bod_height_ros >= bod_height_ros_max) {
  //   bod_height_ros = bod_height_ros_max;  
  // }
  // if (bod_height_ros <= bod_height_ros_min) {
  //   bod_height_ros = bod_height_ros_min;  
  // }
  // roll_ang = joy_msg->axes[3]*30/180.0*M_PI;
  // pitch_ang = joy_msg->axes[4]*30/180.0*M_PI;
  // yaw_ang = joy_msg->axes[0]*30/180.0*M_PI;
  return;
}

int main(int argc, char **argv)
{
    enum coord
    {
        WORLD,
        ROBOT
    };
    coord def_frame = coord::WORLD;

    ros::init(argc, argv, "move_publisher");
    ros::NodeHandle nh;
    ros::Publisher move_publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
    ros::Subscriber joy_stick_sub = nh.subscribe("/joy", 1000, joy_callback);

    gazebo_msgs::ModelState model_state_pub;

    std::string robot_name;
    ros::param::get("/robot_name", robot_name);
    std::cout << "robot_name: " << robot_name << std::endl;

    model_state_pub.model_name = robot_name + "_gazebo";
    ros::Rate loop_rate(500);

    if(def_frame == coord::WORLD)
    {
        model_state_pub.pose.position.x = 0.0;
        model_state_pub.pose.position.y = 0.0;
        model_state_pub.pose.position.z = 0.5;
        
        model_state_pub.pose.orientation.x = 0.0;
        model_state_pub.pose.orientation.y = 0.0;
        model_state_pub.pose.orientation.z = 0.0;
        model_state_pub.pose.orientation.w = 1.0;

        model_state_pub.reference_frame = "world";

        long long time_ms = 0;  //time, ms
        const double period = 5000; //ms
        const double radius = 1.5;    //m
        tf::Quaternion q;
        while(ros::ok())
        {
            // model_state_pub.pose.position.x = radius * sin(2*M_PI*(double)time_ms/period);
            // model_state_pub.pose.position.y = radius * cos(2*M_PI*(double)time_ms/period);
            model_state_pub.pose.position.z = 0.325 + 0.775 + bod_height_ros;
            // model_state_pub.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, - 2*M_PI*(double)time_ms/period);
            model_state_pub.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_ang, pitch_ang, yaw_ang);

            move_publisher.publish(model_state_pub);
            loop_rate.sleep();
            ros::spinOnce();
            time_ms += 1;
        }
    }
    else if(def_frame == coord::ROBOT)
    {
        model_state_pub.twist.linear.x= 0.02; //0.02: 2cm/sec
        model_state_pub.twist.linear.y= 0.0;
        model_state_pub.twist.linear.z= 0.08;
        
        model_state_pub.twist.angular.x= 0.0;
        model_state_pub.twist.angular.y= 0.0;
        model_state_pub.twist.angular.z= 0.0;

        model_state_pub.reference_frame = "base";

        while(ros::ok())
        {
            move_publisher.publish(model_state_pub);
            loop_rate.sleep();
        }
    }
    
}