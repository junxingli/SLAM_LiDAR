#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <md49_messages/md49_data.h>
#include "math.h"

long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
ros::Time current_time, last_time;
double DistancePerCount = (3.14159265 * 0.125) / 980.0;  //D :diameter 125mm   p: line-number of encoder 980

double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
//double deltaLeft = 0;
//double deltaRight = 0;
double v_left = 0.0;
double v_right = 0.0;
double templ = -128.0;
double tempr = -128.0;


void WheelCallback(const md49_messages::md49_data& ticks)
{
	current_time = ros::Time::now();
	//extract the wheel velocities from the tick signals count
	//deltaLeft = ticks.encoder_l - _PreviousLeftEncoderCounts;
	//deltaRight = ticks.encoder_r - _PreviousRightEncoderCounts;
	
	 //v_left = (deltaLeft * DistancePerCount) / (current_time - last_time).toSec();
        //v_right = (deltaRight * DistancePerCount) / (current_time - last_time).toSec();
         if(abs(ticks.speed_l - templ)<30)
	{
         v_left= (ticks.speed_l+128)/141.0;
        templ= ticks.speed_l;
        }
	else
	{
	v_left = (templ+128)/141.0;
	}
        if(abs(ticks.speed_r - tempr)<30)
	{
         v_right=(ticks.speed_r+128)/141.0;
         tempr= ticks.speed_r;
	}
	else
	{
        v_right=(tempr+128)/141.0;
	}

        std::cout <<"v_left=" << v_left <<'\n'<< "templ=" << templ <<'\n';       
        std::cout <<"v_right=" << v_right <<'\n'<< "tempr=" << tempr <<'\n';
        
        
	vx = ((v_right + v_left) / 2);
	vy = 0; 
	vth=((v_right - v_left)/0.337); // angle speed Dw: distance between wheel 337mm
	//_PreviousLeftEncoderCounts = ticks.encoder_l;
	//_PreviousRightEncoderCounts = ticks.encoder_r;
	last_time = current_time;

}

int main(int argc, char **argv)

{

  	ros::init(argc, argv, "odometry_publisher");
  	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("md49_data", 100, WheelCallback);
  	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);  
	tf::TransformBroadcaster odom_broadcaster;
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(2.0);

	while(n.ok())
		{
		ros::spinOnce();  		// check for incoming messages
		current_time = ros::Time::now();
		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time - last_time).toSec();
    		double delta_x = vx * cos(th) * dt; //from base link to world
    		double delta_y = vx * sin(th) * dt;
    		double delta_th = vth * dt;

	    	x += delta_x;
    		y += delta_y;
    		th += delta_th;

 		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

 		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
    		odom_trans.header.stamp = current_time;
    		odom_trans.header.frame_id = "odom";
    		odom_trans.child_frame_id = "base_link";

    		odom_trans.transform.translation.x = x;
    		odom_trans.transform.translation.y = y;
    		odom_trans.transform.translation.z = 0.0;
    		odom_trans.transform.rotation = odom_quat;

    		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

 		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

 		//set the position
		odom.pose.pose.position.x = x;
    		odom.pose.pose.position.y = y;
    		odom.pose.pose.position.z = 0.0;
    		odom.pose.pose.orientation = odom_quat;

    		//set the velocity

    		odom.child_frame_id = "base_link";
    		odom.twist.twist.linear.x = vx;
    		odom.twist.twist.linear.y = vy;
    		odom.twist.twist.angular.z = vth;

    		//publish the message
    		odom_pub.publish(odom);
		//std::cout <<"x=" << x << '\n' << "y=" << y << '\n' << "th=" << th;

		last_time = current_time; 

    		r.sleep();

  		}

}
