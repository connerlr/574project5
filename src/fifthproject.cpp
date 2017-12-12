#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <angles/angles.h>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>
#include "nav_msgs/OccupancyGrid.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <time.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class fifthproject
{
	public:
	double odomX, odomY, odomHeading;
	ros::Publisher pub;
	ros::Subscriber gmap, odomSub;	
	fifthproject(ros::NodeHandle nh)
	{
		odomHeading = 0;//huge number fix
		odomX = 0;
		odomY = 0;
		pub = nh.advertise<geometry_msgs::Twist>
			("cmd_vel", 100);
		odomSub = nh.subscribe("odom", 
			10, &fifthproject::odomCallBack, this);
		gmap = nh.subscribe("map", 
			10, &fifthproject::gmapCallBack, this);
	}
	void gmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
	{


	}

	void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
	{
		odomX = -msg->pose.pose.position.y;
		odomY = msg->pose.pose.position.x;
		odomHeading = tf::getYaw(msg->pose.pose.orientation);
			
	}

	
	void explore(MoveBaseClient ac)
	{
		move_base_msgs::MoveBaseGoal goal;

		goal.target_pose.header.frame_id = "base_link";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = 1.0;
		goal.target_pose.pose.orientation.w = 1.0;

		ac.sendGoal(goal);

		ac.waitForResult();
		
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{	
			ROS_INFO_STREAM("HOORAY???");
		}
		else
		{
			ROS_INFO_STREAM("AWW??");
		}
	}

void translate(double d)
	{
		geometry_msgs::Twist msg;


		//Set the movement command rotation speed
		msg.linear.x = 0.4;
		msg.angular.z = 0.0;
		// Current angle
		double last_x;
		double last_y;
		int count = 0;
		double thisD = 0;
		ros::Rate rate(50.0);

		while (ros::ok()) 
		{
			//ROS_INFO("odomX %f", odomX);
			//if((odomX + odomY) != 0.0)
			{
				if(count == 0)
				{		
					ROS_INFO_STREAM("UNDER THIS IS BEFORE WHILE LOOP");
					ROS_INFO("odomX %f", odomX);
					ROS_INFO("odomY%f", odomY);
					ROS_INFO_STREAM("ABOVE THIS IS BEFORE WHILE LOOP");
					count++;
					last_x = odomX;
					last_y = odomY;
				}
				if(thisD >= d)
				{
					
					msg.linear.x = 0.1;
					
					pub.publish(msg);
					break;
				}
				else if (d - thisD < 0.4)
				{
					msg.linear.x = 0.2;
				}
				else if (d - thisD < 0.15)
				{
					msg.linear.x = 0.1;
				}
				else
				{
					msg.linear.x = 0.4;
				}

				msg.angular.z = 0.0;

				pub.publish(msg);

				thisD = sqrt(pow((odomX - last_x), 2) + pow((odomY - last_y), 2));
			}	
		ros::spinOnce();
		rate.sleep();
		}
		ROS_INFO_STREAM("UNDER THIS IS AFTER WHILE LOOP");
		ROS_INFO("odomX %f", odomX);
		ROS_INFO("odomY%f", odomY);
		ROS_INFO_STREAM("ABOVE THIS IS AFTER WHILE LOOP");

		//Robot stops moving
		msg.linear.x = 0.0;
		pub.publish(msg);
	}


	void moveForward()
	{
		while(ros::ok())
		{
			geometry_msgs::Twist msg;
			msg.angular.z = 0.5;
			msg.linear.x = 0.5;
			pub.publish(msg);
		}


	}
};


int main(int argc, char **argv)

{
	ROS_INFO_STREAM("Enters Main");
	ros::init(argc, argv, "fifthproject");
	ros::NodeHandle nh;
	fifthproject *proj = new fifthproject(nh);

	MoveBaseClient ac("move_base", true);
	proj->moveForward();
	proj->explore(ac);
	ros::shutdown();

}
