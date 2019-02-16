#include "Cleaner.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <math.h>

using namespace ros;

Cleaner::Cleaner()
{
    Cleaner::state = 0;

    Cleaner::commandPub = Cleaner::node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    Cleaner::laserSub = Cleaner::node.subscribe("scan", 1, &Cleaner::scanCallback, this);
    Cleaner::poseSub = Cleaner::node.subscribe("odom", 100, &Cleaner::poseCallback, this);
}

void Cleaner::moveForward() 
{
    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED;
    commandPub.publish(msg);
}


void Cleaner::turn(int side) 
{
    geometry_msgs::Twist msg;
    msg.angular.z = side*TURNING_SPEED;
    commandPub.publish(msg);
}

void Cleaner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    bool isObstacleInFront = false;

    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++)
        if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE)
	{        	
	    isObstacleInFront = true;
            break;
	}
	
    if(Cleaner::state==1 & !isObstacleInFront)
	Cleaner::state=2;
    if(isObstacleInFront)
	Cleaner::state=1;

}

void Cleaner::poseCallback(const nav_msgs::Odometry::ConstPtr& pose)
{
    Cleaner::yaw=tf::getYaw(pose->pose.pose.orientation);
    if(Cleaner::yaw<0)
	Cleaner::yaw=-Cleaner::yaw;
}

void Cleaner::startMoving()
{
    Rate rate(10);
    ROS_INFO("Start moving");
    int count, dir = -1;
    while (ok()) {
	switch(state)
	{
	    case 0:
		moveForward();
	    break;
	    case 1:
		turn(dir);
		count=0;
	    break;
	    case 2:
		moveForward();
		count++;
		if(count>20)
		    Cleaner::state=3;
	    break;
	    case 3:
		turn(dir);
	    	if(Cleaner::yaw<0.1 | Cleaner::yaw>M_PI-0.2)
		    {Cleaner::state=0; dir = - dir;}
	    break;
	}
        spinOnce();
        rate.sleep();
    }
}

