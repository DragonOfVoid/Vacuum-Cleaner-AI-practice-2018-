#include "Cleaner.h"
#include "geometry_msgs/Twist.h"

using namespace ros;

Cleaner::Cleaner()
{
    keepMoving = true;

    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    laserSub = node.subscribe("scan", 1, &Cleaner::scanCallback, this);
}

void Cleaner::moveForward() {
    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED;
    commandPub.publish(msg);
}


void Cleaner::turnRight() {
    geometry_msgs::Twist msg;
    msg.angular.z = -TURNING_SPEED;
    commandPub.publish(msg);
}

void Cleaner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
bool isObstacleInFront = false;

    int minIndex = ceil((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
        if (scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE) {
        	isObstacleInFront = true;
            break;
        }
    }

    if (isObstacleInFront) {
        keepMoving = false;
    }
}

void Cleaner::startMoving()
{
    Rate rate(10);
    ROS_INFO("Start moving");

    while (ok()) {
	while(!keepMoving)
		turnRight();
        moveForward();
        ros::spinOnce();
        rate.sleep();
    }
}

