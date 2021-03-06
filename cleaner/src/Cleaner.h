#ifndef WANDER_BOT_SRC_STOPPER_H_
#define WANDER_BOT_SRC_STOPPER_H_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


using namespace ros;

class Cleaner {
public:
    const static double FORWARD_SPEED = 0.3;
    const static double TURNING_SPEED = 60.0/180*M_PI;
    const static double MIN_SCAN_ANGLE = -15.0/180*M_PI;
    const static double MAX_SCAN_ANGLE = +15.0/180*M_PI;
    const static float MIN_DIST_FROM_OBSTACLE = 0.8;

    Cleaner();
    void startMoving();

private:
    NodeHandle node;
    Publisher commandPub;
    Subscriber laserSub;
    bool keepMoving;
    void moveForward();
    void turnRight();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif /* WANDER_BOT_SRC_STOPPER_H_ */
