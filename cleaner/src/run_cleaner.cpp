#include "Cleaner.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cleaner");

    Cleaner cleaner;

    cleaner.startMoving();

    return 0;
};



