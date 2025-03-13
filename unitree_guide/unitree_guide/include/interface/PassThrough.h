/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "interface/CmdPanel.h"
#include "common/mathTools.h"

class PassThrough : public CmdPanel{
public:
    PassThrough(ros::NodeHandle& _nm);
    ~PassThrough();
private:
    static void* runPassThrough(void *arg);
    void* run(void *arg);
    void CmdCallback(const sensor_msgs::Imu & msg);

    pthread_t _tid;
    ros::Subscriber _v_cmd;
};