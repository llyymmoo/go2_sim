/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/PassThrough.h"
#include <iostream>

PassThrough::PassThrough(ros::NodeHandle& _nm){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    _v_cmd = _nm.subscribe("/go2/velocity_cmd", 1, &PassThrough::CmdCallback, this);

    pthread_create(&_tid, NULL, runPassThrough, (void*)this);
}

PassThrough::~PassThrough(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
}

void* PassThrough::runPassThrough(void *arg){
    ((PassThrough*)arg)->run(NULL);
    return NULL;
}

void* PassThrough::run(void *arg){
    ros::Rate r(1000);
    
    while(ros::ok()){
        // TODO: mode change
        
        ros::spinOnce();
        r.sleep();
    }

    return NULL;
}

void PassThrough::CmdCallback(const sensor_msgs::Imu & msg)
{
    userValue.lx = msg.linear_acceleration.x;
    userValue.ly = msg.linear_acceleration.y;
    userValue.rx = msg.linear_acceleration.z;

    return;
}