/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F "

int get_char()
{
    fd_set rfds;
    struct timeval tv;
    char ch = ' ';
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10; //设置等待超时时间
    if (select(1, &rfds, NULL, NULL, &tv) > 0) //检测键盘是否有输入
    {
        ch = getchar(); 
    }
    return ch;
}


KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    // tcgetattr( fileno( stdin ), &_oldSettings );
    // _newSettings = _oldSettings;
    // _newSettings.c_lflag &= (~ICANON & ~ECHO);
    // tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::L2_B;
    case '2':
        return UserCommand::L2_A;
    case '3':
        return UserCommand::L2_X;
    case '4':
        return UserCommand::START;
#ifdef COMPILE_WITH_MOVE_BASE
    case '5':
        return UserCommand::L2_Y;
#endif  // COMPILE_WITH_MOVE_BASE
    case '0':
        return UserCommand::L1_X;
    case '9':
        return UserCommand::L1_A;
    case '8':
        return UserCommand::L1_Y;
    case ' ':
        userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0);
        break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        break;

    case 'i':case 'I':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        break;
    case 'l':case 'L':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}

void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

void* KeyBoard::run(void *arg){
    system(STTY_US TTY_PATH);
    
    while(1){
        char c = get_char();
        switch (c) {
        case 's':
            userCmd = UserCommand::L2_A;
            break;
        case 'w':
            userCmd = UserCommand::START;
            break;
        case 'c':
            exit(0);
            break;
        case 'i':
            userValue.lx = 0.0;
            userValue.ly = 1.0;
            userValue.rx = 0.0;
            break;
        case 'j':
            userValue.lx = 0.0;
            userValue.ly = 0.0;
            userValue.rx = -1.0;
            break;
        case 'l':
            userValue.lx = 0.0;
            userValue.ly = 0.0;
            userValue.rx = 1.0;
            break;
        case 'h':
            userValue.lx = 0.0;
            userValue.ly = 0.0;
            userValue.rx = 0.0;
            break;
        default:
            break;
        }
        
        usleep(1000);
    }

    return NULL;
}