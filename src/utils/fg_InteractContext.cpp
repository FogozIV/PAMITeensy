//
// Created by fogoz on 21/05/2025.
//
#include "utils/InteractContext.h"
#include "robot/BaseRobot.h"

const char * enterInteractContext(std::shared_ptr<BaseRobot> robot, Stream &stream, std::function<void()> idle) {
    stream.println("Use arrow key to move (if movement is not in the right direction please use the calib_motors command)");
    stream.println("Use space to stop robot movement");
    stream.println("Use q, Q, w, W to exit the interact");
    bool controlDisabled = robot->isControlDisabled();
    robot->setControlDisabled(true);
    StateMachine stateMachine;
    double leftPWM = 0;
    double rightPWM = 0;
    double maxPWMLeft = robot->getLeftMotor()->getMaxPWM();
    double maxPWMRight = robot->getRightMotor()->getMaxPWM();
    stateMachine.set(UP_LAST_CHAR, [&]{
        leftPWM += maxPWMLeft * 0.1;
        rightPWM += maxPWMRight * 0.1;
    });
    stateMachine.set(DOWN_LAST_CHAR, [&]{
        leftPWM -= maxPWMLeft * 0.1;
        rightPWM -= maxPWMRight * 0.1;
    });
    stateMachine.set(LEFT_LAST_CHAR, [&]{
        leftPWM -= maxPWMLeft * 0.1;
        rightPWM += maxPWMRight * 0.1;
    });
    stateMachine.set(RIGHT_LAST_CHAR, [&]{
        leftPWM += maxPWMLeft * 0.1;
        rightPWM -= maxPWMRight * 0.1;
    });
    while(true){
        while(stream.available()){
            char c = stream.read();
            if(c == 27){
                stateMachine.begin();
                continue;
            }
            if(stateMachine.isStarted()){
                stateMachine.append(c);
                if(stateMachine.isStarted()){
                    continue;
                }
            }
            if(c == 32){
                leftPWM = 0;
                rightPWM = 0;
            }

            if(c == 'w' || c=='W' || c=='q' || c=='Q'){
                robot->getLeftMotor()->setPWM(0);
                robot->getRightMotor()->setPWM(0);
                robot->setControlDisabled(controlDisabled);
                return "Thanks for using interact";
            }
            leftPWM = constrain(leftPWM, -maxPWMLeft, maxPWMLeft);
            rightPWM = constrain(rightPWM, -maxPWMRight, maxPWMRight);
            stream.printf("Left pwm : %f, Right pwm: %f \r\n", leftPWM, rightPWM);
            robot->getLeftMotor()->setPWM(leftPWM);
            robot->getRightMotor()->setPWM(rightPWM);
        }
        idle();
        Threads::yield();
    }
}
