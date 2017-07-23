#ifndef ROBOT_H
#define ROBOT_H

#include <thread> // std::thread

#include "IterativeRobot.h" // IterativeRobot
#include "CameraServer.h" // CameraServer

class Robot: public IterativeRobot {
    public:
        Robot();
        ~Robot();

        void RobotInit();
        void TeleopInit();
        void AutonomousInit();
        void DisabledInit();

        void TeleopPeriodic();
        void AutonomousPeriodic();
        void DisabledPeriodic();

    private:
        CameraServer* m_server;
        std::thread m_vision_thread;
};

#endif
