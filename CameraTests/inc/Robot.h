/**
 * @file Robot.h
 * @author Tyler Robbins
 * @brief The header file for the Robot class.
 * @details See Robot.cpp for a more in depth description.
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <thread> // std::thread

#include "IterativeRobot.h" // IterativeRobot
#include "CameraServer.h" // CameraServer

/**
 * @brief Main Robot class.
 */
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
        //! The Camera Server
        CameraServer* m_server;
        //! The thread where the vision processing is performed.
        std::thread m_vision_thread;
};

#endif
