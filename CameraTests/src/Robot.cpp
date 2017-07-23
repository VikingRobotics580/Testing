/**
 * @file Robot.cpp
 * @author Tyler Robbins
 * @brief A simple robot program for testing axis cameras
 * @details
 * WPILib has two methods for working with camera and video, an old way in which
 *  one had to obtain a "ParticleAnalysisReport" and a new way using OpenCV.
 * This file is for testing the new OpenCV way, which is far easier to use and
 *  deal with. I highly recommend using it.
 * How it works:
 * ============
 * WPILib provides a framework around the OpenCV interface, configuring camera
 *  access, names, and IP addresses.
 * There is a central CameraServer running, which holds and manages all video
 *  feeds, both physical and virtual, that travel through the robot.
 * This CameraServer then needs an AxisCamera to be added to it so it knows to
 *  manage a video feed from the IP Address you give it.
 * In order to modify video, one also needs a place to display the video.
 * To do this, one simply creates a CvSource, and tells CameraServer that this
 *  is a new video feed, with  name you give it.
 * Next, one simply uses CvSink to get access to the video feed of the newly
 *  added AxisCamera, and then uses GrabFrame to get the next video frame from
 *  the camera.
 * Then, with that frame, you can modify it to your hearts content using
 *  OpenCV's image processing functions (found in opencv2/imgproc.hpp) before
 *  writing them to the CvSource that was created earlier. This pushes the
 *  modified frames back into the CameraServer, so they can be viewed again at
 *  the camera feed with the name you gave to the CvSource.
 *  
 * To view the modified camera feed in the Smart Dashboard, create or find a
 *  place for a camera feed to be
 * In the Java SmartDashboard:
 *  Create a Camera Server widget, and in its properties menu change the
 *   "Camera Choice" field to the name of the of the CvSource feed
 * In the Labview SmartDashboard:
 *  Underneath the Camera display (on the left side) find a dropdown menu and
 *   use it to select the name of the CvSource feed.
 *   
 * Note that for both of these methods, the general idea is the same: There is a
 *  central CameraServer which holds the camera feed for both the normal
 *  AxisCamera, and your altered feed.
 *  
 * This also opens up multiple possibilities, such as the ability to have
 *  multiple altered camera feeds, such as one which has boxes around other
 *  robots and another which has statistical information like distances, or
 *  maybe one could have two different video feeds side by side. It is all
 *  possible because the Image frames one recieves are simply mathematical
 *  Matricies (see opencv2/core/mat.hpp line 553 for details, and line 740 for
 *  the declaration).
 * 
 * Sources:
 * =======
 * https://robotdotnet.github.io/tutorials/opencv.html
 *  -> C# code, but it's similar enough that one can easily translate it to C++
 *  -> Best tutorial, since it has a working implementation and explains what
 *     everything does.
 * http://docs.opencv.org/trunk/index.html
 * https://wpilib.screenstepslive.com/s/4485/m/24194
 * https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/athena/src/CameraServer.cpp
 *  -> WPILib's C++ implementation of CameraServer
 * http://first.wpi.edu/FRC/roborio/release/docs/cpp/classfrc_1_1CameraServer.html
 *  -> Documentation on WPILib's CameraServer class.
 * https://wpilib.screenstepslive.com/s/4485/m/24194/l/682778-read-and-process-video-cameraserver-class
 *  -> Introduction to how vision processing works
 * https://wpilib.screenstepslive.com/s/4485/m/24194/l/669166-using-the-camera-server-on-the-roborio-2017
 *  -> Tutorial on how to use the UsbCamera class with the CameraServer class.
 */

#include "Robot.h"

#include <iostream> // std::cout, std::endl
#include <thread>   // std::thread

// For all processing functions that OpenCV gives
#include "opencv2/imgproc.hpp" // cv::rectangle

// For Mat(rix)
#include "opencv2/core/mat.hpp" // cv::Mat
#include "cscore_oo.h" // cs::CvSink, cs::CvSource
#include "WPILib.h" // START_ROBOT_CLASS

//! The date and time this code was compiled
#define DATETIME __DATE__ " - " __TIME__

//! The IP address of the Axis Camera
#define CAM_IP "10.5.80.200"

//! Write the current date and time this code was compiled to the console while
//!  compiling
#pragma message "Latest Build: " DATETIME

/**
 * @brief Constructs a new Robot.
 */
Robot::Robot(): m_server(CameraServer::GetInstance())
{
    // Simple print to check if the latest code was deployed properly
	std::cout << "Build from " << DATETIME << std::endl;
}

/**
 * @brief Destroys the Robot.
 */
Robot::~Robot() {
}

/**
 * @brief Initializes the robot.
 */
void Robot::RobotInit() {
    // Create a new thread to execute a lambda
    m_vision_thread = std::thread([this]() {
        // Only use this line if you have a USB camera (Axis cameras are
        //  ethernet)
        // cs::UsbCamera camera = m_server->StartAutomaticCapture();

        // Add a new Camera for the server to watch at CAM_IP
        cs::AxisCamera camera = m_server->AddAxisCamera(CAM_IP);

        // Set the Camera's resolution to 640 x 480
        camera.SetResolution(640, 480);

        // This is the camera feed to get the latest "mats" (frames)
        //  from the camera
        cs::CvSink sink = m_server->GetVideo();

        // Start a new video feed called "Rectangle" of resolution 640x480.
        cs::CvSource source = m_server->PutVideo("Rectangle",
                                                 640, 480);

        // Where a camera frame is stored
        cv::Mat mat;

        while(true) {
            // Get the next frame from the camera feed and store it in mat
            if(sink.GrabFrame(mat) == 0) {
                // If an error occurred, print what it was and continue
                //  to the next frame
                source.NotifyError(sink.GetError());
                continue;
            }

            // Process the frame by placing a rectangle on it
            // See "opencv2/imgproc.hpp" or
            //  "http://docs.opencv.org/trunk/dd/d46/imgproc_8hpp.html" for a
            //  list of all functions that can be used
            // cv::rectangle(Image, Vertex1, Vertex2 [opposite of Vertex1],
            //               Color, Thickness [Default 1],
            //               Line Type [Default LINE_8], Shift [Default 0])
            cv::rectangle(mat,
                          cv::Point(100, 100),
                          cv::Point(400, 400),
                          cv::Scalar(255, 255, 255), 5);

            // Write the modified frame (the one we just scribbled all over with
            //  cv::rectangle) to the Rectangle video feed.
            source.PutFrame(mat);
        }
    });

    // Detach the thread so that it is out of the way
    m_vision_thread.detach();
}

/**
 * @brief Initializes Teleop mode.
 */
void Robot::TeleopInit() {
    
}

/**
 * @brief Initializes Autonomous mode.
 */
void Robot::AutonomousInit() {

}

/**
 * @brief Initializes Disabled mode.
 */
void Robot::DisabledInit() {

}

/**
 * @brief Code to perform during teleop mode.
 */
void Robot::TeleopPeriodic() {

}

/**
 * @brief Code to perform during autonomous mode.
 */
void Robot::AutonomousPeriodic() {

}

/**
 * @brief Code to perform during disabled mode.
 */
void Robot::DisabledPeriodic() {

}

// Begin the robot.
START_ROBOT_CLASS(Robot);
