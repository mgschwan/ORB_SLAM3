/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
*   José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
*   University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation, either version 3 of the License, or (at your option) any later version.
*/

#include <iostream>
#include <string>
#include <thread>
#include <signal.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>

#include <System.h>
#include <sophus/se3.hpp>

#include "localization_service/config.h"
#include "localization_service/slam_state.h"
#include "localization_service/calibration_manager.h"
#include "localization_service/web_server.h"

using namespace localization_service;

// Global flag set by SIGINT so the main loop can exit cleanly.
static LifecycleFlags* g_flags = nullptr;

static void sigintHandler(int)
{
    if (g_flags) g_flags->running = false;
}

// ---- Stdin command handler -------------------------------------------------
// Runs on the control thread alongside the HTTP server so that console
// commands and web requests share the same SLAM state.
static void handleStdinCommand(const std::string&  command,
                                ORB_SLAM3::System&  slam,
                                LifecycleFlags&     flags,
                                std::atomic<bool>&  localizationMode,
                                long unsigned int   mapId)
{
    if (command == "loc" || command == "localize") {
        if (!localizationMode) {
            localizationMode = true;
            slam.GetAtlas()->SwitchToMap(mapId);
            slam.ActivateLocalizationMode();
            slam.ForceRelocalization();
            std::cout << ">>> Switched to Localization Mode <<<\n";
        }
    } else if (command == "map" || command == "mapping") {
        if (localizationMode) {
            localizationMode = false;
            slam.DeactivateLocalizationMode();
            std::cout << ">>> Switched to Mapping Mode <<<\n";
        }
    } else if (command == "pause") {
        flags.paused = true;
        std::cout << ">>> Paused Processing <<<\n";
    } else if (command == "resume") {
        flags.paused = false;
        std::cout << ">>> Resumed Processing <<<\n";
    } else if (command == "quit" || command == "exit") {
        flags.running = false;
    }
}

// ---- Camera open -----------------------------------------------------------
static cv::VideoCapture openCamera(const std::string& source)
{
    cv::VideoCapture cap;

    bool isNumber = !source.empty();
    for (char c : source)
        if (!isdigit(c)) { isNumber = false; break; }

    if (isNumber)
        cap.open(std::stoi(source), cv::CAP_V4L2);
    else if (source.rfind("/dev/video", 0) == 0)
        cap.open(source, cv::CAP_V4L2);
    else
        cap.open(source);

    return cap;
}

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::cerr << "\nUsage: ./localization_service_host"
                     " path_to_vocabulary path_to_settings camera_url"
                     " [localize_only] [map_id]\n\n";
        return 0; // appimage builder needs to be able to call it without an error code
    }

    const std::string vocabPath    = argv[1];
    const std::string settingsPath = argv[2];
    const std::string cameraSource = argv[3];
    const bool        startInLocMode = (argc >= 5);
    const long unsigned int mapId  = (argc >= 6) ? std::stoul(argv[5]) : 0;

    cv::VideoCapture cap = openCamera(cameraSource);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera: " << cameraSource << "\n";
        return 1;
    }

    std::cout << "Initializing ORB-SLAM3\n";
    ORB_SLAM3::System slam(vocabPath, settingsPath, ORB_SLAM3::System::MONOCULAR, true);
    ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_DEBUG);
    const float imageScale = slam.GetImageScale();
    std::cout << "Image scale: " << imageScale << "\n";

    // Shared state
    LifecycleFlags     flags;
    PoseState          pose;
    std::atomic<bool>  localizationMode{false};

    if (startInLocMode) {
        std::cout << "Activating localization mode\n";
        localizationMode = true;
        slam.GetAtlas()->SwitchToMap(static_cast<unsigned int>(mapId));
        slam.ActivateLocalizationMode();
        slam.ForceRelocalization();
    }

    // Install SIGINT handler
    g_flags = &flags;
    signal(SIGINT, sigintHandler);

    // When running as an AppImage, APPDIR is set and the html folder lives there.
    const char* appDir = std::getenv("APPDIR");
    const std::string staticFileRoot = appDir
        ? std::string(appDir) + "/" + std::string(kStaticFileRoot)
        : std::string(kStaticFileRoot);

    // Components
    CalibrationManager calib(slam);
    WebServer          server(slam, flags, pose, calib, localizationMode, mapId, staticFileRoot);

    // Control thread: HTTP server + stdin commands
    std::thread controlThread([&]() {
        // Run the HTTP server in this thread; handle stdin via select() alongside
        // the server socket.  We call server.run() which blocks, but we need stdin
        // too — so we run the accept loop ourselves via a thin wrapper that also
        // monitors STDIN_FILENO.
        //
        // For simplicity: dedicate the control thread fully to WebServer::run()
        // and read stdin in a second lightweight thread.
        std::thread stdinThread([&]() {
            while (flags.running) {
                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(STDIN_FILENO, &fds);
                struct timeval tv = {1, 0};
                if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0
                    && FD_ISSET(STDIN_FILENO, &fds)) {
                    std::string cmd;
                    if (std::cin >> cmd)
                        handleStdinCommand(cmd, slam, flags, localizationMode, mapId);
                }
            }
        });

        server.run(); // blocks until flags.running == false

        stdinThread.join();
    });

    // ---- Main tracking loop ------------------------------------------------
    std::cout << "\n-------\nStart processing sequence ...\n";
    cv::Mat frame;

    while (flags.running) {
#ifdef COMPILEDWITHC11
        auto now   = std::chrono::steady_clock::now();
#else
        auto now   = std::chrono::monotonic_clock::now();
#endif
        double tframe = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now.time_since_epoch()).count();

        // Always drain the capture buffer so we get the freshest frame
        cap.read(frame);
        if (frame.empty()) {
            std::cout << "No image received\n";
            usleep(kPauseSleepUs);
            continue;
        }

        if (imageScale != 1.f) {
            cv::resize(frame, frame,
                       cv::Size(static_cast<int>(frame.cols * imageScale),
                                static_cast<int>(frame.rows * imageScale)));
        }

        // Calibration mode: process frame, skip SLAM tracking
        if (calib.processFrame(frame)) {
            usleep(kCalibSleepUs);
            continue;
        }

        if (flags.paused) {
            usleep(kPauseSleepUs);
            continue;
        }

        // Track
        Sophus::SE3f Tcw = slam.TrackMonocular(frame, tframe);

        const int trackState = slam.GetTrackingState();
        if (trackState == ORB_SLAM3::Tracking::OK
         || trackState == ORB_SLAM3::Tracking::RECENTLY_LOST) {
            Sophus::SE3f     Twc = Tcw.inverse();
            Eigen::Vector3f  t   = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            pose.update(t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
            std::cout << "Position " << Tcw.translation().transpose()
                      << "  Rotation " << Tcw.angleX() << ","
                                       << Tcw.angleY() << ","
                                       << Tcw.angleZ() << "\n";
        } else {
            pose.invalidate();
        }
    }

    // ---- Shutdown ----------------------------------------------------------
    std::cout << "Main loop ended. Shutting down...\n";
    flags.running = false;
    controlThread.join();

    slam.Shutdown();
    slam.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
