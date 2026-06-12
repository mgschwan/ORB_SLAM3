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

#include "localization_service/args.h"
#include "localization_service/config.h"
#include "localization_service/slam_state.h"
#include "localization_service/ingest_queue.h"
#include "localization_service/espnode_source.h"
#include "localization_service/calibration_manager.h"
#include "localization_service/preprocessor.h"
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
                                std::atomic<bool>&  allowMapCreation,
                                std::atomic<bool>&  useMotionModel,
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
    } else if (command == "newmaps_on") {
        allowMapCreation = true;
        slam.SetAllowMapCreation(true);
        std::cout << ">>> Map creation on tracking loss: ENABLED <<<\n";
    } else if (command == "newmaps_off") {
        allowMapCreation = false;
        slam.SetAllowMapCreation(false);
        std::cout << ">>> Map creation on tracking loss: DISABLED <<<\n";
    } else if (command == "motion_on") {
        useMotionModel = true;
        slam.SetUseMotionModel(true);
        std::cout << ">>> Constant-velocity motion model: ENABLED <<<\n";
    } else if (command == "motion_off") {
        useMotionModel = false;
        slam.SetUseMotionModel(false);
        std::cout << ">>> Constant-velocity motion model: DISABLED <<<\n";
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
    ServiceArgs args;
    if (!parseArgs(argc, argv, args))
        return 0; // appimage builder needs a zero exit code on bad args

    const bool useEspnode = (args.cameraSource.rfind("espnode", 0) == 0);
    const bool useIngest  = (args.cameraSource == "none") || useEspnode;

    // ---- ESP32 node setup (discovery before SLAM init to fail fast) ----------
    ImuBuffer imuBuf;
    std::unique_ptr<EspnodeSource> espSrc;
    if (useEspnode) {
        std::string espIp;
        // "espnode:<ip>" → use provided IP; "espnode" alone → auto-discover
        if (args.cameraSource.size() > 8 && args.cameraSource[7] == ':')
            espIp = args.cameraSource.substr(8);
        espSrc = std::make_unique<EspnodeSource>(espIp, args.espnodeFps);
        if (espSrc->discover().empty()) {
            std::cerr << "Failed to discover ESP32 node. Exiting.\n";
            return 1;
        }
        std::cout << "ESP32 node at " << espSrc->ip()
                  << "  trigger fps=" << args.espnodeFps << "\n";
    }

    // ---- Camera open (skipped for espnode and 'none' modes) ------------------
    cv::VideoCapture cap;
    if (!useIngest) {
        cap = openCamera(args.cameraSource);
        if (!cap.isOpened()) {
            std::cerr << "Failed to open camera: " << args.cameraSource << "\n";
            return 1;
        }
    } else if (!useEspnode) {
        std::cout << "Camera source: none — frames will be received via POST /api/frame\n";
    }

    std::cout << "Initializing ORB-SLAM3\n";
    ORB_SLAM3::System slam(args.vocabPath, args.settingsPath, ORB_SLAM3::System::MONOCULAR, true);
    ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_DEBUG);
    const float imageScale = slam.GetImageScale();
    int targetW = 0, targetH = 0;
    slam.GetTargetSize(targetW, targetH);
    const bool targetMode = (targetW > 0 || targetH > 0);

    // Frame preprocessing pipeline (CLAHE, etc.) built from the YAML Preproc.* keys.
    const FramePreprocessor preproc = FramePreprocessor::fromSettingsFile(args.settingsPath);
    std::cout << preproc.describe() << "\n";
    if (targetMode) {
        std::cout << "Target processing resolution: "
                  << (targetW > 0 ? std::to_string(targetW) : "auto") << "x"
                  << (targetH > 0 ? std::to_string(targetH) : "auto")
                  << " (frames scaled uniformly inside tracking)\n";
    } else {
        std::cout << "Image scale: " << imageScale << "\n";
    }

    // Shared state
    LifecycleFlags     flags;
    PoseState          pose;
    IngestQueue        ingestQueue;
    std::atomic<bool>  localizationMode{false};
    std::atomic<bool>  allowMapCreation{true};
    // Initial state follows the Tracking.useMotionModel YAML key (default on).
    std::atomic<bool>  useMotionModel{slam.GetUseMotionModel()};

    if (args.localizeOnly) {
        std::cout << "Activating localization mode\n";
        localizationMode = true;
        slam.GetAtlas()->SwitchToMap(static_cast<unsigned int>(args.mapId));
        slam.ActivateLocalizationMode();
        slam.ForceRelocalization();
        // In localization-only startup, suppress map creation by default.
        allowMapCreation = false;
        slam.SetAllowMapCreation(false);
    }

    // Start espnode session (after SLAM is ready to accept frames)
    if (espSrc)
        espSrc->start(ingestQueue, imuBuf);

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
    WebServer          server(slam, flags, pose, calib, localizationMode, allowMapCreation,
                              useMotionModel, args.mapId, useIngest ? &ingestQueue : nullptr,
                              staticFileRoot, args.port);

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
                        handleStdinCommand(cmd, slam, flags, localizationMode, allowMapCreation, useMotionModel, args.mapId);
                }
            }
        });

        server.run(); // blocks until flags.running == false

        stdinThread.join();
    });

    // ---- Main tracking loop ------------------------------------------------
    std::cout << "\n-------\nStart processing sequence ...\n";
    cv::Mat frame;
    double  tframe = 0.0;

    while (flags.running) {
        if (useIngest) {
            IngestFrame ingest;
            if (!ingestQueue.pop(ingest, kPauseSleepUs))
                continue;
            frame  = std::move(ingest.image);
            tframe = ingest.timestamp;
            if (ingest.hasPose)
                slam.SetNextFramePose(ingest.Tcw);

            // Drain IMU samples that arrived since the last frame.
            if (espSrc) {
                auto imuSamples = imuBuf.drain();
                if (!imuSamples.empty()) {
                    const auto& latest = imuSamples.back();
                    std::cout << "IMU " << imuSamples.size() << " samples"
                              << "  R=" << latest.roll
                              << " P=" << latest.pitch
                              << " Y=" << latest.yaw
                              << "  V=" << latest.vx << "," << latest.vy << "," << latest.vz
                              << " mm/s\n";
                }
            }
        } else {
            // Always drain the capture buffer so we get the freshest frame.
            cap.read(frame);
            if (frame.empty()) {
                std::cout << "No image received\n";
                usleep(kPauseSleepUs);
                continue;
            }
#ifdef COMPILEDWITHC11
            auto now = std::chrono::steady_clock::now();
#else
            auto now = std::chrono::monotonic_clock::now();
#endif
            tframe = static_cast<double>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()).count());
        }

        // Legacy fixed image scaling. In target-resolution mode (targetW > 0)
        // the frame is left at its native size here and downscaled uniformly
        // inside Tracking::GrabImageMonocular, so calibration below also sees
        // the original-resolution frame.
        if (!targetMode && imageScale != 1.f) {
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

        // Preprocess (CLAHE, etc.) the frame fed to tracking. Calibration above
        // intentionally runs on the raw frame; mapping and localization both see
        // the preprocessed frame so feature descriptors stay consistent.
        preproc.process(frame);

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
    if (espSrc) espSrc->stop();
    controlThread.join();

    slam.Shutdown();
    slam.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
