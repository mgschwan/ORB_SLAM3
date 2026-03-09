/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>


#include<opencv2/core/core.hpp>
#include<opencv2/videoio.hpp>

#include<System.h>


bool running = true;

//signal handler
void sigint_handler(int sig)
{
    running = false;
}



using namespace std;

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings camera_url [localize_only] [map_id]" << endl;
        return 1;
    }

    bool localizationMode = false;
    if ( argc >= 5 ) {
        localizationMode = true;
    }
    string strFile = string(argv[3]); 

    int map_id = 0;
    if ( argc >= 6 ) {
        map_id = atoi(argv[5]);
    }

    cv::VideoCapture cap;
    
    cap.open(strFile);



    //int nImages = vstrImageFilenames.size();
    //cout << "Number of images: " << nImages << endl;
    cout << "Initializing ORB-SLAM3" << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_DEBUG);
    float imageScale = SLAM.GetImageScale();

    if (localizationMode) 
    {
        cout << "Activating localization mode" << endl;
        SLAM.GetAtlas()->SwitchToMap(map_id);
        SLAM.ActivateLocalizationMode();
        SLAM.ForceRelocalization();
    }


    cout << "Image scale: " << imageScale << endl;

    std::thread controlThread([&SLAM, &localizationMode, map_id]() {
        int server_fd;
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            cerr << "Socket creation failed" << endl;
            return;
        }
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
        struct sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(8080);
        
        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            cerr << "Socket bind failed" << endl;
            return;
        }
        listen(server_fd, 3);
        
        cout << endl << "--------------------------------------------------------" << endl;
        cout << "Remote control running at http://localhost:8080" << endl;
        cout << "Console commands: 'loc', 'map', 'quit'" << endl;
        cout << "--------------------------------------------------------" << endl << endl;

        while (running) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(server_fd, &readfds);
            FD_SET(STDIN_FILENO, &readfds);
            
            int max_sd = server_fd > STDIN_FILENO ? server_fd : STDIN_FILENO;
            
            struct timeval tv = {1, 0};
            int activity = select(max_sd + 1, &readfds, NULL, NULL, &tv);
            
            if (activity > 0) {
                if (FD_ISSET(STDIN_FILENO, &readfds)) {
                    std::string command;
                    std::cin >> command;
                    if(command == "loc" || command == "localize") {
                        if (!localizationMode) {
                            localizationMode = true;
                            SLAM.GetAtlas()->SwitchToMap(map_id);
                            SLAM.ActivateLocalizationMode();
                            SLAM.ForceRelocalization();
                            cout << ">>> Switched to Localization Mode <<<" << endl;
                        }
                    } else if(command == "map" || command == "mapping") {
                        if (localizationMode) {
                            localizationMode = false;
                            SLAM.DeactivateLocalizationMode();
                            cout << ">>> Switched to Mapping Mode <<<" << endl;
                        }
                    } else if(command == "quit" || command == "exit") {
                        running = false;
                    }
                }
                if (FD_ISSET(server_fd, &readfds)) {
                    int new_socket = accept(server_fd, NULL, NULL);
                    if (new_socket < 0) continue;
                    
                    char buffer[1024] = {0};
                    int valread = read(new_socket, buffer, 1024);
                    if (valread > 0) {
                        std::string req(buffer);
                        if (req.find("GET /loc") != std::string::npos) {
                            if (!localizationMode) {
                                localizationMode = true;
                                SLAM.GetAtlas()->SwitchToMap(map_id);
                                SLAM.ActivateLocalizationMode();
                                SLAM.ForceRelocalization();
                                cout << ">>> [Web] Switched to Localization Mode <<<" << endl;
                            }
                        } else if (req.find("GET /map") != std::string::npos) {
                            if (localizationMode) {
                                localizationMode = false;
                                SLAM.DeactivateLocalizationMode();
                                cout << ">>> [Web] Switched to Mapping Mode <<<" << endl;
                            }
                        }
                        
                        std::string mode_str = localizationMode ? "Localization" : "Mapping";
                        std::string html = std::string("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n") +
                                           "<html><head><title>ORB-SLAM3 Control</title>" +
                                           "<style>body{font-family:sans-serif;margin:40px;background:#f5f5f5;text-align:center;}" +
                                           ".container{background:#fff;padding:30px;border-radius:8px;box-shadow:0 4px 6px rgba(0,0,0,0.1);max-width:500px;margin:auto;}" +
                                           ".btn{padding:12px 24px;margin:10px;text-decoration:none;border:1px solid #ccc;border-radius:4px;display:inline-block;color:#333;font-weight:bold;transition:0.3s;}" +
                                           ".btn:hover{background:#e0e0e0;} .active{background:#007bff;color:#fff;border-color:#007bff;} .active:hover{background:#0056b3;}</style>" +
                                           "</head><body><div class=\"container\">" +
                                           "<h2>ORB-SLAM3 Remote Control</h2>" +
                                           "<h3>Current Mode: <span style=\"color:" + (localizationMode ? "#28a745" : "#007bff") + ";\">" + mode_str + "</span></h3><hr>" +
                                           "<div>" +
                                           "<a href=\"/loc\" class=\"btn " + (localizationMode ? "active" : "") + "\">Localization Mode</a>" +
                                           "<a href=\"/map\" class=\"btn " + (!localizationMode ? "active" : "") + "\">Mapping Mode</a>" +
                                           "</div>" +
                                           "<p style=\"margin-top:20px;font-size:12px;color:#777;\">Refresh page to update current status.</p>" +
                                           "</div></body></html>";
                        int r = write(new_socket, html.c_str(), html.length());
                        (void)r; // suppress warning
                    }
                    close(new_socket);
                }
            }
        }
        close(server_fd);
    });

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat im;

    //Install signal handler 
    signal(SIGINT, sigint_handler);

    while(running)  //for(int ni=0; ni<nImages; ni++)
    {

        double tframe = 0.0;
        //cout << "Read image at: " << vstrImageFilenames[ni] << endl;
        // Read image from file
        #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point read_time = std::chrono::steady_clock::now();
            tframe = std::chrono::duration_cast<std::chrono::milliseconds>(read_time.time_since_epoch()).count();
        #else
            std::chrono::monotonic_clock::time_point read_time = std::chrono::monotonic_clock::now();
            tframe = std::chrono::duration_cast<std::chrono::milliseconds>(read_time.time_since_epoch()).count();
        #endif

        cap.read(im);
        
        if ( im.empty() ) {
            cout << "No image received" << endl;
            continue;
        }
        

        if(imageScale != 1.f)
        {
            cout << "Resizing image to: " << imageScale << endl;

            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));

        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        //cv::imwrite("/tmp/image.jpg", im);

        // if (localizationMode) 
        // {
        //     SLAM.ForceRelocalization();
        // }

        // cout << "Track image" << endl;
        // Pass the image to the SLAM system
        Sophus::SE3f Tcw = SLAM.TrackMonocular(im, tframe);

        cout << "Position " << Tcw.translation().transpose() << " Rotation " << Tcw.angleX() << "," << Tcw.angleY() << "," << Tcw.angleZ() <<  endl;



#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    }

    cout << "End of sequence reached. Waiting 5 seconds before closing everything." << endl;
    running = false; // ensure thread can exit
    controlThread.join();
    usleep(5000000);

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

