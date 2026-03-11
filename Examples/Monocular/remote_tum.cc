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
bool paused = false;

// Global pose variables for SSE streaming
std::mutex pose_mutex;
float g_pose_x = 0, g_pose_y = 0, g_pose_z = 0;
float g_pose_qx = 0, g_pose_qy = 0, g_pose_qz = 0, g_pose_qw = 1;
bool g_pose_valid = false;

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
    
    bool is_number = true;
    for(char c : strFile) {
        if(!isdigit(c)) {
            is_number = false;
            break;
        }
    }
    
    if(is_number) {
        cap.open(stoi(strFile), cv::CAP_V4L2);
    } else if (strFile.find("/dev/video") == 0) {
        cap.open(strFile, cv::CAP_V4L2);
    } else {
        cap.open(strFile);
    }



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
        cout << "Console commands: 'loc', 'map', 'quit', 'pause', 'resume'" << endl;
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
                    } else if(command == "pause") {
                        paused = true;
                        cout << ">>> Paused Processing <<<" << endl;
                    } else if(command == "resume") {
                        paused = false;
                        cout << ">>> Resumed Processing <<<" << endl;
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
                        
                        // Default to serving index.html
                        std::string response;
                        
                        if (req.find("GET /api/stream/pose") != std::string::npos) {
                            // Launch detached thread for Server-Sent Events (SSE)
                            std::thread sse_thread([new_socket]() {
                                std::string headers = "HTTP/1.1 200 OK\r\n"
                                                      "Content-Type: text/event-stream\r\n"
                                                      "Cache-Control: no-cache\r\n"
                                                      "Connection: keep-alive\r\n"
                                                      "Access-Control-Allow-Origin: *\r\n\r\n";
                                
                                // Ignore SIGPIPE on Linux so write doesn't crash the program if client disconnects
                                signal(SIGPIPE, SIG_IGN);

                                if (write(new_socket, headers.c_str(), headers.length()) < 0) {
                                    close(new_socket);
                                    return;
                                }

                                while (running) {
                                    std::string json;
                                    {
                                        std::lock_guard<std::mutex> lock(pose_mutex);
                                        if (g_pose_valid) {
                                            json = "data: {\"valid\":true,\"x\":" + std::to_string(g_pose_x) + 
                                                   ",\"y\":" + std::to_string(g_pose_y) + 
                                                   ",\"z\":" + std::to_string(g_pose_z) + 
                                                   ",\"qx\":" + std::to_string(g_pose_qx) + 
                                                   ",\"qy\":" + std::to_string(g_pose_qy) + 
                                                   ",\"qz\":" + std::to_string(g_pose_qz) + 
                                                   ",\"qw\":" + std::to_string(g_pose_qw) + "}\n\n";
                                        } else {
                                            json = "data: {\"valid\":false}\n\n";
                                        }
                                    }
                                    
                                    if (write(new_socket, json.c_str(), json.length()) < 0) {
                                        // Client disconnected or socket error
                                        break;
                                    }
                                    
                                    usleep(33000); // ~30 fps
                                }
                                close(new_socket);
                            });
                            sse_thread.detach();
                            continue; // Do not close socket here, the detached thread handles it
                        }
                        else if (req.find("GET /api/status") != std::string::npos) {
                            long unsigned int current_map_id = 0;
                            if(SLAM.GetAtlas()->GetCurrentMap()) {
                                current_map_id = SLAM.GetAtlas()->GetCurrentMap()->GetId();
                            }

                            std::string json = "{";
                            json += "\"localizationMode\": " + std::string(localizationMode ? "true" : "false") + ",";
                            json += "\"paused\": " + std::string(paused ? "true" : "false") + ",";
                            json += "\"currentMapId\": " + std::to_string(current_map_id) + ",";
                            json += "\"maps\": [";
                            
                            std::vector<ORB_SLAM3::Map*> maps = SLAM.GetAtlas()->GetAllMaps();
                            bool first = true;
                            for (ORB_SLAM3::Map* pMap : maps) {
                                if (!pMap) continue;
                                if (!first) json += ",";
                                json += "{";
                                json += "\"id\": " + std::to_string(pMap->GetId()) + ",";
                                json += "\"keyframes\": " + std::to_string(pMap->KeyFramesInMap()) + ",";
                                json += "\"mappoints\": " + std::to_string(pMap->MapPointsInMap());
                                json += "}";
                                first = false;
                            }
                            json += "]}";

                            response = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n" + json;
                        } 
                        else if (req.find("GET /loc ") != std::string::npos) {
                            if (!localizationMode) {
                                localizationMode = true;
                                SLAM.GetAtlas()->SwitchToMap(map_id);
                                SLAM.ActivateLocalizationMode();
                                SLAM.ForceRelocalization();
                                cout << ">>> [Web] Switched to Localization Mode <<<" << endl;
                            }
                            response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK";
                        } else if (req.find("GET /map ") != std::string::npos) {
                            if (localizationMode) {
                                localizationMode = false;
                                SLAM.DeactivateLocalizationMode();
                                cout << ">>> [Web] Switched to Mapping Mode <<<" << endl;
                            }
                            response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK";
                        } else if (req.find("GET /pause ") != std::string::npos) {
                            paused = true;
                            cout << ">>> [Web] Paused Processing <<<" << endl;
                            response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK";
                        } else if (req.find("GET /resume ") != std::string::npos) {
                            paused = false;
                            cout << ">>> [Web] Resumed Processing <<<" << endl;
                            response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK";
                        } else if (req.find("GET /switchmap?id=") != std::string::npos) {
                            size_t pos = req.find("GET /switchmap?id=");
                            size_t end_pos = req.find(" HTTP", pos);
                            if (end_pos != std::string::npos) {
                                std::string id_str = req.substr(pos + 18, end_pos - (pos + 18));
                                int new_map_id = std::stoi(id_str);
                                SLAM.SwitchToMap(new_map_id);
                                if (localizationMode) {
                                    SLAM.ForceRelocalization();
                                }
                                cout << ">>> [Web] Switched to Map ID: " << new_map_id << " <<<" << endl;
                            }
                            response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK";
                        } else if (req.find("GET /newmap ") != std::string::npos) {
                            SLAM.GetAtlas()->CreateNewMap();
                            SLAM.SwitchToMap(SLAM.GetAtlas()->CountMaps() - 1);
                            cout << ">>> [Web] Created and switched to New Map <<<" << endl;
                            response = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nOK";
                        } else {
                            // Serve static files
                            size_t start_pos = req.find("GET /");
                            if (start_pos != std::string::npos) {
                                start_pos += 5;
                                size_t end_pos = req.find(" HTTP", start_pos);
                                if (end_pos != std::string::npos) {
                                    std::string path = req.substr(start_pos, end_pos - start_pos);
                                    if (path.empty()) {
                                        path = "index.html";
                                    }
                                    
                                    // Prevent directory traversal attacks
                                    if (path.find("..") != std::string::npos || path.find("//") != std::string::npos || path[0] == '/') {
                                        response = "HTTP/1.1 403 Forbidden\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nForbidden";
                                    } else {
                                        std::string full_path = "html/" + path;
                                        std::ifstream file(full_path, std::ios::binary);
                                        if (file.is_open()) {
                                            std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
                                            std::string content_type = "text/html";
                                            if (path.find(".css") != std::string::npos) content_type = "text/css";
                                            else if (path.find(".js") != std::string::npos) content_type = "application/javascript";
                                            else if (path.find(".png") != std::string::npos) content_type = "image/png";
                                            
                                            response = "HTTP/1.1 200 OK\r\nContent-Type: " + content_type + "\r\nConnection: close\r\n\r\n" + content;
                                        } else {
                                            response = "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nFile not found: " + full_path;
                                        }
                                    }
                                }
                            }
                        }

                        if (response.empty()) {
                            response = "HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\nBad Request";
                        }
                        
                        int r = write(new_socket, response.c_str(), response.length());
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
        if(paused) {
            usleep(100000); // Sleep for 100ms when paused to avoid busy looping
            continue;
        }

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

        if (SLAM.GetTrackingState() == ORB_SLAM3::Tracking::OK || SLAM.GetTrackingState() == ORB_SLAM3::Tracking::RECENTLY_LOST) {
            Sophus::SE3f Twc = Tcw.inverse();
            Eigen::Vector3f t = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            
            {
                std::lock_guard<std::mutex> lock(pose_mutex);
                g_pose_x = t.x();
                g_pose_y = t.y();
                g_pose_z = t.z();
                g_pose_qx = q.x();
                g_pose_qy = q.y();
                g_pose_qz = q.z();
                g_pose_qw = q.w();
                g_pose_valid = true;
            }
            cout << "Position " << Tcw.translation().transpose() << " Rotation " << Tcw.angleX() << "," << Tcw.angleY() << "," << Tcw.angleZ() <<  endl;
        } else {
            std::lock_guard<std::mutex> lock(pose_mutex);
            g_pose_valid = false;
        }



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

