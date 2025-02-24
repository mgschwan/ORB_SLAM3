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
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings camera_url [localize_only]" << endl;
        return 1;
    }

    bool localizationMode = false;
    if ( argc == 5 ) {
        localizationMode = true;
    }

    string strFile = string(argv[3]); 

    cv::VideoCapture cap;
    
    cap.open(strFile);



    //int nImages = vstrImageFilenames.size();
    //cout << "Number of images: " << nImages << endl;
    cout << "Initializing ORB-SLAM3" << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    float imageScale = SLAM.GetImageScale();

    if (localizationMode) 
    {
        cout << "Activating localization mode" << endl;
        SLAM.ActivateLocalizationMode();
    }


    cout << "Image scale: " << imageScale << endl;

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

        cv::imwrite("/tmp/image.jpg", im);

        cout << "Track image" << endl;
        // Pass the image to the SLAM system
        Sophus::SE3f Tcw = SLAM.TrackMonocular(im, tframe);

        cout << "Position " << Tcw.translation().transpose() << " Rotation" << Tcw.angleX() << "," << Tcw.angleY() << "," << Tcw.angleZ() <<  endl;



#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

