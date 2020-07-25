/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(string &strPath, vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);
std::string getDirEnd(std::string dataset_dir);
ofstream staticsFile("./output/"+string("statics_temp.txt"));
int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_euroc path_to_vocabulary path_to_settings dataset_path" << endl;
        return 1;
    }
    //// Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamp;
    string dataset_path = string(argv[3]);
    LoadImages(dataset_path, vstrImageLeft, vstrImageRight, vTimeStamp);
    if(vstrImageLeft.empty() || vstrImageRight.empty())
    {
        cerr << "ERROR: No images in provided path." << endl;
        return 1;
    }
    if(vstrImageLeft.size()!=vstrImageRight.size())
    {
        cerr << "ERROR: Different number of left and right images." << endl;
        return 1;
    }
    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);

    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
       rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }
        //cv::imshow("imLeft",imLeft);
        //cv::waitKey(0);

        if(imRight.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageRight[ni]) << endl;
            return 1;
        }
        
//         校正
        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

        //cv::Size imageSize(cols_l,rows_l);
        //cv::Mat canvas(imageSize.height, imageSize.width * 2, CV_8UC3);
        //cv::Mat canLeft = canvas(cv::Rect(0, 0, imageSize.width, imageSize.height));
        //cv::Mat canRight = canvas(cv::Rect(imageSize.width, 0, imageSize.width, imageSize.height));
        ////cout<<"canLeft: "<<imLeft.type()<<" canvas: "<<canvas.type()<<endl;
        //imLeftRect(cv::Rect(0, 0, imageSize.width, imageSize.height)).copyTo(canLeft);
        //imRightRect(cv::Rect(0, 0, imageSize.width, imageSize.height)).copyTo(canRight);
        //for (int j = 0; j <= canvas.rows; j += 16)
        //    cv::line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
        //cv::imshow("canvas",canvas);
        //cv::waitKey(0);

        double tframe = vTimeStamp[ni];

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        cout << "frame: " << ni << " ";
        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeftRect,imRightRect,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        
//         track时长
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimeStamp[ni+1]-tframe;
        //用上一次的近似
        else if(ni>0)
            T = tframe-vTimeStamp[ni-1];

        //if(ni==2)
        //{
        //    while(1);
        //}
        if(ttrack<T)
        {
            //cout << "real-time frame: " << ni << " ttrack/T: "<<ttrack<<' '<<T << endl;
            usleep((T-ttrack)*1e6);
        }
        else
            cout << "fake-time frame: " << ni <<" ttrack/T: "<<ttrack<<' '<<T<< endl;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    string file_prefix = dataset_path + "robot" + getDirEnd(dataset_path)+"_";
    SLAM.SaveTrajectoryTUM(file_prefix + string("orb_stereo_slam.txt"));

    return 0;
}

#define SPEED_UP 5
void LoadImages(string &strPath, vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    cerr << "Start LoadImages." << endl;
    vTimeStamps.reserve(10000);
    vstrImageLeft.reserve(10000);
    vstrImageRight.reserve(10000);


    unsigned int iSize = strPath.size();
    if(strPath.at(iSize-1)!='/')
        strPath.push_back('/');

    ifstream fTimes;
    string strPathTimeFile = strPath + "cameraStamps.txt";
    fTimes.open(strPathTimeFile.c_str());
    uint8_t cnt = 0;
    while(!fTimes.eof())
    {
        cnt++;
        if(cnt<SPEED_UP)
            continue;
        else
            cnt = 0;
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimeStamps.push_back(t);
        }
    }
    string strPathLeft = strPath + "left";
    string strPathRight = strPath + "right";
    int img_i=0;
    do{
        img_i = img_i + 1;
        cnt++;
        if(cnt<SPEED_UP)
            continue;
        else
            cnt = 0;
        stringstream ss;
        ss << setfill('0') << setw(6) << img_i;
        std::string file = strPathLeft + "/" + ss.str() + ".jpg";
        if(exists_file(file))
        {
            double t = img_i/10.0;
            ss.clear();ss.str("");
            ss << setfill('0') << setw(6) << img_i;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".jpg");
            ss.clear();ss.str("");
            ss << setfill('0') << setw(6) << img_i;
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".jpg");
        }
        else
            break;
    }while(1);

    assert(vTimeStamps.size()==vstrImageLeft.size() && vTimeStamps.size()==vstrImageRight.size());

    cout<<"Finish LoadImages: "<<vstrImageLeft.size()<<endl;
}
