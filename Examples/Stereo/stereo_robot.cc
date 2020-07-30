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
#include"util.h"
#include"ImuTypes.h"

using namespace std;
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro, vector<cv::Point3f> &vOdo);
void LoadImages(string &strPath, vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);
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
    vector<cv::Point3f> vAcc, vGyro, vOdo;
    vector<double> vTimestampsImu;
    int first_imu = 0;

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


    string pathImu = dataset_path + "/robot.txt";
    cout << "Loading IMU ";
    LoadIMU(pathImu, vTimestampsImu, vAcc, vGyro, vOdo);
    cout << "LOADED!" << endl;

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
    std::chrono::steady_clock::time_point t_init = std::chrono::steady_clock::now();
    double tframeInit = vTimeStamp[0];
    int frameNext = 0;
    vector<ORB_SLAM2::IMU::Point> vImuMeas;
    for(int ni=0; ni<nImages; ni++)
    {
        //if(ni<frameNext)
        //    continue;
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
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

        // Load imu measurements from previous frame
        vImuMeas.clear();

        if(ni>0)
            while(vTimestampsImu[first_imu]<=vTimeStamp[ni]) // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
            {
                vImuMeas.push_back(ORB_SLAM2::IMU::Point(vAcc[first_imu].x,vAcc[first_imu].y,vAcc[first_imu].z,
                                                         vGyro[first_imu].x,vGyro[first_imu].y,vGyro[first_imu].z,
                                                         vTimestampsImu[first_imu]));
                first_imu++;
            }

        cout << "frame: " << (ni + 1)*SPEED_UP-1 << " ";
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
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_now = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t_now = std::chrono::monotonic_clock::now();
#endif
        double fromInit = std::chrono::duration_cast<std::chrono::duration<double> >(t_now - t_init).count();
        for(int niNext=ni; niNext<nImages; niNext++)
        {
            double timePass = vTimeStamp[niNext] - tframeInit;
            if(timePass<=fromInit)
                frameNext = niNext;
            else
                break;
        }
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
    if(CLOSE_LOOP)
        SLAM.SaveTrajectoryTUM(file_prefix + string("orb_stereo_slam.txt"));
    else
        SLAM.SaveTrajectoryTUM(file_prefix + string("orb_stereo_vo.txt"));

    return 0;
}

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
        string s;
        getline(fTimes,s);
        if(cnt<SPEED_UP)
            continue;
        else
            cnt = 0;
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

    //load image 法一：
    //getSortedImages(strPathLeft, vstrImageLeft);
    //getSortedImages(strPathRight, vstrImageRight);

    //load image 法二：
    int img_i=-1;
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
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro, vector<cv::Point3f> &vOdo)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);
    vOdo.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty()) {
            string item;
            size_t pos = 0;
            double data[10];
            int count = 0;
            while ((pos = s.find(' ')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            //字符串转浮点数
            data[9] = stod(item);

            vTimeStamps.push_back(data[0]);
            vGyro.push_back(cv::Point3f(data[1], data[2], data[3]));
            vAcc.push_back(cv::Point3f(data[4], data[5], data[6]));
            vOdo.push_back(cv::Point3f(data[7], data[8], data[9]));
        }
    }
    cout << "Finish LoadIMU: " << vTimeStamps.size() << endl;
}

