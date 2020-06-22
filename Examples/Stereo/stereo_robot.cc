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
int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_euroc path_to_vocabulary path_to_settings dataset_path" << endl;
        return 1;
    }
    // Retrieve paths to images
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

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r,T_lr,R_lr;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;


    fsSettings["TranslationOfCamera2"] >> T_lr;
    fsSettings["RotationOfCamera2"] >> R_lr;
    //cout<<R_lr<<endl;
    //R_lr=R_lr.t();
    //cout<<R_lr<<endl;
    //T_lr = T_lr/10;

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

    cv::Size imageSize(cols_l,rows_l);
    //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
    cv::Mat Rl, Rr, Pl, Pr, Q;
    //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域, 其内部的所有像素都有效
    cv::Rect validROIL;
    cv::Rect validROIR;
    //经过双目标定得到摄像头的各项参数后，采用OpenCV中的stereoRectify(立体校正)得到校正旋转矩阵R、投影矩阵P、重投影矩阵Q
    //flags-可选的标志有两种零或者 CV_CALIB_ZERO_DISPARITY ,如果设置 CV_CALIB_ZERO_DISPARITY 的话，该函数会让两幅校正后的图像的主点有相同的像素坐标。否则该函数会水平或垂直的移动图像，以使得其有用的范围最大
    //alpha-拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
    stereoRectify(K_l, D_l, K_r, D_r, imageSize, R_lr, T_lr, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY,
                  0, imageSize, &validROIL, &validROIR);
    //cout<<Pl<<endl;
    //cout<<Pr<<endl;
// 相机校正
    cv::Mat M1l,M2l,M1r,M2r;
    //再采用映射变换计算函数initUndistortRectifyMap得出校准映射参数,该函数功能是计算畸变矫正和立体校正的映射变换
    cv::initUndistortRectifyMap(K_l,D_l,Rl,Pl.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,Rr,Pr.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    //cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    //cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    //cv::initUndistortRectifyMap(K_l,D_l,R_l,cv::Mat(),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    //cv::initUndistortRectifyMap(K_r,D_r,R_r,cv::Mat(),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

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
        //cout<<"canLeft: "<<imLeft.type()<<" canvas: "<<canvas.type()<<endl;
        //imLeftRect(cv::Rect(0, 0, imageSize.width, imageSize.height)).copyTo(canLeft);
        //imRightRect(cv::Rect(0, 0, imageSize.width, imageSize.height)).copyTo(canRight);
        //cout << "done" << endl;
        //for (int j = 0; j <= canvas.rows; j += 16)
        //    cv::line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
        //    cout << "stereo rectify done" << endl;
        //cv::imshow("canvas",canvas);
        //cv::waitKey(0);

        double tframe = vTimeStamp[ni];


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

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
        if(ttrack<T)
        {
            cout << "real-time frame: " << ni << " ttrack/T: "<<ttrack<<' '<<T << endl;
            //usleep((T-ttrack)*1e6);
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
    SLAM.SaveTrajectoryTUM(file_prefix + string("orb_slam.txt"));

    return 0;
}

#include <sys/stat.h>
inline bool exists_file (const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

std::string getDirEnd(std::string dataset_dir)
{
    std::string end;
    unsigned int iSize = dataset_dir.size();
    unsigned int i = 0;
    for(i = 0; i < iSize; i++)
    {
        if(dataset_dir.at(i)=='/' && i!=iSize-1)
            end=dataset_dir.substr(i+1);
    }
    if (end[end.size()-1]=='/')
        end.pop_back();
    return end;
}


void LoadImages(string &strPath, vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    cerr << "Start LoadImages." << endl;
    //vTimeStamps.reserve(10000);
    //vstrImageLeft.reserve(10000);
    //vstrImageRight.reserve(10000);

    unsigned int iSize = strPath.size();
    if(strPath.at(iSize-1)!='/')
        strPath.push_back('/');

    string strPathLeft = strPath + "left";
    string strPathRight = strPath + "right";
    int left_i=0, right_i=1;
    do{
        stringstream ss;
        ss << setfill('0') << setw(6) << left_i;
        std::string file = strPathLeft + "/" + ss.str() + ".jpg";
        if(exists_file(file))
        {
            double t = 0.05*left_i;
            vTimeStamps.push_back(t);
            ss.clear();ss.str("");
            ss << setfill('0') << setw(6) << left_i;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".jpg");
            ss.clear();ss.str("");
            ss << setfill('0') << setw(6) << right_i;
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".jpg");
            left_i = left_i + 2;
            right_i = right_i + 2;
        }
        else
            break;
    }while(1);

    cout<<"Finish LoadImages: "<<vstrImageLeft.size()<<endl;
}
