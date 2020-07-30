//
// Created by qzj on 2020/7/29.
//

#ifndef ORB_SLAM2_ODOMETER_H
#define ORB_SLAM2_ODOMETER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <unistd.h>
#include "Tracking.h"
#include <mutex>
#include "Converter.h"


namespace ORB_SLAM2 {

    class OdoPose {
    public:
        cv::Mat mTwo;

        Eigen::Quaternionf mQuatf;

        cv::Point3f mxyz;

        double mTimestamp;

    public:
        OdoPose(){}

        OdoPose(const OdoPose& pose):mQuatf(pose.mQuatf), mxyz(pose.mxyz), mTimestamp(pose.mTimestamp), mTwo(pose.mTwo){}

        OdoPose(const float &odo_x, const float &odo_y, const float &odo_z, const float &qx, const float &qy,
                const float &qz, const float &qw, const double &timestamp) {

            mQuatf = Eigen::Quaternionf(qw,qx,qy,qz);
            mxyz = cv::Point3f(odo_x,odo_y,odo_z);
            mTimestamp = timestamp;

            mTwo = cv::Mat::eye(4, 4, CV_32F);
            mTwo.at<float>(0,3) = mxyz.x;
            mTwo.at<float>(1,3) = mxyz.y;
            mTwo.at<float>(2,3) = mxyz.z;

            Converter::toCvMat((Eigen::Matrix3f)mQuatf.matrix()).copyTo(mTwo.rowRange(0,3).colRange(0,3));
        }

        cv::Mat GetVelInCamera(cv::Mat Tc_odo);
    };

    class Odometer {
    public:
        Odometer() {};

        Odometer(const string& strSettingPath);

        float mRoll;
        float mPicth;
        float mYaw;

        Eigen::Quaternionf quaternionfOdo;

        float mOdo_x;
        float mOdo_y;
        float mOdo_z;

        // Odometer pose.
        cv::Mat mTwo;

        cv::Mat mTc_odo;

        OdoPose mCurrentOdo;
        OdoPose mLastOdo;

        //Motion Model
        cv::Mat mVelocity;
        cv::Mat mVelocityCam;

        static long unsigned int nNextId;

        long unsigned int mnId;

        std::mutex mMutexToc;

    public:
        void UpdatePose(vector<OdoPose>& odoPose);
        void RememberLast();
        cv::Mat GetTc_odo();
        void SetTc_odo(cv::Mat mat);
    };
}

#endif //ORB_SLAM2_ODOMETER_H
