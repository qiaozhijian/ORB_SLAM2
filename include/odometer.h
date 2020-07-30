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


namespace ORB_SLAM2 {

    class OdoPose {

    public:
        OdoPose(){}

        OdoPose(OdoPose pose):mQuatf(pose.mQuatf), mxyz(pose.mxyz), mTimestamp(pose.mTimestamp){}

        OdoPose(const float &odo_x, const float &odo_y, const float &odo_z, const float &qx, const float &qy,
                const float &qz, const float &qw, const double &timestamp) {

            mQuatf = Eigen::Quaternionf(qw,qx,qy,qz);
            mxyz = cv::Point3f(odo_x,odo_y,odo_z);
            mTimestamp = timestamp;

            mTow.at<float>(0,3) = mxyz.x;
            mTow.at<float>(1,3) = mxyz.y;
            mTow.at<float>(2,3) = mxyz.z;

            Converter::toCvMat((Eigen::Matrix3d)mQuatf.matrix()).copyTo(mTow.rowRange(0,3).colRange(0,3));
            //cout<<mQuatf.matrix()<<endl;
            //cout<<mTow<<endl;
        }

        cv::Mat GetVelInCamera(cv::Mat Tc_odo);

        cv::Mat mTow;

        Eigen::Quaternionf mQuatf;

        cv::Point3f mxyz;

        double mTimestamp;
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
        cv::Mat mTow;
        cv::Mat mTwo;

        cv::Mat mTc_odo;

        OdoPose mCurrentOdo;
        OdoPose mLastOdo;

        //Motion Model
        cv::Mat mVelocity;

        long unsigned int mnId;
        static long unsigned int nNextId = 0;
    public:
        void UpdatePose(vector<OdoPose>& odoPose);
        void RememberLast();

    };
}

#endif //ORB_SLAM2_ODOMETER_H
