//
// Created by qzj on 2020/7/29.
//

#include "odometer.h"
#include <opencv2/core/eigen.hpp>
#include "Converter.h"

namespace ORB_SLAM2{

    cv::Mat OdoPose::GetVelInCamera(cv::Mat Tc_odo)
    {
        cv::Mat T_odo = cv::Mat::eye(4, 4, CV_32F);
        T_odo.at<float>(0,3) = mxyz.x;
        T_odo.at<float>(1,3) = mxyz.y;
        T_odo.at<float>(2,3) = mxyz.z;

        Converter::toCvMat((Eigen::Matrix3d)mQuatf.matrix()).copyTo(T_odo.rowRange(0,3).colRange(0,3));
        cout<<mQuatf.matrix()<<endl;
        cout<<T_odo<<endl;
        
        cv::Mat Todo_c = Converter::toCvMatInverse(T_odo);
        
        

    }

    Odometer::Odometer(const string& strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mTc_odo = fSettings["Tc_odo"].mat();
    }

    void Odometer:: UpdatePose(vector<OdoPose>& odoPose)
    {
        mnId = nNextId++;
        mCurrentOdo = OdoPose(*odoPose.end());

        if (mnId != 0) {
            cv::Mat LastTwo = Converter::toCvMatInverse(mLastOdo.mTow);
            mVelocity = mCurrentOdo.mTow * LastTwo;
        } else
            mVelocity = cv::Mat();


    }

    void Odometer:: RememberLast()
    {
        mLastOdo = OdoPose(mCurrentOdo);
    }


}