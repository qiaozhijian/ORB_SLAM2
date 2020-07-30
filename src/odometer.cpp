//
// Created by qzj on 2020/7/29.
//

#include "odometer.h"
#include <opencv2/core/eigen.hpp>
#include "Converter.h"

namespace ORB_SLAM2{

    long unsigned int Odometer::nNextId = 0;

    cv::Mat OdoPose::GetVelInCamera(cv::Mat Tc_odo)
    {
        cv::Mat T_odo = cv::Mat::eye(4, 4, CV_32F);
        T_odo.at<float>(0,3) = mxyz.x;
        T_odo.at<float>(1,3) = mxyz.y;
        T_odo.at<float>(2,3) = mxyz.z;

        Converter::toCvMat((Eigen::Matrix3f)mQuatf.matrix()).copyTo(T_odo.rowRange(0,3).colRange(0,3));
        
        cv::Mat Todo_c = Converter::toCvMatInverse(T_odo);
    }

    Odometer::Odometer(const string& strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        fSettings["Tc_odo"] >> mTc_odo;

        mVelocityCam = cv::Mat();
        mVelocity = cv::Mat();

        mCurrentOdo.mTimestamp = -1.0;
        mLastOdo.mTimestamp = -1.0;
    }
    // todo 加时间戳对齐和SE空间插值
    void Odometer:: UpdatePose(vector<OdoPose>& vOdoPose)
    {
        mnId = nNextId++;
        if(!vOdoPose.empty())
        {
            OdoPose m = vOdoPose.back();
            mCurrentOdo = OdoPose(vOdoPose.back());
        } else
        {
            mVelocity = cv::Mat();
            mVelocityCam = cv::Mat();
            mCurrentOdo.mTimestamp = -1.0;
            mLastOdo.mTimestamp = -1.0;
            return;
        }

        if (mLastOdo.mTimestamp > 0.0 && mCurrentOdo.mTimestamp > 0.0) {
            cv::Mat CurTow = Converter::toCvMatInverse(mCurrentOdo.mTwo);
            mVelocity = CurTow * mLastOdo.mTwo;
            {
                unique_lock<mutex> lock(mMutexToc);
                mVelocityCam = mTc_odo * mVelocity;
                cv::Mat mTodo_c = Converter::toCvMatInverse(mTc_odo);
                mVelocityCam = mVelocityCam * mTodo_c;
            }
        } else
        {
            mVelocity = cv::Mat();
            mVelocityCam = cv::Mat();
        }
    }
    cv::Mat Odometer:: GetTc_odo()
    {
        unique_lock<mutex> lock(mMutexToc);
        return mTc_odo;
    }

    void Odometer:: SetTc_odo(cv::Mat mat)
    {
        unique_lock<mutex> lock(mMutexToc);
        mTc_odo = mat;
    }


    void Odometer:: RememberLast()
    {
        if(mCurrentOdo.mTimestamp>0.0)
            mLastOdo = OdoPose(mCurrentOdo);
    }


}