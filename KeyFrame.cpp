//
//  KeyFrame.cpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/10/19.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#include "KeyFrame.hpp"

KeyFrame::KeyFrame(int iStep, Eigen::Matrix<double, 10, 1>& Xv) : FrameID(iStep), MedianDepth(1.0)
{
    t_Wc = Xv.head(3);
    v_Wc = Xv.tail(3);
    
    Eigen::Quaterniond qcw(Xv(3), Xv(4), Xv(5), Xv(6));
    R_Cw = qcw.matrix().transpose();
}


void KeyFrame::SetPose(Eigen::Vector3d &rwc, Eigen::Matrix3d &Rcw)
{
    t_Wc = rwc;
    R_Cw = Rcw;
}


void KeyFrame::SetPoseSix(Eigen::Matrix<double, 6, 1> &pose)
{
    Eigen::Vector3d r_;
    
    r_ = pose.head(3);
    
    double angle = r_.norm();
    if (angle > 1e-6) {
        r_ /= angle;
    }
    
    Eigen::AngleAxisd axi(angle, r_);
    Eigen::Matrix3d R(axi);
    
    R_Cw = R;
    t_Wc = pose.tail(3);
}


void KeyFrame::SetPoseNine(Eigen::Matrix<double, 9, 1> &pose)
{
    Eigen::Vector3d r_ ;
    
    r_ = pose.head(3);
    
    
    double angle = r_.norm();
    r_ /= angle;
    Eigen::AngleAxisd axi(angle, r_);
    Eigen::Matrix3d R(axi);
    
    R_Cw = R;
    t_Wc = pose.segment(3, 3);
    v_Wc = pose.tail(3);
}


void KeyFrame::AddObservation(MapPoint *ppt)
{
    vpMapPoints.insert(ppt);
}