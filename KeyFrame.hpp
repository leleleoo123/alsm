//
//  KeyFrame.hpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/10/19.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef KeyFrame_hpp
#define KeyFrame_hpp

#include <stdio.h>
#include <Eigen/Dense>
#include <set>
#include <Eigen/Geometry>
#include "MapPoint.hpp"

class KeyFrame
{
public:
    
    KeyFrame(int iStep, Eigen::Matrix<double, 10, 1>& Xv);
    
    void SetPose(Eigen::Vector3d& rwc, Eigen::Matrix3d& Rcw);
    void SetPoseSix(Eigen::Matrix<double, 6, 1>& pose);
    void SetPoseNine(Eigen::Matrix<double, 9, 1>& pose);
    
    void AddObservation(MapPoint* ppt);
    
public:
    
    Eigen::Matrix3d R_Cw;
    Eigen::Vector3d t_Wc;
    Eigen::Vector3d v_Wc;
    
    int FrameID;
    
    double MedianDepth;
    
    std::set<MapPoint*> vpMapPoints;
    
    std::map<KeyFrame*, int> pCoviKFs_Weights;
    
};




#endif /* KeyFrame_hpp */
