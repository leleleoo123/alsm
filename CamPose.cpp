//
//  CamPose.cpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/9/23.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#include "CamPose.hpp"

CamPose::CamPose(int iStep, int num_features) {
    
    FrameID = iStep;
    Count   = num_features;
    
}