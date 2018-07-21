//
//  Feature.cpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/9/21.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#include "Feature.hpp"

Feature::Feature(cv::Mat& patch_ini, cv::Mat& patch_mat, int iStep):
 times_notmeasured(0), low_innovation_inlier(0), high_innovation_inlier(0), converged(0), hok(0), zok(0), too_uncertain(0), compatible(0), lastok(true)
{
    patch_ini.copyTo(patch_when_initialized);
    patch_mat.copyTo(patch_when_matching);
    FrameID = iStep;
}
