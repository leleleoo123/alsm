//
//  Feature.hpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/9/21.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef Feature_hpp
#define Feature_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class Feature {
public:
    
    Feature(cv::Mat& patch_ini, cv::Mat& patch_mat, int iStep);
    
    cv::Mat patch_when_initialized;
    cv::Mat patch_when_matching;
    
    int times_notmeasured;
    
    int FrameID;
    bool low_innovation_inlier;
    bool high_innovation_inlier;
    bool converged;
    bool hok;
    bool zok;
    bool too_uncertain;
    bool compatible;
    
    cv::Point2f LastPt;
    bool lastok;
    
    Eigen::Vector2d z;
    Eigen::Vector2d h;
    Eigen::Matrix2d S;
    
    Eigen::Matrix<double, 2, 7> H_rq;
    Eigen::Matrix<double, 2, 3> H_ri;
    Eigen::Matrix<double, 2, 3> H_tpr;
    
    
};


#endif /* Feature_hpp */
