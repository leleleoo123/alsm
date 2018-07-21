//
//  MapPoint.hpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/10/19.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef MapPoint_hpp
#define MapPoint_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

using namespace cv;

class MapPoint
{
public:
    MapPoint(size_t pt_id, Mat& patch_ini, Mat& patch_mat, int kf_id, Point2f pt2f);
    
    void SetXwInverse(Eigen::Vector3d &ri, Eigen::Vector3d& tpr);
    
    void AddMeasurement(int kfid, Eigen::Vector2d z);
    
    bool isOk();
    
public:
    
    size_t PointID;
    
    Eigen::Vector3d XW;
    
    Mat PatchIni;
    
    Mat PatchMat;
    
    std::map<int, Point2f> KFIDs_Points2f;
};

#endif /* MapPoint_hpp */
