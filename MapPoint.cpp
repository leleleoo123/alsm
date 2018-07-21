//
//  MapPoint.cpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/10/19.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#include "MapPoint.hpp"

MapPoint::MapPoint(size_t pt_id, Mat& patch_ini, Mat& patch_mat, int kf_id, Point2f pt2f) : PointID(pt_id)
{
    patch_ini.copyTo(PatchIni);
    patch_mat.copyTo(PatchMat);
    
    KFIDs_Points2f[kf_id] = pt2f;
}


void MapPoint::SetXwInverse(Eigen::Vector3d &ri, Eigen::Vector3d &tpr)
{
    double theta = tpr(0), phi = tpr(1);
    Eigen::Vector3d mi;
    
    double cphi = cos(phi);
    mi(0) = cphi * sin(theta);
    mi(1) = -sin(phi);
    mi(2) = cphi * cos(theta);
    
    XW = ri + mi/tpr(2);
}



void MapPoint::AddMeasurement(int kfid, Eigen::Vector2d z)
{
    Point2f pt(z(0), z(1));
    KFIDs_Points2f[kfid] = pt;
}


bool MapPoint::isOk()
{
    if (KFIDs_Points2f.size() > 1) {
        return true;
    } else {
        return false;
    }
}