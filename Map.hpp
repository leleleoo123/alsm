//
//  Map.hpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/10/19.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef Map_hpp
#define Map_hpp

#include <stdio.h>
#include <set>
#include <Eigen/Dense>
#include "MapPoint.hpp"
#include "KeyFrame.hpp"
#include "ImuFactor.hpp"

#include "Optimizer.hpp"

typedef std::map<size_t, MapPoint*>::iterator ID_PPT_ITER;
typedef std::map<int, KeyFrame*>::iterator ID_PKF_ITER;
typedef std::map<int, ImuFactor*>::iterator ID_PIMU_ITER;

const double camfx = 458.65488     /2.0;
const double camfy = 457.296696    /2.0;
const double camCx = 367.2158      /2.0;
const double camCy = 248.375341    /2.0;

class Map
{
public:
    Map();
    
    void Reset();
    
    void AddKeyFrame(KeyFrame* pkf);
    
    void AddMapPoint(MapPoint* ppt);
    
    void AddImuFactor(ImuFactor* pImuFac);
    
    KeyFrame* GetOneKeyFrame(int id);

    void GetAllKeyFrames(std::vector<KeyFrame*> &v_kfs);
    
    void GetAllMapPoints(std::map<size_t, MapPoint*> &ids_mpts);
    
    size_t MapPointsSize();
    size_t KeyFramesSize();
    
    void BundleAdjustAll(bool useImu, bool rob, int maxKFid);
    
    void PrintStatics();
    
    void FuseSeveralPoints(std::set<MapPoint*> &ppts);
    
    void KillBadPoints();
    
private:
    
    void BundleAdjust(std::set<KeyFrame*> &sKFs, std::set<KeyFrame*> &sFix, std::set<MapPoint*> &sMpts, bool useImu, bool rob, int maxKFid, size_t maxMPTid);
    
    std::map<size_t, MapPoint*> IDs_pMapPoints;
    std::map<int, KeyFrame*> IDs_pKeyFrames;
    
    std::map<int, ImuFactor*> IDs_pImuFactors;
    
};

#endif /* Map_hpp */
