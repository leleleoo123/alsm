//
//  Map.cpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/10/19.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#include "Map.hpp"

Map::Map() {
    Reset();
}


void Map::Reset() {
    // clear mappoints & keyframes
    for (std::map<size_t, MapPoint*>::iterator it=IDs_pMapPoints.begin(); it!=IDs_pMapPoints.end(); it++) {
        delete it->second;
    }
    
    for (std::map<int, KeyFrame*>::iterator it=IDs_pKeyFrames.begin(); it!=IDs_pKeyFrames.end(); it++) {
        delete it->second;
    }
    
    IDs_pMapPoints.clear();
    IDs_pKeyFrames.clear();
}

void Map::AddKeyFrame(KeyFrame *pkf)
{
    IDs_pKeyFrames[pkf->FrameID] = pkf;
}

void Map::AddMapPoint(MapPoint *ppt)
{
    IDs_pMapPoints[ppt->PointID] = ppt;
}

void Map::AddImuFactor(ImuFactor *pImuFac)
{
    IDs_pImuFactors[pImuFac->begin_frame_id] = pImuFac;
}


KeyFrame* Map::GetOneKeyFrame(int id)
{
    if (IDs_pKeyFrames.count(id) < 1) {
        std::cout << "Error: No keyframe with this id #" << id << "\n";
        waitKey();
    }
    return IDs_pKeyFrames[id];
}


void Map::GetAllKeyFrames(std::vector<KeyFrame *> &v_kfs)
{
    for (ID_PKF_ITER it=IDs_pKeyFrames.begin(); it!=IDs_pKeyFrames.end(); it++) {
        v_kfs.push_back(it->second);
    }
}


void Map::GetAllMapPoints(std::map<size_t, MapPoint *> &ids_mpts)
{
    ids_mpts = IDs_pMapPoints;
}



size_t Map::MapPointsSize()
{
    return IDs_pMapPoints.size();
}

size_t Map::KeyFramesSize()
{
    return IDs_pKeyFrames.size();
}

void Map::PrintStatics()
{
    std::map<int, int> obs_nums;
    int n;
    
    MapPoint* ppt;
    for (std::map<size_t, MapPoint*>::iterator it=IDs_pMapPoints.begin(); it!=IDs_pMapPoints.end(); it++) {
        ppt = it->second;
        
        n = (int)ppt->KFIDs_Points2f.size();
        
        if (obs_nums.count(n)) {
            obs_nums[n] += 1;
        } else {
            obs_nums[n] = 1;
        }
    }
    
    std::cout << "meas:\t" << "count\n";
    for (std::map<int, int>::iterator it=obs_nums.begin(); it!=obs_nums.end(); it++) {
        std::cout <<"   "<< it->first << "\t   " << it->second << "\n";
    }
}



void Map::FuseSeveralPoints(std::set<MapPoint *> &ppts)
{
    bool first = 1;
    
    for (std::set<MapPoint*>::iterator it = ppts.begin(); it!=ppts.end(); it++) {
        
        MapPoint* main_pt;
        if (first) {
            main_pt = *it;
            first = false;
            continue;
        }
        
        MapPoint* poor_pt = *it;
        
        for (std::map<int, Point2f>::iterator
             jt=poor_pt->KFIDs_Points2f.begin(); jt!=poor_pt->KFIDs_Points2f.end(); jt++)
        {
            
            int kf_id = jt->first;
            main_pt->KFIDs_Points2f[kf_id] = jt->second;
            
            KeyFrame* poor_kf = IDs_pKeyFrames[kf_id];
            poor_kf->vpMapPoints.erase(poor_pt);
            poor_kf->vpMapPoints.insert(main_pt);
            
        }
        
        IDs_pMapPoints.erase(poor_pt->PointID);
    }
    
    // delete poor_pt;
    
}


void Map::KillBadPoints()
{
    std::vector<MapPoint*> bad_pts;
    
    for (ID_PPT_ITER it=IDs_pMapPoints.begin(); it!=IDs_pMapPoints.end(); it++) {
        
        MapPoint* ppt = it->second;
        
        if (ppt->KFIDs_Points2f.size() > 2) {
            continue;
        }
        
        IDs_pMapPoints.erase(it->first);
        bad_pts.push_back(ppt);
        if (!ppt->KFIDs_Points2f.empty())
        {
            for (std::map<int, Point2f>::iterator
                 jt=ppt->KFIDs_Points2f.begin(); jt!=ppt->KFIDs_Points2f.end(); jt++)
            {
                int kf_id = jt->first;
                IDs_pKeyFrames[kf_id]->vpMapPoints.erase(ppt);
            }
        }
        
    }
    
    
    // for (size_t i=0; i<bad_pts.size(); i++) delete[] bad_pts[i];
    
}



void Map::BundleAdjustAll(bool useImu, bool rob, int maxKFid)
{
    // ----- Construct the sets of keyframes/points to be adjusted -----
    std::set<KeyFrame*> sKFs;
    std::set<KeyFrame*> sFixed;
    std::set<MapPoint*> sMpts;
    
    sFixed.insert(IDs_pKeyFrames[0]);
    
    // --- KeyFrames ---
    KeyFrame* pkf;
    for (ID_PKF_ITER it=IDs_pKeyFrames.begin(); it!=IDs_pKeyFrames.end(); it++) {
        pkf = it->second;
        if (pkf->FrameID > maxKFid) {
            continue;
        }
        
        if (pkf->vpMapPoints.size() > 0) {
            sKFs.insert(pkf);
        }
    }
    
    // --- Max map point id ---
    size_t maxMPTid = 0;
    pkf = IDs_pKeyFrames[maxKFid];
    for (std::set<MapPoint*>::iterator it = pkf->vpMapPoints.begin();
         it != pkf->vpMapPoints.end();
         it++)
    {
        if ((*it)->PointID > maxMPTid) {
            maxMPTid = (*it)->PointID;
        }
    }
    
    
    
    // --- MapPoints ---
    MapPoint* ppt;
    for (ID_PPT_ITER it=IDs_pMapPoints.begin(); it!=IDs_pMapPoints.end(); it++) {
        ppt = it->second;
        if (ppt->PointID > maxMPTid) {
            continue;
        }
        
        if (ppt->KFIDs_Points2f.size() > 1) {
            sMpts.insert(ppt);
        }
        
    }
    
    BundleAdjust(sKFs, sFixed, sMpts, useImu, rob, maxKFid, maxMPTid);
    
}

    


void Map::BundleAdjust(std::set<KeyFrame *> &sKFs, std::set<KeyFrame *> &sFix, std::set<MapPoint *> &sMpts, bool useImu, bool rob, int maxKFid, size_t maxMPTid)
{
    Optimizer Newton;
    
    std::map<MapPoint*, size_t> mpt_bid;
    std::map<size_t, MapPoint*> bid_mpt;
    std::map<KeyFrame*, size_t> kf_bid;
    std::map<size_t, KeyFrame*> bid_kf;
    
    // Add keyframes' poses to the bundle adjuster. Fisrt fixed, then non-fixed
    size_t bundle_id;
    KeyFrame* pkf;
    MapPoint* ppt;
    
    for (std::set<KeyFrame*>::iterator it=sKFs.begin(); it!=sKFs.end(); it++) {
        pkf = *it;
        
        bundle_id = Newton.addCamera(pkf->R_Cw, pkf->t_Wc, pkf->v_Wc);
        kf_bid[pkf] = bundle_id;
        bid_kf[bundle_id] = pkf;
    }
    
    
    // Add points' 3D position
    for (std::set<MapPoint*>::iterator it=sMpts.begin(); it!=sMpts.end(); it++) {
        ppt = *it;
        
        bundle_id = Newton.addPoint(ppt->XW);
        mpt_bid[ppt] = bundle_id;
        bid_mpt[bundle_id] = ppt;
    }
    
    
    // Add all the MapPoints' measurements
    Point2f pt2f;
    size_t bid_for_kf, bid_for_pt;
    int frame_id;
    
    for (std::set<MapPoint*>::iterator it=sMpts.begin(); it!=sMpts.end(); it++) {
        
        ppt = *it;
        
        bid_for_pt = mpt_bid[ppt];
        
        
        for (std::map<int, Point2f>::iterator jt=ppt->KFIDs_Points2f.begin(); jt!=ppt->KFIDs_Points2f.end(); jt++) {
            frame_id = jt->first;
            pkf = IDs_pKeyFrames[frame_id];
            
            if (sKFs.count(pkf) < 1) {
                continue;
            }
            
            bid_for_kf = kf_bid[pkf];
            
            pt2f = jt->second;
            
            Newton.addMeas(bid_for_kf, bid_for_pt, pt2f.x, pt2f.y, rob);
            
        }
    }
    
    // Set the fixed camera pose
    for (std::set<KeyFrame*>::iterator it=sFix.begin(); it!=sFix.end(); it++) {
        
        bid_for_kf = kf_bid[*it];
        
        Newton.fixACamera(bid_for_kf);
        
    }
    
    
    // Add ImuFactors
    if (useImu) {
        ImuFactor* p_imu;
        
        for (ID_PIMU_ITER it = IDs_pImuFactors.begin(); it != IDs_pImuFactors.end(); it++) {
            
            p_imu = it->second;
            int b_frid = p_imu->begin_frame_id;
            int e_frid = p_imu->end_frame_id;
            
            KeyFrame* b_pkf = IDs_pKeyFrames[b_frid];
            KeyFrame* e_pkf = IDs_pKeyFrames[e_frid];
            
            if (kf_bid.count(b_pkf) < 1 || kf_bid.count(e_pkf) < 1) {
                continue;
            }
            
            size_t b_bid_for_kf = kf_bid[b_pkf];
            size_t e_bid_for_kf = kf_bid[e_pkf];
            
            Newton.addImuMeas(b_bid_for_kf, e_bid_for_kf, p_imu);
        }
    }
    
    
    // Run the optimizer
    Newton.work();
    
    
    // Save the optimized results
    std::vector<Eigen::Vector3d> v_pts;
    std::vector<Eigen::Matrix<double, 9, 1> > v_poses;
    
    Newton.tellMe(v_pts, v_poses);
    
    for (size_t i=0; i<v_pts.size(); i++) {
        ppt = bid_mpt[i];
        ppt->XW = v_pts[i];
    }
    
    Eigen::Matrix3d R_new_old = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_new_old = Eigen::Vector3d::Zero();
    for (size_t i=0; i<v_poses.size(); i++) {
        pkf = bid_kf[i];
        
        if (pkf->FrameID == maxKFid)
        {
            Eigen::Matrix3d R_c_old = pkf->R_Cw;
            Eigen::Vector3d t_old_c = pkf->t_Wc;
            pkf->SetPoseNine(v_poses[i]);
            Eigen::Matrix3d R_new_c = pkf->R_Cw.transpose();
            Eigen::Vector3d t_new_c = pkf->t_Wc;
            
            R_new_old = R_new_c * R_c_old;
            t_new_old = t_new_c - R_new_old * t_old_c;
            
            continue;
        }
        pkf->SetPoseNine(v_poses[i]);
    }
    
    for (ID_PKF_ITER it = IDs_pKeyFrames.begin();
         it != IDs_pKeyFrames.end();
         it++)
    {
        pkf = it->second;
        if (pkf->FrameID > maxKFid)
        {
            Eigen::Matrix3d R_c_old = pkf->R_Cw;
            Eigen::Vector3d t_old_c = pkf->t_Wc;
            pkf->R_Cw = (R_c_old * (R_new_old.transpose()));
            pkf->t_Wc = R_new_old * t_old_c + t_new_old;
        }
    }
    
    for (ID_PPT_ITER it=IDs_pMapPoints.begin(); it != IDs_pMapPoints.end(); it++) {
        ppt = it->second;
        if (ppt->PointID > maxMPTid) {
            Eigen::Vector3d xw = ppt->XW;
            ppt->XW = R_new_old*xw + t_new_old;
        }
        
    }
    
    
    
    // Delete the outlier
    Eigen::Vector3d xc;
    double residual;
    std::vector<int> kfids_tbd;     // to be deleted
    for (size_t i=0; i<v_pts.size(); i++) {
        ppt = bid_mpt[i];
        
        kfids_tbd.clear();
        
        for (std::map<int, Point2f>::iterator it=ppt->KFIDs_Points2f.begin(); it!=ppt->KFIDs_Points2f.end(); it++) {
            
            frame_id = it->first;
            pkf = IDs_pKeyFrames[frame_id];
            if (pkf->FrameID > maxKFid) {
                continue;
            }
            
            pt2f = it->second;
            
            xc = pkf->R_Cw * (ppt->XW - pkf->t_Wc);
            
            double invz = 1.0 / xc(2);
            
            double upred = camfx * xc(0) * invz + camCx;
            double vpred = camfy * xc(1) * invz + camCy;
            
            residual = pow(upred - pt2f.x, 2) + pow(vpred-pt2f.y, 2);
            
            if (residual > 36) {
                pkf->vpMapPoints.erase(ppt);
                kfids_tbd.push_back(frame_id);
            }
        }
        
        for (size_t j=0; j<kfids_tbd.size(); j++) {
            ppt->KFIDs_Points2f.erase(kfids_tbd[j]);
        }
    }

}








