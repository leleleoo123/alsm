//
//  system.cpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/9/21.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#include "system.hpp"
#include "math_operations.h"


system::system(bool half_resolution):Half_Res(true) {
    Half_Res = half_resolution;
    PointIDGenerator = 0;
    mpMap = new Map();
}

void system::LoadParameters(std::string& dataset)
{
    //std::string config_file = "/Users/tanzhidan/Documents/data/euroc/config.yaml";
    
    std::string config_file = "/Users/tanzhidan/Documents/data/euroc/config.yaml";
    FileStorage fs(config_file, FileStorage::READ);
    
    if (!fs.isOpened()) {
        std::cout << "Can not open config file!" << std::endl;
        waitKey();
    }
    
    fs["IWantYou"] >> IWantYou;
    fs["fast_threshold"] >> fast_threshold;
    
    double c_sigma_g, c_sigma_a;
    fs["sigma_gyro"]  >> c_sigma_g;  // continuous
    fs["sigma_accel"] >> c_sigma_a;
    fs["imu_rate"]    >> imu_rate;
    deltat = 1.0 / imu_rate;
    sigma_gyro = c_sigma_g * std::sqrt(imu_rate); // discrete
    sigma_accel = c_sigma_a * std::sqrt(imu_rate);
    
    fs["sigma_pixel"] >> sigma_pixel;
    
    fs["gravity"] >> gravity;
    fs["hps_ini"] >> hps_ini;
    fs["hps_mat"] >> hps_mat;
    edge_band = MAX(21, hps_ini+2);
    
    fs["ncc_threshold"] >> ncc_threshold;
    fs["zncc_threshold"] >>zncc_threshold;
    fs["ssd_threshold"] >> ssd_threshold;
    
    fs["prior_inverse_depth"] >> prior_inverse_depth;
    fs["max_inverse_depth"] >> max_inverse_depth;
    fs["min_inverse_depth"] >> min_inverse_depth;
    well_estimated = false;
    median_inverse_depth = prior_inverse_depth;
    
    int imWidth, imHeight;
    double fx, fy, cx, cy;
    fs["max_angle"] >> max_angle;
    fs["image_width"]  >> imWidth;
    fs["image_height"] >> imHeight;
    fs["fx"] >> fx;
    fs["fy"] >> fy;
    fs["cx"] >> cx;
    fs["cy"] >> cy;
    
    if (Half_Res) {
        fx /= 2.0;
        fy /= 2.0;
        cx /= 2.0;
        cy /= 2.0;
        imWidth  /= 2;
        imHeight /= 2;
        hps_mat  /= 2;
        hps_ini  /= 2;
    }
    
    cam.SetIntrinsics(imWidth, imHeight, fx, fy, cx, cy);
    
    Mat q;
    fs["q_Sc"] >> q;
    q_Sc = Eigen::Quaterniond(q.at<double>(0,0), q.at<double>(1,0), q.at<double>(2,0), q.at<double>(3,0));
    q_Sc.normalize();
    q_Cs = q_Sc.inverse();
    R_Cs = q_Sc.matrix();
    
    Eigen::Matrix4d bo_dqp, bu_dqp;
    dqp_by_dq(q_Cs, bo_dqp);
    dqp_by_dp(q_Sc, bu_dqp);
    Bob_dqp = bo_dqp * bu_dqp;
    
    FileNode fn = fs[dataset];
    fn["imu_begin"] >> imu_begin;
    fn["grt_begin"] >> grt_begin;
    fn["first_frame"] >> first_frame;
    fn["total_frames"] >> total_frames;
    
}



void system::Initialize(v4d &vq, v3d &init_v, v3d& bias_w, v3d& bias_a)
{
    b_w = bias_w;
    b_a = bias_a;
    
    Eigen::Quaterniond q_C0_w;
    q_C0_w = Eigen::Quaterniond(vq(0), vq(1), vq(2), vq(3)) * q_Cs;
    
    x_k_k.resize(10);
    x_k_k << 0,0,0, q_C0_w.w(), q_C0_w.vec(), init_v;
    
    P_k_k.resize(10, 10);
    P_k_k.setZero();
    
    for (int i=0; i<3; i++)  P_k_k(i , i) = 1e-8;
    for (int i=3; i<7; i++)  P_k_k(i , i) = 1e-6;
    for (int i=7; i<10; i++) P_k_k(i , i) = 9e-6;
    
    Eigen::Matrix<double, 7, 1> apose;
    apose = x_k_k.head(7);
    ekf_poses.push_back(apose);
    
    is_keyframes.push_back(true);
}



void system::Propagate(std::vector<v3d> &wms, std::vector<v3d> &ams, int iStep)
{
    v3d pp, vv, pap2, vav2;
    Eigen::Quaterniond qq, qaq2;
    Eigen::Matrix<double, 10, 10> RR, RaR2;
    
    
    // ---pre-integrate the IMU readings---
    PreIntegrate(wms, ams, pp, qq, vv, RR);
    
    CookImuFactors(wms, ams, pap2, qaq2, vav2, RaR2);
    ImuFactor* pImuFac = new ImuFactor(pap2, qaq2, vav2, RaR2, iStep-1, iStep);
    mpMap->AddImuFactor(pImuFac);
    
    
    // ---progate using the preintegrated IMU measurements---
    Eigen::VectorXd Xv = x_k_k.head(10);
    v3d r_W = Xv.head(3);
    Eigen::Quaterniond q_C1_w(Xv(3), Xv(4), Xv(5), Xv(6));
    v3d v_W = Xv.tail(3);
    
    Eigen::Quaterniond q_S1_w = q_C1_w * q_Sc;
    Eigen::Matrix3d R_W_s1 = q_S1_w.matrix();
    Eigen::Quaterniond q_C2_c1 = q_Sc * (qq * q_Cs);
    
    v3d g(0, 0, -gravity);
    double DT = deltat * double(wms.size());
    
    // State---
    r_W += (v_W*DT + 0.5*g*DT*DT + R_W_s1*pp);
    v_W += (g*DT + R_W_s1*vv);
    Eigen::Quaterniond q_C2_w = q_C1_w * q_C2_c1;
    
    Xv << r_W, q_C2_w.w(), q_C2_w.vec(), v_W;
    x_k_k.head(10) = Xv;  /*----x_k_k----*/
    
    // ---Covariance
    Eigen::Matrix<double, 10, 10> G, F;
    G.setZero();
    F.setIdentity();
    
    // G
    Eigen::Matrix4d q_par_qq, tempM;
    dqp_by_dq(q_Cs, q_par_qq);
    dqp_by_dp(q_S1_w, tempM);
    q_par_qq = q_par_qq * tempM;
    
    G.topLeftCorner(3, 3) = R_W_s1;
    G.block(3, 3, 4, 4) = q_par_qq;
    G.bottomRightCorner(3, 3) = R_W_s1;
    
    // F
    Eigen::Matrix3d Rq0, Rqx, Rqy, Rqz;
    dR_by_dq(q_C1_w, Rq0, Rqx, Rqy, Rqz);
    
    v3d pp_C1, vv_C1;
    Eigen::Matrix<double, 3, 4> r_par_q, v_par_q;
    pp_C1 = R_Cs * pp;
    vv_C1 = R_Cs * vv;
    r_par_q << Rq0*pp_C1, Rqx*pp_C1, Rqy*pp_C1, Rqz*pp_C1;
    v_par_q << Rq0*vv_C1, Rqx*vv_C1, Rqy*vv_C1, Rqz*vv_C1;
    
    dqp_by_dq(q_C2_c1, tempM);
    
    F.block(0, 3, 10, 4) << r_par_q, tempM, v_par_q;
    F(0,7) = DT; F(1,8) = DT; F(2,9) = DT;
    
    if (x_k_k.size() > 10) {
        int yN = int(x_k_k.size()) - 10;
        Eigen::MatrixXd P_xv_y = F * (P_k_k.topRightCorner(10, yN));
        Eigen::Matrix<double, 10, 10> P_xv = P_k_k.topLeftCorner(10, 10);
        
        P_k_k.topLeftCorner(10, 10) = F * P_xv * F.transpose() + G * RR * G.transpose();
        P_k_k.topRightCorner(10, yN) = P_xv_y;
        P_k_k.bottomLeftCorner(yN, 10) = P_xv_y.transpose();
    } else {
        P_k_k = F * P_k_k * F.transpose() + G * RR * G.transpose();
    }
    
    P_k_k = (P_k_k + P_k_k.transpose()) / 2.0;
    
}


void system::AddFirstKeyFrame(int istep0)
{
    Eigen::Matrix<double, 10, 1> xv = x_k_k.head(10);
    
    mpCurrKF = new KeyFrame(istep0, xv);
    mpMap->AddKeyFrame(mpCurrKF);
}




void system::ManageMap(cv::Mat &img, int iStep)
{
    // Estimate median depth
    EstimateMedianInverseDepth();
    
    // 1. Delete features
    // Notice that the following terms shall change:
    // x_k_k, P_k_k, entris, features_info, poses_info
    DeleteFeatures();
    UpdateEntries();
    
    // 2. Count well-tracked features & update features_info
    int numMeasured = 0;
    for (int i=0; i<features_info.size(); i++) {
        if (features_info[i].low_innovation_inlier || features_info[i].high_innovation_inlier) {
            numMeasured++;
            //features_info[i].times_notmeasured = 0;
        } else {
            features_info[i].times_notmeasured++;
        }
        
        features_info[i].compatible = 0;
        features_info[i].hok = 0;
        features_info[i].zok = 0;
        features_info[i].low_innovation_inlier  = 0;
        features_info[i].high_innovation_inlier = 0;
        
    }
    int numWanted = IWantYou - numMeasured;
    
    if (numWanted > 1)
    {
        std::vector<Point2f> newFeatures;
        newFeatures.reserve(numWanted);
        SearchNewFeatures(img, numWanted, newFeatures);
        
        if (newFeatures.size() > 0) {
            
            // Add new features
            AddToFilter(newFeatures, median_inverse_depth, median_inverse_depth); // initial_rho, std_rho
            AddToInfos(newFeatures, img, iStep);
            UpdateEntries();
            
        }
        
        std::cout << "new features:\t" << newFeatures.size() << "\n";
    }

    
    img.copyTo(LastImage);
}




void system::SearchMatches(cv::Mat &I)
{
    //----------------------------------
    // 1. Predict camera measurements
    Calculate_little_hs();
    
    // 2. Calculate derivatives
    Calculate_big_Hs();
    
    // 3. Calculate innovations
    Calculate_Ss();
    
    
    //---------------------------------------------------
    // TO DO: warp patches (predict features' appearance)
    //---------------------------------------------------
    
    //------- Matching-------
    NccMatching(I);
    
    TermCriteria tcri = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    std::vector<size_t> vlocs;
    std::vector<Point2f> vLastPts;
    std::vector<Point2f> vThisPts;
    for (size_t i=0; i<features_info.size(); i++)
    {
        if (features_info[i].lastok && features_info[i].hok)
        {
            vlocs.push_back(i);
            vLastPts.push_back(features_info[i].LastPt);
            const Eigen::Vector2d &h = features_info[i].h;
            vThisPts.push_back(Point2f(h(0), h(1)));
        }
        features_info[i].lastok = false;
    }
    
    if (vLastPts.size() < 1)
    {
        return;
    }
    
    
    std::vector<uchar> status;
    Mat err;
    calcOpticalFlowPyrLK(LastImage, I, vLastPts, vThisPts, status, err, Size(21,21), 5, tcri, false);
    
    for (size_t i=0; i<status.size(); i++)
    {
        if (status[i])
        {
            size_t l = vlocs[i];
            Point2f pt2f = vThisPts[i];
            //features_info[l].lastok = true;
            //features_info[l].LastPt = pt2f;
            
            if (features_info[l].compatible)
            {
                continue;
            }
            
            Eigen::Vector2d z(pt2f.x, pt2f.y);
            features_info[l].z = z;
            features_info[l].zok = true;
            
            // ---Chi-Square Error Checking---
            Eigen::Vector2d nu = z - features_info[l].h;
            if (nu.transpose() * features_info[l].S.inverse() * nu < CHI2_099) {
                features_info[l].compatible = true;
            }

            
        }
    }
}


void system::BigUpdate(int iStep) {
    std::cout << "state size:\t" << x_k_k.size() << "\n";
    
    // 1-point ransac
    OnePointRansac(0.999);
    
    // Update using low innovation inliers
    EkfUpdateLowInnos();
    
    // Rescue high innovation inliers
    RescueHighInnos();
    
    // Partial update using the high innovation inliers
    EkfUpdateHighInnos();
    
    // Iterate----------
    // EkfUpdateAll();
    
    
    Eigen::Matrix<double, 7, 1> apose;
    apose = x_k_k.head(7);
    ekf_poses.push_back(apose);
    is_keyframes.push_back(true);
    
    // ---------------------map----------------------
    Eigen::Matrix<double, 10, 1> Xv = x_k_k.head(10);
    // --- keyframe ---
    mpCurrKF = new KeyFrame(iStep, Xv);
    mpMap->AddKeyFrame(mpCurrKF);
    
    // --- map points have new measurments in this keyframe---
    MapPoint* ppt;
    for (size_t i=0; i<features_info.size(); i++) {
        if (features_info[i].low_innovation_inlier || features_info[i].high_innovation_inlier) {
            ppt = mpts_info[i];
            ppt->AddMeasurement(iStep, features_info[i].z);
            mpCurrKF->AddObservation(ppt);
        }
    }
    
    // --- update map points' 3D position ---
    v3d ri, tpr;
    for (size_t i=0; i<features_info.size(); i++) {
        int et1 = entries[0][i];
        int et2 = entries[1][i];
        
        // Get inverse-depth parameters
        ri  << x_k_k[et1], x_k_k[et1+1], x_k_k[et1+2];
        tpr << x_k_k[et2], x_k_k[et2+1], x_k_k[et2+2];
        
        mpts_info[i]->SetXwInverse(ri, tpr);
    }
    
    std::cout << "map points:\t" << mpMap->MapPointsSize() << "\n";
    std::cout << "key frames:\t" << mpMap->KeyFramesSize() << "\n";
    std::cout << "mpts infos:\t" << mpts_info.size() << "\n";
    std::cout << "feat infos:\t" << features_info.size() << "\n";
}


void system::GetXrqv(Eigen::VectorXd &Xrqv) {
    
    Xrqv = x_k_k.head(10);
}



void system::DrawOnImage(cv::Mat &I) {
    
    namedWindow("ayaya", 0);
    Mat cimg;
    cvtColor(I, cimg, CV_GRAY2BGR);
    for (int i=0; i<features_info.size(); i++) {
        if (features_info[i].zok) {
            Point2f pt(features_info[i].z(0), features_info[i].z(1));
            
            line(cimg, features_info[i].LastPt, pt, Scalar(0,255,255));
            features_info[i].lastok = true;
            features_info[i].LastPt = pt;
            
            if (features_info[i].low_innovation_inlier) {
                circle(cimg, pt, 4, Scalar(0,255,0), -1);
            }
            else if(features_info[i].high_innovation_inlier) {
                circle(cimg, pt, 3, Scalar(255,0,0), -1);
            }
            else {
                circle(cimg, pt, 3, Scalar(0,0,255), -1);
            }
        }
    }
    imshow("ayaya", cimg);
}


void system::PrintMapInfo(){
    mpMap->PrintStatics();
}



void system::FinalOptimization(bool useImu, bool rob, int maxKFid) {
    mpMap->BundleAdjustAll(useImu, rob, maxKFid);
}



void system::PrintPoses(v3d& init_p, std::vector<v3d>& poses, std::vector<v3d>& points, bool toFile)
{
    std::vector<KeyFrame*> v_kfs;
    mpMap->GetAllKeyFrames(v_kfs);
    
    int id;
    KeyFrame* kf;
    Eigen::Matrix<double, 7, 1> pose;
    for (size_t i=0; i<v_kfs.size(); i++) {
        kf = v_kfs[i];
        id = kf->FrameID;
        
        Eigen::Quaterniond q(kf->R_Cw);  // q_Wc
        
        pose << q.w(), -q.vec(), kf->t_Wc;
        
        ekf_poses[id] = pose;
        is_keyframes[id] = true;
        
    }
    
    poses.clear();
    for (size_t i=0; i<ekf_poses.size(); i++) {
        pose = ekf_poses[i];
        poses.push_back(v3d(pose(4,0),pose(5,0), 0));
    }
    
    std::map<size_t, MapPoint*> ids_ppts;
    mpMap->GetAllMapPoints(ids_ppts);
    
    points.clear();
    
    for (std::map<size_t, MapPoint*>::iterator it=ids_ppts.begin(); it!=ids_ppts.end(); it++) {
        
        if (it->second->KFIDs_Points2f.size() < 3) {
            continue;
        }
        Eigen::Vector3d xw = it->second->XW;
        points.push_back(xw);
    }

    
    if (toFile)
    {
        std::ofstream fout("/Users/tanzhidan/Documents/results.csv");
        for (size_t i=0; i<ekf_poses.size(); i++) {
            pose = ekf_poses[i];
            
            for (size_t j=0; j<7; j++) {
                fout << pose(j) << ",";
            }
            fout << is_keyframes[i] << "\n";
        }
        
        fout.close();
        
        //
        std::ofstream pout("/Users/tanzhidan/Documents/points.csv");
        for (std::map<size_t, MapPoint*>::iterator it=ids_ppts.begin(); it!=ids_ppts.end(); it++) {
            
            if (it->second->KFIDs_Points2f.size() < 3) {
                continue;
            }
            
            Eigen::Vector3d xw = it->second->XW;
            
            pout << xw(0) << ",";
            pout << xw(1) << ",";
            pout << xw(2) << "\n";
        }
        
        pout.close();
        
        
        // ---
        // Save database
        FileStorage fs("/Users/tanzhidan/Documents/db.yaml", FileStorage::WRITE);
        
        int numel = 0;
        for (std::map<size_t, MapPoint*>::iterator it=ids_ppts.begin(); it!=ids_ppts.end(); it++) {
            if (it->second->KFIDs_Points2f.size() < 3) {
                continue;
            }
            
            Eigen::Vector3d xw = it->second->XW;
            
            std::stringstream ss;
            ss << "i" << numel;
            
            fs << ss.str();
            fs << "{" << "x" << xw(0) << "y" << xw(1) << "z" << xw(2);
            fs << "d" << it->second->PatchMat;
            fs << "}";
            
            numel++;
        }
        
        fs << "count" << numel;
        fs << "initx" << init_p(0);
        fs << "inity" << init_p(1);
        fs << "initz" << init_p(2);
        
        fs.release();
        
    }

}


void system::GetActivePoints(std::vector<v3d> &active_points) {
    
    active_points.clear();
    active_points.reserve(features_info.size());
    
    for (int i=0; i<features_info.size(); i++) {
        if (features_info[i].low_innovation_inlier || features_info[i].high_innovation_inlier) {
            int et0 = entries[0][i], et1 = entries[1][i];
            v3d ri = x_k_k.segment(et0, 3);
            v3d tpr = x_k_k.segment(et1, 3);
            double cosp = std::cos(tpr(1));
            v3d mi(cosp * std::sin(tpr(0)), -std::sin(tpr(1)), cosp * std::cos(tpr(0)));
            v3d rW = ri + mi / tpr(2);
            active_points.push_back(rW);
        }
    }
}


void system::IHaveALoop(cv::Mat &Iq, cv::Mat &Im, int iquery, int imatch, float du, float dv)
{
    KeyFrame* kfq = mpMap->GetOneKeyFrame(iquery);
    KeyFrame* kfm = mpMap->GetOneKeyFrame(imatch);
    
    std::vector<Point2f> pts1, pts2;
    std::vector<MapPoint*> mpts;
    std::vector<bool> from1;
    
    int S = 30;
    int L = 6;
    int SL = S+L;
    int TLP = 2*L + 1;
    int TSLP = 2*SL + 1;
    
    MapPoint* mpt;
    for (std::set<MapPoint*>::iterator it=kfm->vpMapPoints.begin(); it !=kfm->vpMapPoints.end(); it++) {
        
        mpt = *it;
        Point2f pt1 = mpt->KFIDs_Points2f[imatch];
        
        float u_pred = pt1.x + du;
        float v_pred = pt1.y + dv;
        
        if (pt1.x < L || pt1.x > cam.nCols-L-1 || pt1.y < L || pt1.y > cam.nRows-L-1) {
            continue;
        }
        
        if (u_pred < SL || u_pred > cam.nCols-SL-1 || v_pred < SL || v_pred > cam.nRows-SL-1) {
            continue;
        }
        
        
        // Ncc matching
        Mat patch1 = Im(Rect(pt1.x-L, pt1.y-L, TLP, TLP));
        Mat patch2 = Iq(Rect(u_pred-SL, v_pred-SL, TSLP, TSLP));
        
        cv::Mat scores(patch2.rows-patch1.rows+1, patch2.rows-patch1.rows+1, CV_32FC1);
        matchTemplate(patch2, patch1, scores, CV_TM_CCOEFF_NORMED);
        
        double minScore, maxScore;
        Point minLoc, maxLoc;
        minMaxLoc(scores, &minScore, &maxScore, &minLoc, &maxLoc);
        
        if (maxScore > 0.9) {
            pts1.push_back(pt1);
            pts2.push_back(Point2f(u_pred-S+maxLoc.x, v_pred-S+maxLoc.y));
            mpts.push_back(mpt);
            from1.push_back(true);
        }
    }
    
    
    for (std::set<MapPoint*>::iterator it=kfq->vpMapPoints.begin(); it !=kfq->vpMapPoints.end(); it++) {
        
        mpt = *it;
        Point2f pt2 = mpt->KFIDs_Points2f[iquery];
        
        float u_pred = pt2.x - du;
        float v_pred = pt2.y - dv;
        
        if (pt2.x < L || pt2.x > cam.nCols-L-1 || pt2.y < L || pt2.y > cam.nRows-L-1) {
            continue;
        }
        
        if (u_pred < SL || u_pred > cam.nCols-SL-1 || v_pred < SL || v_pred > cam.nRows-SL-1) {
            continue;
        }
        
        
        // Ncc matching
        Mat patch2 = Iq(Rect(pt2.x-L, pt2.y-L, TLP, TLP));
        Mat patch1 = Im(Rect(u_pred-SL, v_pred-SL, TSLP, TSLP));
        
        cv::Mat scores(patch1.rows-patch2.rows+1, patch1.rows-patch2.rows+1, CV_32FC1);
        matchTemplate(patch1, patch2, scores, CV_TM_CCOEFF_NORMED);
        
        double minScore, maxScore;
        Point minLoc, maxLoc;
        minMaxLoc(scores, &minScore, &maxScore, &minLoc, &maxLoc);
        
        if (maxScore > 0.9) {
            pts2.push_back(pt2);
            pts1.push_back(Point2f(u_pred-S+maxLoc.x, v_pred-S+maxLoc.y));
            mpts.push_back(mpt);
            from1.push_back(false);
        }
    }
    
    
    if (pts1.size() > 25) {
        // estimate essential matrix
        std::vector<Point2f> bpts1;
        std::vector<Point2f> bpts2;
        
        for (size_t i=0; i<pts1.size(); i++) {
            bpts1.push_back(Point2f((pts1[i].x-cam.Cx)/cam.fx,
                                    (pts1[i].y-cam.Cy)/cam.fy));
            bpts2.push_back(Point2f((pts2[i].x-cam.Cx)/cam.fx,
                                    (pts2[i].y-cam.Cy)/cam.fy));
        }
        
        Mat inliers;
        Mat essen = findEssentialMat(pts1, pts2, 1.0, Point2f(0,0), RANSAC, 0.999, 1.0, inliers);
        
        if (countNonZero(inliers) > 20) {
            for (int i=0; i<pts1.size(); i++) {
                if (inliers.at<bool>(i,0)) {
                    
                    // Add to map
                    mpt = mpts[i];
                    
                    if (from1[i]) {
                        mpt->AddMeasurement(iquery, Eigen::Vector2d(pts2[i].x, pts2[i].y));
                        kfq->AddObservation(mpt);
                    } else {
                        mpt->AddMeasurement(imatch, Eigen::Vector2d(pts1[i].x, pts1[i].y));
                        kfm->AddObservation(mpt);
                    }
                    
                }
            }
        }
        
    }
    
}


void system::Fuse(int neighborThresh, int maxKFid)
{
    int u_right = cam.nCols - edge_band - 1;
    int v_bottom = cam.nRows - edge_band - 1;
    int S = 2*hps_ini-2*hps_mat+1;
    int central_S = S/2;
    
    std::vector<KeyFrame*> v_kfs;
    mpMap->GetAllKeyFrames(v_kfs);
    
    std::map<std::pair<MapPoint*, MapPoint*>, int > candidates;
    
    std::set<MapPoint*> s_mpts;
    
    std::vector<MapPoint*> thiskf_mpts;
    std::vector<Point2f> pts;
    Eigen::Vector3d xc;
    KeyFrame* pkf;
    int kfid;
    for (size_t i=0; i<v_kfs.size(); i++) {
        pkf = v_kfs[i];
        kfid = pkf->FrameID;
        
        if (kfid > maxKFid) {
            break;
        }
        
        Eigen::Matrix3d Rcw = pkf->R_Cw;
        Eigen::Vector3d twc = pkf->t_Wc;
        
        // Get this keyframe's observation
        pts.clear();
        thiskf_mpts.clear();
        for (std::set<MapPoint*>::iterator it=pkf->vpMapPoints.begin(); it!=pkf->vpMapPoints.end(); it++) {
            thiskf_mpts.push_back(*it);
            pts.push_back((*it)->KFIDs_Points2f[kfid]);
        }
        
        double AMinDepth = pkf->MedianDepth * 0.6;
        double AMaxDepth = pkf->MedianDepth * 1.4;
        
        for (std::set<MapPoint*>::iterator it=s_mpts.begin(); it!=s_mpts.end(); it++) {
            
            if (pkf->vpMapPoints.count(*it)) {
                continue;
            }
            
            // Project into this keyframe
            xc = Rcw * ((*it)->XW - twc);
            
            
            if (xc(2) < AMinDepth || xc(2) > AMaxDepth || xc(0) > 3 * xc(2) || xc(1) > 3*xc(2)) {
                continue;
            }
            
            double u_pred = cam.fx * xc(0) / xc(2) + cam.Cx;
            double v_pred = cam.fy * xc(1) / xc(2) + cam.Cy;
            
            if (u_pred < edge_band || u_pred > u_right || v_pred < edge_band || v_pred > v_bottom) {
                continue;
            }
            
            // See if there exist some points arond ?
            double du, dv;
            for (size_t j=0; j<pts.size(); j++) {
                
                v3d dxx = (*it)->XW - thiskf_mpts[j]->XW;
                if (dxx.norm()>2) {
                    continue;
                }
                
                du = u_pred - pts[j].x;
                dv = v_pred - pts[j].y;
                
                if (fabs(du) > neighborThresh || fabs(dv) > neighborThresh) {
                    continue;
                }
                
                std::pair<MapPoint*, MapPoint*> acandi(*it, thiskf_mpts[j]);
                
                if (candidates.count(acandi)) {
                    candidates[acandi] += 1;
                } else {
                    candidates[acandi] = 1;
                }
                
            }
            
        }
        
        
        // Add all this keyframe's mappoints into s_mpts
        for (size_t j=0; j<thiskf_mpts.size(); j++) {
            s_mpts.insert(thiskf_mpts[j]);
        }
        
    }
    
    
    //----------------------------------------------------------------------
    int aooo=0;
    int cooo=0;
    double minScore, maxScore;
    Point minLoc, maxLoc;
    std::vector<std::pair<MapPoint*, MapPoint*> > twins;
    for (std::map<std::pair<MapPoint*, MapPoint*>, int>::iterator
         it=candidates.begin(); it!=candidates.end(); it++)
    {
        if (it->second < 4) {
            continue;
        }
        
        MapPoint* first_pt = (it->first).first;
        MapPoint* second_pt = (it->first).second;
        
        aooo++;
        
        cv::Mat scores(S, S, CV_32FC1);
        matchTemplate(second_pt->PatchIni, first_pt->PatchMat, scores, CV_TM_CCOEFF_NORMED);
        
        minMaxLoc(scores, &minScore, &maxScore, &minLoc, &maxLoc);
        
        if (maxScore > 0.9 && abs(maxLoc.x - central_S)<1.5 && abs(maxLoc.y-central_S)<1.5) {
            cooo++;
            
            twins.push_back(std::pair<MapPoint*, MapPoint*>(first_pt, second_pt));
        }
        
    }
    
    std::cout << "aooo " << aooo << "\n";
    std::cout << "cooo " << cooo << "\n";
    
    
    std::set<MapPoint*> acluster;
    std::vector<size_t> itobedel;
    
    int axiba=0;
    while (!twins.empty()) {
        
        acluster.clear();
        itobedel.clear();
        
        acluster.insert(twins[0].first);
        acluster.insert(twins[0].second);
        itobedel.push_back(0);
        
        for (size_t i=1; i<twins.size(); i++) {
            
            if (acluster.count(twins[i].first)) {
                acluster.insert(twins[i].second);
                itobedel.push_back(i);
            } else if (acluster.count(twins[i].second)) {
                acluster.insert(twins[i].first);
                itobedel.push_back(i);
            }
            
        }
        
        
        for (size_t i=itobedel.size(); i>0; i--) {
            twins.erase(twins.begin()+i-1);
        }
        
        axiba += acluster.size()-1;
        mpMap->FuseSeveralPoints(acluster);
        
    }
    
    std::cout << "axiba " << axiba << "\n";
    
}


//-----------------------------------------------------
// --- Private functions -------------------------------------
//-------------------------------------------------------------------

void system::PreIntegrate(std::vector<v3d> &wms, std::vector<v3d> &ams, v3d &pp, Eigen::Quaterniond& qq, v3d &vv, Eigen::Matrix<double, 10, 10> &RR)
{
    // -----Set F
    Eigen::Matrix<double, 10, 10> F;
    F.setIdentity();
    F(0,7) = deltat; F(1,8) = deltat; F(2,9) = deltat;
    
    
    // -----initialize---
    pp.setZero();
    qq.setIdentity();
    vv.setZero();
    RR.setZero();
    
    // -----Integrate----
    double cov_w = sigma_gyro * sigma_gyro;
    double cov_a = sigma_accel * sigma_accel;
    double deltat2 = deltat * deltat;
    double alice = cov_a * deltat2;
    v3d a, wdt, adt;
    Eigen::Quaterniond dq;
    
    
    for (int i=0; i<wms.size(); i++) {
        wdt = (wms[i] - b_w) * deltat;
        a   = ams[i] - b_a;
        adt = a * deltat;
        
        Eigen::Matrix3d R_S1_st = qq.matrix();
        v3d a_S1 = R_S1_st * a;
        vec2quat(wdt, dq);
        
        // -- Covariance propagation
        // F
        Eigen::Matrix4d q_par_q;
        dqp_by_dq(dq, q_par_q);
        
        Eigen::Matrix3d Rq0, Rqx, Rqy, Rqz;
        dR_by_dq(qq, Rq0, Rqx, Rqy, Rqz);
        
        Eigen::Matrix<double, 3, 4> v_par_q;
        v_par_q << Rq0*adt, Rqx*adt, Rqy*adt, Rqz*adt;
        
        F.block(3, 3, 4, 4) = q_par_q;
        F.block(7, 3, 3, 4) = v_par_q;
        
        // G
        Eigen::Matrix<double, 4, 3> q_par_nw, ddq_par_dv;
        dq_by_dv(wdt, ddq_par_dv);
        dqp_by_dp(qq, q_par_q);
        q_par_nw = q_par_q * ddq_par_dv * deltat;
        
        RR = F * RR * F.transpose();
        RR.block(3,3,4,4) += (cov_w * q_par_nw * q_par_nw.transpose());
        for (int k=7; k<10; k++) RR(k,k) += alice;
        
        
        // -- State propagation
        pp = pp + vv * deltat + a_S1 * deltat2 / 2.0;
        vv = vv + a_S1 * deltat;
        qq = qq * dq;
    }
}



void system::CookImuFactors(std::vector<v3d> &wms, std::vector<v3d> &ams, v3d &pp, Eigen::Quaterniond& qq, v3d &vv, Eigen::Matrix<double, 10, 10> &RR)
{
    // -----Set F
    Eigen::Matrix<double, 10, 10> F;
    F.setIdentity();
    F(4,7) = deltat; F(5,8) = deltat; F(6,9) = deltat;
    
    
    // -----initialize---
    pp.setZero();
    qq.setIdentity();
    vv.setZero();
    RR.setZero();
    
    // -----Integrate----
    double cov_w = sigma_gyro * sigma_gyro;
    double deltat2 = deltat * deltat;
    double alice = 10*sigma_accel * sigma_accel * deltat2;
    
    v3d a, wdt, adt;
    Eigen::Quaterniond dqs, dq, idq, q0(1,0,0,0);
    
    Eigen::Matrix3d Rq0, Rqx, Rqy, Rqz;
    dR_by_dq(q0, Rq0, Rqx, Rqy, Rqz);
    
    for (int i=0; i<wms.size(); i++) {
        wdt = (wms[i] - b_w) * deltat;
        a   = ams[i] - b_a;
        adt = a * deltat;
        
        Eigen::Matrix3d R_C1_ct = qq.matrix();
        v3d act = R_Cs * a * deltat;
        v3d a_C1_dt = R_C1_ct * act;
        vec2quat(wdt, dqs);
        dq = q_Sc * dqs * q_Cs;
        idq = dq.inverse();
        
        // -- Covariance propagation
        // F
        Eigen::Matrix4d par_qp_p_idq, par_qp_q_dq;;
        dqp_by_dq(dq, par_qp_q_dq);
        dqp_by_dp(idq, par_qp_p_idq);
        F.topLeftCorner(4, 4) = par_qp_q_dq * par_qp_p_idq;
        
        Eigen::Matrix<double, 3, 4> Rq_act;
        Rq_act << Rq0*act, Rqx*act, Rqy*act, Rqz*act;
        
        F.bottomLeftCorner(3, 4) = R_C1_ct * Rq_act;
        
        
        // G
        Eigen::Matrix<double, 4, 3> errqq_par_nw, par_q_theta;
        dq_by_dv(wdt, par_q_theta);
        
        errqq_par_nw = par_qp_p_idq * Bob_dqp * par_q_theta * deltat;
        
        RR = F * RR * F.transpose();
        RR.topLeftCorner(4, 4) += (cov_w * errqq_par_nw * errqq_par_nw.transpose());
        for (int k=7; k<10; k++) RR(k,k) += alice;
        
        
        // -- State propagation
        pp = pp + vv * deltat + a_C1_dt * deltat / 2.0;
        vv = vv + a_C1_dt;
        qq = qq * dq;
    }
}




void system::SearchNewFeatures(cv::Mat &I, int numWanted, std::vector<Point2f>& new_pts)
{
    if (numWanted < 1) {
        return;
    }
    
    int max_attempts = 50, attempts = 0;
    int initialized = 0;
    
    int box_w = 60, box_h = 40;     // box size
    int half_box_w = box_w / 2, half_box_h = box_h / 2;
    int inner_w = cam.nCols - 2*edge_band - box_w;
    int inner_h = cam.nRows - 2*edge_band - box_h;
    
    std::cout << inner_w << "\t" << inner_h << "\n";
    
    //-----Features already exist
    Calculate_little_hs();
    std::vector<Point2f> uv_pred;
    for (int i=0; i<features_info.size(); i++) {
        if (features_info[i].hok) {
            uv_pred.push_back(Point2f(features_info[i].h(0), features_info[i].h(1)));
        }
    }
    // Clear the predicted hs
    for (int i=0; i<features_info.size(); i++) {
        features_info[i].hok = false;
    }
        
    // It seems that we don't need to assign features into grids,
    // because for deciding whether the neighborhood is clean or not,
    // the computational cost is rather low.
    
    std::vector<KeyPoint> key_pts;
    int rand_times = 0;
    while (initialized < numWanted && attempts < max_attempts && rand_times<100) {
        
        // Generate a random point
        rand_times++;
        double ran1 = (double)rand() / RAND_MAX;
        double ran2 = (double)rand() / RAND_MAX;
        int centerx = ran1 * inner_w + edge_band + half_box_w;
        int centery = ran2 * inner_h + edge_band + half_box_h;
        
        // Is the neighborhood clean?
        bool isClean = true;
        for (int i=0; i<uv_pred.size(); i++) {
            float distx = abs(uv_pred[i].x - centerx);
            float disty = abs(uv_pred[i].y - centery);
            if (distx < half_box_w && disty < half_box_h) {
                isClean = false;
                break;
            }
        }
        
        // If clean, detect fast corners and select the strongest one
        if (isClean) {
            attempts++;
      
            int topleftx = centerx - half_box_w - 3;
            int toplefty = centery - half_box_h - 3;
            Rect roi(topleftx, toplefty, box_w+6, box_h+6);
            
            
            FAST(I(roi), key_pts, fast_threshold, true, FastFeatureDetector::TYPE_9_16);   // detect FAST corners
            
            if (!key_pts.empty()) {  // Select a keypoint with the highest Shi-Tomashi score
                int besti = 0;
                float bestScore = key_pts[0].response;
                for (int i=1; i<key_pts.size(); i++) {
                    float score = key_pts[i].response;
                    if (score > bestScore) {
                        bestScore = score;
                        besti = i;
                    }
                }
                Point2f bestPt = key_pts[besti].pt + Point2f(topleftx, toplefty);
                new_pts.push_back(bestPt);
                uv_pred.push_back(bestPt);
                initialized++;
            }
            
        } // if clean...
    } // while...
    
}



void system::AddToFilter(std::vector<Point2f> &newFeatures, double initial_rho, double std_rho)
{
    //1. x_k_k
    //2. P_k_k
    
    if (newFeatures.empty()) return;
    int N = (int)newFeatures.size();
    int N3 = 3*N;
    
    double Cx, Cy, ifx, ify;
    Cx = cam.Cx;  Cy = cam.Cy;
    ifx = 1.0 / cam.fx;
    ify = 1.0 / cam.fy;
    
    Eigen::VectorXd Xrq = x_k_k.head(7);
    v3d r_Wc = Xrq.head(3);
    Eigen::Quaterniond q_Cw(Xrq(3), Xrq(4), Xrq(5), Xrq(6));
    Eigen::Matrix3d R_Wc = q_Cw.matrix();
    Eigen::Matrix3d Rq0, Rqx, Rqy, Rqz;
    dR_by_dq(q_Cw, Rq0, Rqx, Rqy, Rqz);
    
    //-----------------------------------------------------
    Eigen::VectorXd new_tprs(N3);  // new (theta phi rho) s
    Eigen::MatrixXd dnewY_drq = Eigen::MatrixXd::Zero(N3+3, 7);
    dnewY_drq(0,0) = 1.0;
    dnewY_drq(1,1) = 1.0;
    dnewY_drq(2,2) = 1.0;
    
    Eigen::MatrixXd P_add = Eigen::MatrixXd::Zero(N3+3, N3+3);
    double cov_rho = std_rho * std_rho;
    double cov_pixel = sigma_pixel * sigma_pixel;
    
    Eigen::Matrix<double, 3, 4> dgw_dq;
    Eigen::Matrix<double, 2, 4> dthetaphi_dq;
    Eigen::Matrix<double, 2, 3> dthetaphi_dgw;
    Eigen::Matrix2d dthetaphi_duv;
    Eigen::Matrix<double, 3, 2> dgw_duv;
    
    Eigen::Matrix<double, 3, 2> tempM32;
    tempM32 << ifx, 0, 0, ify, 0, 0;
    dgw_duv = R_Wc * tempM32;
    
    
    //---------------------------------------------
    for (int i=0; i<N; i++) {
        
        // Bearing vector in camera frame
        v3d vcbear;
        vcbear << (newFeatures[i].x-Cx)*ifx,
                 (newFeatures[i].y-Cy)*ify,
                 1;
        
        // Bearing vector transformed to world frame
        v3d vwbear = R_Wc * vcbear;
        
        // Inverse parameters
        double theta, phi;
        theta = std::atan2(vwbear(0), vwbear(2));
        phi   = std::atan2(-vwbear(1), std::sqrt(pow(vwbear(0), 2) + pow(vwbear(2), 2)));
        
        //
        int i3 = i*3;
        new_tprs.segment(i3, 3) << theta, phi, initial_rho;
        
        //
        double nx = vwbear(0), ny = vwbear(1), nz = vwbear(2);
        double nxz2 = nx * nx + nz * nz;
        double nnn  = 1.0 / ((nxz2 + ny*ny)*std::sqrt(nxz2));
        dthetaphi_dgw << nz/nxz2,           0,     -nx/nxz2,
                         nx*ny*nnn,  -nxz2*nnn,   ny*nz*nnn;
        
        dgw_dq << Rq0*vcbear, Rqx*vcbear, Rqy*vcbear, Rqz*vcbear;
        
        dthetaphi_duv = dthetaphi_dgw * dgw_duv;
        
        dnewY_drq.block(i3+3, 3, 2, 4) = (dthetaphi_dgw * dgw_dq); // dthetaphi_dq
        
        P_add.block(i3+3, i3+3, 2, 2) = cov_pixel * dthetaphi_duv * dthetaphi_duv.transpose();
        P_add(i3+5,i3+5) = cov_rho;
        
    }
    
    // State vector
    int old_size = int(x_k_k.size());
    x_k_k.conservativeResize(old_size+3+N3);
    x_k_k.tail(3+N3) << r_Wc, new_tprs;
    
    // Covariance matrix
    Eigen::MatrixXd P_rq = P_k_k.topLeftCorner(7, 7);
    Eigen::MatrixXd P_temp;
    P_temp = dnewY_drq * (P_k_k.topRows(7));
    
    P_k_k.conservativeResize(x_k_k.size(), x_k_k.size());
    P_k_k.bottomLeftCorner(3+N3, old_size) = P_temp;
    P_k_k.topRightCorner(old_size, 3+N3) = P_temp.transpose();
    P_k_k.bottomRightCorner(3+N3, 3+N3) = dnewY_drq * P_rq * dnewY_drq.transpose() + P_add;
    
}



void system::AddToInfos(std::vector<Point2f> &newFeatures, cv::Mat &I, int iStep)
{
    if (newFeatures.empty()) return;
    
    int N = int(newFeatures.size());
    
    // poses_info-----------------------------
    poses_info.reserve(poses_info.size()+1);
    poses_info.push_back(CamPose(iStep, N));
    
    
    // features_info-------And MapPoints_info-----------------------
    features_info.reserve(features_info.size()+N);
    
    int l_ini = 2 * hps_ini + 1;
    int l_mat = 2 * hps_mat + 1;
    
    for (int i=0; i<N; i++) {
        //
        int u = newFeatures[i].x;
        int v = newFeatures[i].y;
        
        Mat patch_ini = I(Rect(u-hps_ini, v-hps_ini, l_ini, l_ini));
        Mat patch_mat = I(Rect(u-hps_mat, v-hps_mat, l_mat, l_mat));
        
        features_info.push_back(Feature(patch_ini, patch_mat, iStep));
        features_info.back().LastPt = Point2f(u,v);
        
        // --- map -----
        MapPoint* pMpt = new MapPoint(PointIDGenerator, patch_ini, patch_mat, iStep-1, newFeatures[i]);
        mpts_info.push_back(pMpt);
        
        mpMap->AddMapPoint(pMpt);
        mpCurrKF->AddObservation(pMpt);
        
        PointIDGenerator++;
        
    }
    
    
}


void system::UpdateEntries()
{
    if (features_info.empty()) return;
    int N = (int)features_info.size();
    
    entries[0].clear();     entries[1].clear();
    entries[0].reserve(N);  entries[1].reserve(N);
    
    int k=10, pose_k, l=0;
    for (int i=0; i<poses_info.size(); i++) {
        
        pose_k = k;
        k += 3;
        int ID = poses_info[i].FrameID;
        
        for (int j=0; j<poses_info[i].Count; j++) {
            
            if (features_info[l].FrameID != ID) {
                std::cout << "Error : In UpdateEntries function, FrameID mismatch\n";
                waitKey();
            } else {
                entries[0].push_back(pose_k);
                entries[1].push_back(k);
            }
            
            l++;
            k += 3;
        }
    }
    
    if (entries[0].size() != N) {
        std::cout << "Error : In UpdateEntries function, entries & features_info size mismatch\n";
        waitKey();
    }
}


void system::Calculate_little_hs() {
    
    double fx=cam.fx, fy=cam.fy;
    double Cx=cam.Cx, Cy=cam.Cy;
    int umax = cam.nCols-1, vmax = cam.nRows-1;
    
    v3d t_Wc = x_k_k.head(3);
    Eigen::Quaterniond q_Cw(x_k_k(3), x_k_k(4), x_k_k(5), x_k_k(6));
    Eigen::Matrix3d R_Cw = (q_Cw.matrix()).transpose();
    
    v3d ri, mi;
    double theta, phi, rho;
    for (int i=0; i<features_info.size(); i++) {
        
        int et1 = entries[0][i];
        int et2 = entries[1][i];
        
        // Get inverse-depth parameters
        ri << x_k_k[et1], x_k_k[et1+1], x_k_k[et1+2];
        theta = x_k_k[et2];
        phi   = x_k_k[et2+1];
        rho   = x_k_k[et2+2];
        if (rho < min_inverse_depth || rho > max_inverse_depth) {
            continue;
        }
        
        
        // Point3D in camera-frame (up to a scale factor : rho)
        double cphi = std::cos(phi);
        mi(0) = cphi * std::sin(theta);
        mi(1) = -std::sin(phi);
        mi(2) = cphi * std::cos(theta);
        v3d xc = R_Cw * (rho*(ri-t_Wc) + mi);
        
        if (xc(2) < 0.05 || xc(2)>10) {
            continue;
        }
        
        
        // Is it in front of the camera?
        double angle1 = fabs(std::atan2(xc(0), xc(2)));
        double angle2 = fabs(std::atan2(xc(1), xc(2)));
        if (angle1 > max_angle || angle2 > max_angle) {
            continue;
        }
        
        // Project to the image plane
        double invz = 1.0 / xc(2);
        double u = fx * xc(0) * invz + Cx;
        double v = fy * xc(1) * invz + Cy;
        
        // Is it visible in the image?
        if (u>0 && u<umax && v>0 && v<vmax) {
            features_info[i].h << u, v;
            features_info[i].hok = true;
        }
        
    } // for...
}


void system::Calculate_big_Hs() {
    
    double fx=cam.fx, fy=cam.fy;
    
    v3d t_Wc = x_k_k.head(3);
    Eigen::Quaterniond q_Cw(x_k_k(3), x_k_k(4), x_k_k(5), x_k_k(6));
    Eigen::Quaterniond q_Wc = q_Cw.inverse();
    Eigen::Matrix3d R_Cw = q_Wc.matrix();
    
    Eigen::Matrix3d Rq0, Rqx, Rqy, Rqz;
    dR_by_dq(q_Wc, Rq0, Rqx, Rqy, Rqz);
    
    
    //------
    v3d ri, mi, xw, xwm, xc;
    double theta, phi, rho;
    Eigen::Matrix<double, 2, 3> dh_by_dxc, tempM23;
    Eigen::Matrix<double, 3, 4> tempM34;
    Eigen::Matrix3d dxwm_by_dtpr;
    //----------------------------------------------------
    for (int i=0; i<features_info.size(); i++) {
        if (!features_info[i].hok) {
            continue;
        }
        
        // Entry
        int et1 = entries[0][i];
        int et2 = entries[1][i];
        
        // Get inverse-depth parameters
        ri << x_k_k[et1], x_k_k[et1+1], x_k_k[et1+2];
        theta = x_k_k[et2];
        phi   = x_k_k[et2+1];
        rho   = x_k_k[et2+2];
        
        // Some temporary variable
        double cp, sp, ct, st;
        cp = std::cos(phi);
        sp = std::sin(phi);
        ct = std::cos(theta);
        st = std::sin(theta);
        mi << cp*st, -sp, cp*ct;
        
        // Point 3D in world/camera frame (up to a scale factor)
        xw  = ri * rho + mi;
        xwm = xw - rho * t_Wc;
        xc  = R_Cw * xwm;
        double icz  = 1.0/xc(2);
        double icz2 = icz * icz;
        
        
        // --------Intermediate Derivatives----------------
        dh_by_dxc << fx*icz,    0,      -fx*icz2*xc(0),
                     0,      fy*icz,    -fy*icz2*xc(1);

        tempM23 = dh_by_dxc * R_Cw;
        
        tempM34 << Rq0*xwm, Rqx*(-xwm), Rqy*(-xwm), Rqz*(-xwm);
        
        dxwm_by_dtpr << cp*ct,  -sp*st,     ri(0)-t_Wc(0),
                        0,      -cp,        ri(1)-t_Wc(1),
                       -cp*st,  -sp*ct,     ri(2)-t_Wc(2);
        
        
        // --------Derivatives-----------------------------------
        features_info[i].H_rq  << (-rho)*tempM23, dh_by_dxc*tempM34;
        
        features_info[i].H_ri  = rho*tempM23;
        features_info[i].H_tpr = tempM23 * dxwm_by_dtpr;
    }
}



void system::Calculate_Ss() {
    
    double cov_pixel = sigma_pixel * sigma_pixel;
    Eigen::Matrix2d Ri;
    Ri << cov_pixel,    0,
          0,    cov_pixel;
    
    int lx = (int)x_k_k.size();
    
    Eigen::MatrixXd P_x_rq = P_k_k.leftCols(7);
    Eigen::Matrix<double, 2, 7> Hi_rq;
    Eigen::Matrix<double, 2, 3> Hi_ri, Hi_tpr;
    Eigen::MatrixXd P_x_ri(lx, 3), P_x_tpr(lx, 3);
    
    //--------------------------------------------------
    for (int i=0; i<features_info.size(); i++) {
        if (!features_info[i].hok) {
            continue;
        }
        
        // Entry
        int et1 = entries[0][i];
        int et2 = entries[1][i];
        
        // Get corresponding matries--------------
        P_x_ri  = P_k_k.block(0, et1, lx, 3);
        P_x_tpr = P_k_k.block(0, et2, lx, 3);
        
        Hi_rq  = features_info[i].H_rq;
        Hi_ri  = features_info[i].H_ri;
        Hi_tpr = features_info[i].H_tpr;
        
        
        // Calculat P*H'--------------------------
        Eigen::MatrixXd phT;
        phT = P_x_rq*(Hi_rq.transpose()) + P_x_ri*(Hi_ri.transpose()) + P_x_tpr*(Hi_tpr.transpose());
        
        
        // Then calculate H*phT + R
        features_info[i].S = (Hi_rq  * phT.topRows(7)
                            + Hi_ri  * phT.block(et1, 0, 3, 2)
                            + Hi_tpr * phT.block(et2, 0, 3, 2)
                            + Ri);
        
    }
}


void system::NccMatching(cv::Mat& I) {
    int im_width  = I.cols;
    int im_height = I.rows;
    int ps_mat = 2*hps_mat + 1;
    
    Eigen::Vector2d h;
    Eigen::Matrix2d S;
    int hsrsx, hsrsy;  // half search region size
    int srs;
    int minsx, minsy, maxsx, maxsy;
    
    for (int i=0; i<features_info.size(); i++) {
        if (!features_info[i].hok) {
            continue;
        }
        
        h = features_info[i].h;
        S = features_info[i].S;
        
        if (S(0,0) < 1 || S(1,1) < 1) {
            features_info[i].hok = false;
            std::cout << "how can S be negative?\n"; 
            waitKey();
        }
        
        hsrsx = ceil(2*std::sqrt(S(0,0)));
        hsrsy = ceil(2*std::sqrt(S(1,1)));
        srs = (2*hsrsx+1) * (2*hsrsy+1);
        
        if (hsrsx>100 || hsrsy>100 || hsrsx*hsrsy>4900) {
            features_info[i].too_uncertain = true;
            std::cout << "too uncertain!\n";
            continue;
        }
        
        minsx = MAX(round(h(0))-hsrsx, hps_mat);
        minsy = MAX(round(h(1))-hsrsy, hps_mat);
        maxsx = MIN(round(h(0))+hsrsx, im_width -hps_mat-1);
        maxsy = MIN(round(h(1))+hsrsy, im_height-hps_mat-1);
        if (maxsx < minsx || maxsy < minsy) {
            continue;
        }
        
        // Template
        cv::Mat patch_T = features_info[i].patch_when_matching;
        
        // Image patch to be searched
        cv::Mat patch_I = I(Rect(minsx-hps_mat, minsy-hps_mat, maxsx-minsx+ps_mat, maxsy-minsy+ps_mat));
                
        // ---Matching---
        cv::Mat scores(maxsy-minsy+1, maxsx-minsx+1, CV_32FC1);
        matchTemplate(patch_I, patch_T, scores, CV_TM_CCOEFF_NORMED);
        
        double minScore, maxScore;
        Point minLoc, maxLoc;
        minMaxLoc(scores, &minScore, &maxScore, &minLoc, &maxLoc);
        
        if (maxScore > ncc_threshold) {
            Eigen::Vector2d z(minsx+maxLoc.x, minsy+maxLoc.y);
            features_info[i].z = z;
            features_info[i].zok = true;
            
            // ---Chi-Square Error Checking---
            Eigen::Vector2d nu = z - h;
            if (nu.transpose() * S.inverse() * nu < CHI2_099) {
                features_info[i].compatible = true;
            }
        }
        
    } // for...
}


void system::OnePointRansac(double palosf) {
    
    // Max number of iteration, will be updated
    int N_hyp = 500;
    
    // Supporters' count
    int max_support = 0;
    
    //Compatible features' position in features_info
    std::vector<int> posis_cp;
    posis_cp.reserve(features_info.size());
    
    std::cout << "features_info\t" << features_info.size() << "\n";
    
    for (int i=0; i<features_info.size(); i++) {
        if (features_info[i].compatible) {
            posis_cp.push_back(i);
        }
    }
    
    int N_cp = (int)posis_cp.size();
    if (N_cp < 2) {
        return;
    }
    
    
    //----Ransac------------------------
    Eigen::VectorXd x_new(x_k_k.size());
    std::vector<int> i_inliers_best;
    
    int k=0;
    while (k<N_hyp) {
        k++;
        
        // --Randomly select a match
        double ran = (double)rand() / RAND_MAX;
        int i_rand = floor(ran*N_cp);
        int i_feature = posis_cp[i_rand];
        
        // --1-match EKF state update
        OnePointUpdateState(x_new, i_feature);
        
        // --Count supporters
        std::vector<int> i_inliers;
        i_inliers.reserve(N_cp);
        ComeOnGuys(x_new, posis_cp, i_inliers);
        int N_inliers = (int)i_inliers.size();
        
        // --Refresh
        if (N_inliers > max_support) {
            max_support = N_inliers;
            i_inliers_best = i_inliers;
            
            double epsilon = 1.0 - (double)N_inliers/N_cp;
            N_hyp = ceil(std::log(1-palosf) / std::log(epsilon));
            
        }
    } // while...
    
    //--Finally, we set the innovation inliers
    for (int i=0; i<i_inliers_best.size(); i++) {
        int i_feature = i_inliers_best[i];
        features_info[i_feature].low_innovation_inlier = 1;
    }
    
    std::cout << "low innovation inliers:\t" << max_support << "\n";
    
}


void system::OnePointUpdateState(Eigen::VectorXd &xNew, int iFeature) {
    if (!features_info[iFeature].zok) {
        std::cout << "Error: OnePointUpdateState, feature not measured\n";
        waitKey();
    }
    
    int lx = (int)x_k_k.size();
    
    // Entry
    int et1 = entries[0][iFeature];
    int et2 = entries[1][iFeature];
    
    // Get corresponding covariance block
    Eigen::MatrixXd P_x_rq = P_k_k.leftCols(7);
    Eigen::MatrixXd P_x_ri = P_k_k.block(0, et1, lx, 3);
    Eigen::MatrixXd P_x_tpr = P_k_k.block(0, et2, lx, 3);
    
    // Calculat P*H'--------------------------
    Eigen::MatrixXd phT;
    phT = P_x_rq  * (features_info[iFeature].H_rq).transpose()
        + P_x_ri  * (features_info[iFeature].H_ri).transpose()
        + P_x_tpr * (features_info[iFeature].H_tpr).transpose();
    
    Eigen::Matrix2d S = features_info[iFeature].S;
    Eigen::MatrixXd K = phT * S.inverse();
    
    xNew = x_k_k + K * (features_info[iFeature].z - features_info[iFeature].h);
    
    // Quaternion normalization
    xNew.segment(3, 4).normalize();
    
}


void system::ComeOnGuys(Eigen::VectorXd &xNew, std::vector<int>& posis_cp, std::vector<int> &iInliers)
{
    v3d t_Wc = xNew.head(3);
    Eigen::Quaterniond q_Cw(xNew(3), xNew(4), xNew(5), xNew(6));
    Eigen::Matrix3d R_Cw = q_Cw.matrix().transpose();
    
    double fx = cam.fx, fy = cam.fy;
    double Cx = cam.Cx, Cy = cam.Cy;
    
    //-------------
    v3d ri, mi;
    double theta, phi, rho;
    //------------------------------------------------
    for (int k=0; k<posis_cp.size(); k++) {
        int i_feature = posis_cp[k];
        
        // Entry
        int et1 = entries[0][i_feature];
        int et2 = entries[1][i_feature];
        
        // Get inverse-depth parameters
        ri = xNew.segment(et1, 3);
        theta = xNew[et2];
        phi   = xNew[et2+1];
        rho   = xNew[et2+2];
        
        // Point3D in camera-frame (up to a scale factor : rho)
        double cphi = std::cos(phi);
        mi << cphi * std::sin(theta),
             -std::sin(phi),
             cphi * std::cos(theta);
        v3d xc = R_Cw * (rho*(ri-t_Wc) + mi);
        
        // Projection
        double invz = 1.0 / xc(2);
        Eigen::Vector2d h;
        h << fx * xc(0) * invz + Cx,
             fy * xc(1) * invz + Cy;
        
        // Error
        double dist_pixel = (h-features_info[i_feature].z).norm();
        
        if (dist_pixel < 2) {
            iInliers.push_back(i_feature);
        }
    }
}


void system::EkfUpdateLowInnos() {
    
    // Get low innovation inlier indices in features_info
    std::vector<int> i_low_innos;
    i_low_innos.reserve(features_info.size());
    
    for (int i=0; i<features_info.size(); i++) {
        if (features_info[i].low_innovation_inlier) {
            i_low_innos.push_back(i);
        }
    }
    
    // Update
    EkfUpdate(i_low_innos, true);
    
}

void system::EkfUpdateHighInnos() {
    
    // Get high innovation inlier indices in features_info
    std::vector<int> i_high_innos;
    i_high_innos.reserve(features_info.size());
    
    for (int i=0; i<features_info.size(); i++) {
        if (features_info[i].high_innovation_inlier) {
            i_high_innos.push_back(i);
        }
    }
    
    // Update
    EkfUpdate(i_high_innos, false);
    
    std::cout << "high inliers:\t" << i_high_innos.size() << "\n";
}


void system::EkfUpdateAll() {
    
    // Get high/low innovation inlier indices in features_info
    std::vector<int> i_high_low;
    i_high_low.reserve(features_info.size());
    
    for (int i=0; i<features_info.size(); i++) {
        if (features_info[i].high_innovation_inlier || features_info[i].low_innovation_inlier) {
            i_high_low.push_back(i);
        }
    }
    
    // Update
    EkfUpdate(i_high_low, true);
}




void system::RescueHighInnos() {
    
    //-------------------------------------------
    // ------Recalculate h, H, S--------
    //---using the new x_k_k & P_k_k----------
    //--------------------------------------------
    // 1. Predict camera measurements
    Calculate_little_hs();
    
    // 2. Calculate derivatives
    Calculate_big_Hs();
    
    // 3. Calculate innovations
    Calculate_Ss();
    
    
    //---------------------------------
    //---ChiSquare Error Checking-----
    //---------------------------------
    for (int i=0; i<features_info.size(); i++) {
        if (features_info[i].zok && (!features_info[i].low_innovation_inlier)) {
            Eigen::Vector2d nu = features_info[i].z - features_info[i].h;
            Eigen::Matrix2d S = features_info[i].S;
            
            if (nu.transpose() * S.inverse() * nu < CHI2_095) {
                features_info[i].high_innovation_inlier = true;
            }
        }
    }
}


void system::EkfUpdate(std::vector<int>& iFeatures, bool isLow)
{
    if (iFeatures.empty()) return;
    int M = (int)x_k_k.size();
    int N  = (int)iFeatures.size();
    int N2 = 2*N;
    
    // Get z & h----------------------
    Eigen::VectorXd z(N2), h(N2);
    
    for (int k=0; k<iFeatures.size(); k++) {
        int i_feature = iFeatures[k];
        int k2 = 2*k;
        
        z.segment(k2, 2) = features_info[i_feature].z;
        h.segment(k2, 2) = features_info[i_feature].h;
    }
    
    
    // Compute P*H'----------------------
    Eigen::MatrixXd PHT(M, N2);
    
    for (int k=0; k<iFeatures.size(); k++) {
        int i_feature = iFeatures[k];
        int k2 = 2*k;
        
        // Entry
        int et1 = entries[0][i_feature];
        int et2 = entries[1][i_feature];
        
        //
        Eigen::MatrixXd P_HiT;
        P_HiT = P_k_k.leftCols(7)      * (features_info[i_feature].H_rq).transpose()
              + P_k_k.block(0,et1,M,3) * (features_info[i_feature].H_ri).transpose()
              + P_k_k.block(0,et2,M,3) * (features_info[i_feature].H_tpr).transpose();
        PHT.block(0, k2, M, 2) = P_HiT;
    }
    
    
    // Compute H*P*H'--------------------------
    Eigen::MatrixXd S(N2, N2);
    
    for (int k=0; k<iFeatures.size(); k++) {
        int i_feature = iFeatures[k];
        int k2 = 2*k;
        
        // Entry
        int et1 = entries[0][i_feature];
        int et2 = entries[1][i_feature];
        
        //
        Eigen::MatrixXd Hi_PHT;
        Hi_PHT = features_info[i_feature].H_rq  * PHT.topRows(7)
               + features_info[i_feature].H_ri  * PHT.block(et1, 0, 3, N2)
               + features_info[i_feature].H_tpr * PHT.block(et2, 0, 3, N2);
        
        S.block(k2, 0, 2, N2) = Hi_PHT;
    }
    
    
    // Then compute S = H*P*H' + R-----
    double cov_pixel = sigma_pixel * sigma_pixel;
    //if (!isLow) {cov_pixel *= 2;}
    for (int k=0; k<N2; k++) {
        S(k,k) += cov_pixel;
    }
    
    
    // Kalman gain K = P * H' / S--
    Eigen::MatrixXd K(M, N2);
    K = PHT * S.inverse();
    
    
    // Update state & covariance
    x_k_k += K * (z-h);
    P_k_k -= PHT * K.transpose();
    
    P_k_k = (P_k_k + P_k_k.transpose()) / 2;
    
    // Quaternion normalization
    x_k_k.segment(3, 4).normalize();
    
    // TO DO: Covariance shall change when normalizing the quaternion
    // Now we simply omit this part
    
}


void system::DeleteFeatures() {
    
    std::vector<int> i_feature_godie;
    std::vector<int> i_pose_godie;
    Eigen::VectorXi isBad = Eigen::VectorXi::Zero(x_k_k.size());
    Eigen::Vector3i one3(1,1,1);
    
    int k=10, l=0, pose_k;
    for (int i=0; i<poses_info.size(); i++) {
        int count = poses_info[i].Count;
        int ID = poses_info[i].FrameID;
        pose_k = k;
        k += 3;
        
        for (int j=0; j<count; j++) {
            if (features_info[l].FrameID != ID) {
                std::cout << "Error: In DeleteFeatures, ID mismatch\n";
                waitKey();
            }
            
            if (features_info[l].times_notmeasured > 1 || features_info[i].too_uncertain) {
                i_feature_godie.push_back(l);
                poses_info[i].Count--;
                isBad.segment(k,3) = one3;
            }
            
            l++;
            k += 3;
        }
        
        if (poses_info[i].Count < 1) {
            i_pose_godie.push_back(i);
            isBad.segment(pose_k, 3) = one3;
        }
    }
    std::cout << "godie:\t" << i_feature_godie.size() << "\n";
    
    // -----------------------------------

    // features_info
    for (size_t i=i_feature_godie.size(); i>0; i--) {
        int i_f = i_feature_godie[i-1];
        features_info.erase(features_info.begin()+i_f);
        
        mpts_info.erase(mpts_info.begin()+i_f);
        
        int et0 = entries[0][i_f], et1 = entries[1][i_f];
        
        v3d ri = x_k_k.segment(et0, 3);
        v3d tpr = x_k_k.segment(et1, 3);
        double std_rho = std::sqrt(P_k_k(et1+2, et1+2));
        if (std_rho > 0.2*tpr(2) || tpr(2)<0.05 ) {
            continue;
        }
        double cosp = std::cos(tpr(1));
        v3d mi(cosp * std::sin(tpr(0)), -std::sin(tpr(1)), cosp * std::cos(tpr(0)));
        v3d rW = ri + mi / tpr(2);
        
        map_points.push_back(rW);
        
    }
    
    // poses_info
    for (size_t i=i_pose_godie.size(); i>0; i--) {
        int i_p = i_pose_godie[i-1];
        poses_info.erase(poses_info.begin()+i_p);
    }
    
    // x_k_k
    int N = (int)x_k_k.size()-3;
    for (int i=N; i>9; i-=3) {
        if (isBad(i)>0) {
            int S = (int)x_k_k.size();
            int L = S-i-3;
            if (L > 0) {
                x_k_k.segment(i, L) = x_k_k.segment(i+3, L);
            }
            x_k_k.conservativeResize(S-3);
        }
    }
    
    // P_k_k
    for (int i=N; i>9; i-=3) {
        if (isBad(i)>0) {
            int H = (int)P_k_k.cols();
            int L = H-i-3;
            if (L > 0) {
                P_k_k.block(i, 0, L, H) = P_k_k.block(i+3,0, L,H);
                P_k_k.block(0, i, H-3, L) = P_k_k.block(0, i+3, H-3, L);
            }
            P_k_k.conservativeResize(H-3,H-3);
        }
    }
}


void system::EstimateMedianInverseDepth() {
    
    std::vector<double> rhos;
    rhos.reserve(features_info.size());
    
    for (int i=0; i<features_info.size(); i++) {
        if (features_info[i].low_innovation_inlier || features_info[i].high_innovation_inlier) {
            int et = entries[1][i] + 2;
            rhos.push_back(x_k_k(et));
        }
    }
    
    // Get new median (fused with prior information)
    double median;
    size_t size = rhos.size();
    if (size > 20) {
        
        sort(rhos.begin(), rhos.end());
        
        if (size % 2 == 0) {
            median = (rhos[size/2-1] + rhos[size/2])/2.0;
        } else {
            median = rhos[size/2];
        }
        
        if (median < max_inverse_depth && median > min_inverse_depth) {
            median = (median*(double)size + prior_inverse_depth*30.0) / ((double)size+30.0);
            well_estimated = true;
        } else {
            well_estimated = false;
            median = prior_inverse_depth;
        }
        
    } else {
        well_estimated = false;
        median = prior_inverse_depth;
    }
    
    median_inverse_depth = (median_inverse_depth + median) / 2;
    
    mpCurrKF->MedianDepth = 1.0 / median_inverse_depth;
    
    std::cout << "median:\t"<< median_inverse_depth << "\n";
}




