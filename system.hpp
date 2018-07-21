//
//  system.hpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/9/21.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef system_hpp
#define system_hpp

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <set>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include "camera.hpp"
#include "Feature.hpp"
#include "CamPose.hpp"

#include "Map.hpp"
#include "KeyFrame.hpp"
#include "MapPoint.hpp"
#include "ImuFactor.hpp"

#define CHI2_095 5.9915
#define CHI2_099 9.2103

using namespace cv;

typedef Eigen::Vector4d v4d;
typedef Eigen::Vector3d v3d;

class system
{
public:
    
    system(bool half_resolution);
    
    int imu_begin;
    int grt_begin;
    int first_frame;
    int total_frames;
    
    Eigen::Quaterniond q_Sc;
    Eigen::Quaterniond q_Cs;
    Eigen::Matrix3d R_Cs;
    Eigen::Matrix4d Bob_dqp;
    
    std::vector<v3d> map_points;
    
    v3d b_a;
    v3d b_w;
    
    void LoadParameters(std::string& dataset);
    
    void Initialize(v4d& init_q, v3d& init_v, v3d& bias_w, v3d& bias_a);
    
    void Propagate(std::vector<v3d>& wms, std::vector<v3d>& ams, int iStep);
    
    void ManageMap(Mat& I, int iStep);
    
    void SearchMatches(Mat& I);
    
    void BigUpdate(int iStep);
    
    void GetXrqv(Eigen::VectorXd& Xrqv);
        
    void DrawOnImage(Mat &I);
    
    void AddFirstKeyFrame(int istep0);
    
    void PrintMapInfo();
    
    void PrintPoses(v3d& init_p, std::vector<v3d> &poses, std::vector<v3d>& points, bool toFile);
    
    void FinalOptimization(bool useImu, bool robus, int maxKFid);
    
    void IHaveALoop(Mat &Iq, Mat &Im, int iquery, int imatch, float du, float dv);
    
    void Fuse(int neighborThresh, int maxKFid);
    
    void GetActivePoints(std::vector<v3d>& active_points);
    
    std::vector<Eigen::Matrix<double, 7, 1> > ekf_poses;
    std::vector<bool> is_keyframes;
    
private:
    
    cv::Mat LastImage;
    
    size_t PointIDGenerator;
    Map* mpMap;
    KeyFrame* mpCurrKF;
    
    bool Half_Res;
    
    int IWantYou;
    
    int fast_threshold;
    
    double sigma_gyro;
    double sigma_accel;
    double sigma_pixel;
    double imu_rate;
    double deltat;
    double gravity;
    camera cam;
    
    double ncc_threshold;
    double zncc_threshold;
    double ssd_threshold;
    
    int hps_ini; // half patch size when initializing
    int hps_mat; // half patch size when matching
    int edge_band;
    double max_angle; // camera
    
    double median_inverse_depth;
    double max_inverse_depth;
    double min_inverse_depth;
    double prior_inverse_depth;
    bool   well_estimated;
    
    Eigen::VectorXd x_k_k;
    Eigen::MatrixXd P_k_k;
    
    std::vector<MapPoint*> mpts_info;
    std::vector<Feature> features_info;
    std::vector<CamPose> poses_info;
    std::vector<int> entries[2];
    
    
    void DeleteFeatures();
    
    void PreIntegrate(std::vector<v3d>& wms, std::vector<v3d>& ams, v3d& pp, Eigen::Quaterniond& qq, v3d& vv, Eigen::Matrix<double, 10, 10>& RR);
    
    void CookImuFactors(std::vector<v3d>& wms, std::vector<v3d>& ams, v3d& pp, Eigen::Quaterniond& qq, v3d& vv, Eigen::Matrix<double, 10, 10>& RR);
    
    void SearchNewFeatures(Mat& I, int numWanted, std::vector<Point2f>& newFeatures);
    
    void AddToFilter(std::vector<Point2f>& newFeatures, double initial_rho, double std_rho);
    
    void AddToInfos(std::vector<Point2f>& newFeatures, Mat& I, int iStep);
    
    void UpdateEntries();
    
    void Calculate_little_hs();
    
    void Calculate_big_Hs();
    
    void Calculate_Ss();
    
    void NccMatching(Mat& I);
    
    void OnePointRansac(double palosf);
    
    void OnePointUpdateState( Eigen::VectorXd &xNew, int iFeature);
    
    void ComeOnGuys(Eigen::VectorXd &xNew, std::vector<int>& posis_cp, std::vector<int>& iInliers);
    
    void EkfUpdateLowInnos();
    
    void EkfUpdateHighInnos();
    
    void EkfUpdateAll();
    
    void EkfUpdate(std::vector<int>& iFeatures, bool isLow);
    
    void RescueHighInnos();
    
    void EstimateMedianInverseDepth();
    
};

#endif /* system_hpp */
