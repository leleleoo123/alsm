//
//  Optimizer.cpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/10/19.
//  Copyright © 2016年 谭智丹. All rights reserved.
//


#include "Optimizer.hpp"


Optimizer::Optimizer()
{
    vPoints.clear();
    vCameras.clear();
    
    // Options
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // SPARSE_SCHUR, DENSE_SCHUR, SPARSE_NORMAL_CHOLESKY, DENSE_QR;
    
    options.sparse_linear_algebra_library_type = ceres::CX_SPARSE; // SUITE_SPARSE, CX_SPARSE
    
    options.trust_region_strategy_type = ceres::DOGLEG; // options are: LEVENBERG_MARQUARDT, DOGLEG
    
    options.num_linear_solver_threads = 1;
    
    options.minimizer_progress_to_stdout = true;
    
    options.max_num_iterations = 5;
    
    options.num_threads = 1;
    
}


size_t Optimizer::addCamera(Eigen::Matrix3d &R_Cw, Eigen::Vector3d &t_Wc, Eigen::Vector3d& v_Wc)
{
    size_t n = (int)vCameras.size();
    
    Eigen::AngleAxisd axi(R_Cw);
    Eigen::Vector3d r, t;
    
    r = axi.angle() * axi.axis();
    
    Vector6d cam;
    cam << r,t_Wc;
    
    vCameras.push_back(cam);
    vVels.push_back(v_Wc);
    
    return n;
    
}


size_t Optimizer::addPoint(Eigen::Vector3d &x_W)
{
    size_t n = (int)vPoints.size();
    
    Eigen::Vector3d x = x_W;
    
    vPoints.push_back(x_W);
    
    return n;
}


void Optimizer::addMeas(size_t bid_for_kf, size_t bid_for_pt, float u, float v, bool rob)
{
    CostFunction* cost_func = ReproError::Create(u, v);
    
    if (rob) {
        LossFunction* loss_func = new HuberLoss(2.45);
        problem.AddResidualBlock( cost_func, loss_func, &(vCameras[bid_for_kf][0]), &(vPoints[bid_for_pt][0]) );
    } else {
        problem.AddResidualBlock( cost_func, NULL, &(vCameras[bid_for_kf][0]), &(vPoints[bid_for_pt][0]) );
    }
    
}



void Optimizer::addImuMeas(size_t bid_for_kf1, size_t bid_for_kf2, ImuFactor *imufac)
{
    
    CostFunction* cost_function = new AutoDiffCostFunction<ImuError, 9, 6, 6, 3, 3>(new ImuError(imufac, 0, 0.05));
    
    problem.AddResidualBlock(cost_function, NULL, &(vCameras[bid_for_kf1](0)),
                                                  &(vCameras[bid_for_kf2](0)),
                                                  &(vVels[bid_for_kf1](0))   ,
                                                  &(vVels[bid_for_kf2](0))  );

}




void Optimizer::fixACamera(size_t bid_for_kf)
{
     problem.SetParameterBlockConstant( &(vCameras[bid_for_kf][0]) );
    
}



void Optimizer::work()
{
    
    ceres::Solver::Summary summery;
    
    ceres::Solve(options, &problem, &summery);
    
    std::cout << summery.FullReport() << "\n";
}



void Optimizer::tellMe(std::vector<Eigen::Vector3d> &v_pts, std::vector<Eigen::Matrix<double, 9, 1> > &v_poses)
{
    v_pts = vPoints;
    
    Eigen::Matrix<double, 9, 1> apose;
    for (size_t i=0; i<vCameras.size(); i++) {
        apose << vCameras[i], vVels[i];
        v_poses.push_back(apose);
    }

}
