//
//  Optimizer.hpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/10/19.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef Optimizer_hpp
#define Optimizer_hpp

#include <stdio.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/normal_prior.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ImuFactor.hpp"

using namespace ceres;

const double fx = 458.65488     /2.0;
const double fy = 457.296696    /2.0;
const double Cx = 367.2158      /2.0;
const double Cy = 248.375341    /2.0;
const double g = 9.8;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

struct ReproError {
    ReproError(float u, float v):u_(u), v_(v) {}
    
    template<typename T>
    bool operator()(const T* camera,
                    const T* point,
                    T* residuals) const {
        T ap[3], p[3];
        ap[0] = point[0] - camera[3];
        ap[1] = point[1] - camera[4];
        ap[2] = point[2] - camera[5];
        
        ceres::AngleAxisRotatePoint(camera, ap, p);     // rotaion
        
        T iz = T(1.0) / p[2];
        T pu = fx * iz * p[0] + Cx;
        T pv = fy * iz * p[1] + Cy;
        
        residuals[0] = pu - T(u_);
        residuals[1] = pv - T(v_);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const float u, const float v) {
        return (new ceres::AutoDiffCostFunction<ReproError, 2, 6, 3>(
                    new ReproError(u, v) ) );
    }
    
private:
    float u_;
    float v_;
};



struct ImuError {
    ImuError(ImuFactor* pimuf, double t1, double t2)
    {
        pp_ = pimuf->pp ;   vv_ = pimuf->vv;
        Dt_ = t2-t1;    hDt2_ = 0.5*Dt_*Dt_;
        
        Eigen::Quaterniond qq = pimuf->qq;
        iqq_ << qq.w(), -qq.vec();
        
        Eigen::MatrixXd R9 = pimuf->RR.block(1,1,9,9);
        A = (R9.inverse()).llt().matrixL().transpose();
    }
    
    template<typename T>
    bool operator()(const T* cam1,
                    const T* cam2,
                    const T* vel1,
                    const T* vel2,
                    T* residuals) const {
        
        // Relative quaternion
        T q1[4], q2[4], q21[4];  // cam[0~2] is angleaxis, for R_Cw or q_Wc
        
        AngleAxisToQuaternion(cam1, q1);    // q_W_c1
        AngleAxisToQuaternion(cam2, q2);    // q_W_c2
        
        T iq2[4] = {q2[0], -q2[1], -q2[2], -q2[3]}; // q_C2_w
        
        QuaternionProduct(q1, iq2, q21);    // q_C2_c1
        
        
        // Relative position
        T Dpw[3], Dp12[3];
        Dpw[0] = cam2[3] - cam1[3] - vel1[0]*Dt_;
        Dpw[1] = cam2[4] - cam1[4] - vel1[1]*Dt_;
        Dpw[2] = cam2[5] - cam1[5] - vel1[2]*Dt_ + hDt2_*g;
        
        AngleAxisRotatePoint(cam1, Dpw, Dp12);
        
        
        // Relative velocity
        T Dvw[3], Dv12[3];
        Dvw[0] = vel2[0] - vel1[0];
        Dvw[1] = vel2[1] - vel1[1];
        Dvw[2] = vel2[2] - vel1[2] + g*Dt_;
        
        AngleAxisRotatePoint(cam1, Dvw, Dv12);
        
        
        // errors
        T resi_q[4];
        T iqq[4] = {T(iqq_(0)), T(iqq_(1)), T(iqq_(2)), T(iqq_(3))};
        QuaternionProduct(q21, iqq, resi_q);
        
        T resi[9];
        resi[0] = resi_q[1];
        resi[1] = resi_q[2];
        resi[2] = resi_q[3];
        
        resi[3] = Dp12[0] - pp_(0);
        resi[4] = Dp12[1] - pp_(1);
        resi[5] = Dp12[2] - pp_(2);
        
        resi[6] = Dv12[0] - vv_(0);
        resi[7] = Dv12[1] - vv_(1);
        resi[8] = Dv12[2] - vv_(2);
        
        
        // multiplied by the stiffness matrix
        for(int i=0; i<9; i++) {
            residuals[i] = T(0);
            for(int j=i; j<9; j++) {
                residuals[i] += T(A(i,j)) * resi[j];
            }
        }
        
        return true;
    }
    
    
private:
    
    Eigen::Vector3d pp_;
    Eigen::Vector4d iqq_;
    Eigen::Vector3d vv_;
    Eigen::MatrixXd A;
    double Dt_;
    double hDt2_;
    
};

 

class Optimizer
{
public:
    
    Optimizer();
    
    size_t addCamera(Eigen::Matrix3d& R_Cw, Eigen::Vector3d& t_Wc, Eigen::Vector3d& v_Wc);
    
    size_t addPoint(Eigen::Vector3d& x_W);
    
    void addMeas(size_t bid_for_kf, size_t bid_for_pt, float u, float v, bool rob);
    
    void addImuMeas(size_t bid_for_kf1, size_t bid_for_kf2, ImuFactor* p_imufac);
    
    void fixACamera(size_t bid_for_kf);
    
    void work();
    
    void tellMe(std::vector<Eigen::Vector3d> &v_pts, std::vector<Eigen::Matrix<double, 9, 1> > &v_poses);
    
private:
    Problem problem;
    
    ceres::Solver::Options options;
    
    std::vector<Eigen::Vector3d> vPoints;
    std::vector<Vector6d> vCameras;
    std::vector<Eigen::Vector3d> vVels;
    
};

#endif /* Optimizer_hpp */

