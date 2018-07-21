//
//  ImuFactor.hpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/10/22.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef ImuFactor_hpp
#define ImuFactor_hpp

#include <stdio.h>
#include <Eigen/Dense>

class ImuFactor
{
public:
    
    ImuFactor(Eigen::Vector3d& pap, Eigen::Quaterniond& qaq, Eigen::Vector3d& vav, Eigen::Matrix<double, 10, 10>& RaR, int i, int j);
    
    int begin_frame_id;
    int end_frame_id;
    
    Eigen::Vector3d pp;
    Eigen::Quaterniond qq;
    Eigen::Vector3d vv;
    Eigen::Matrix<double, 10, 10> RR;
    
};

#endif /* ImuFactor_hpp */
