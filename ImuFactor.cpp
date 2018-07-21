//
//  ImuFactor.cpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/10/22.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#include "ImuFactor.hpp"

ImuFactor::ImuFactor(Eigen::Vector3d& pap, Eigen::Quaterniond& qaq, Eigen::Vector3d& vav, Eigen::Matrix<double, 10, 10>& RaR, int i, int j)
{
    begin_frame_id = i;
    end_frame_id = j;
    
    pp = pap;
    qq = qaq;
    vv = vav;
    RR = RaR;
}