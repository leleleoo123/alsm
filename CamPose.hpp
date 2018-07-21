//
//  CamPose.hpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/9/23.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef CamPose_hpp
#define CamPose_hpp

#include <stdio.h>

class CamPose {
public:
    CamPose(int iStep, int num_features);
    
    int FrameID;
    int Count;
    
};


#endif /* CamPose_hpp */
