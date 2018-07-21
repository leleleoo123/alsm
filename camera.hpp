//
//  camera.hpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/9/21.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef camera_hpp
#define camera_hpp

#include <stdio.h>

class camera
{
public:
    void SetIntrinsics(int width, int height, double focalx, double focaly, double ppx, double ppy);
    
    int nCols;
    int nRows;
    double fx;
    double fy;
    double Cx;
    double Cy;
};

#endif /* camera_hpp */
