//
//  camera.cpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/9/21.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#include "camera.hpp"

void camera::SetIntrinsics(int width, int height, double focalx, double focaly, double ppx, double ppy)
{
    nCols = width;
    nRows = height;
    fx = focalx;
    fy = focaly;
    Cx = ppx;
    Cy = ppy;
}
