//
//  write_config_parameters.h
//  ekf_slam
//
//  Created by 谭智丹 on 16/9/21.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef write_config_parameters_h
#define write_config_parameters_h

//#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

void write_parameters() {

    string filename = "/Users/tanzhidan/Documents/data/euroc/config.yaml";
    
    FileStorage fs(filename, FileStorage::WRITE);
    
    fs<<"image_width"<<752;
    fs<<"image_height"<<480;
    fs<<"fx"<<458.654880721;
    fs<<"fy"<<457.296696463;
    fs<<"cx"<<367.215803962;
    fs<<"cy"<<248.37534061;
    fs<<"imu_rate"<<200;
    fs<<"camera_rate"<<20;
    fs<<"sigma_gyro"<<12.0e-4;
    fs<<"sigma_accel"<<8.0e-3;
    
    Mat q_Sc = (Mat_<double>(4,1)<<0.712301, 0.007707, -0.010499, -0.701753);
    fs<<"q_Sc"<<q_Sc;
    
    fs<<"MH_01_easy";
    fs<<"{"<<"imu_begin"<<232;
    fs<<"grt_begin"<<16;
    fs<<"first_frame"<<24;
    fs<<"total_frames"<<3637;
    fs<<"}";
    
    fs<<"MH_02_easy";
    fs<<"{"<<"imu_begin"<<185;
    fs<<"grt_begin"<<4;
    fs<<"first_frame"<<19;
    fs<<"total_frames"<<2999;
    fs<<"}";
    
    fs<<"MH_03_meidum";
    fs<<"{"<<"imu_begin"<<479;
    fs<<"grt_begin"<<1;
    fs<<"first_frame"<<48;
    fs<<"total_frames"<<2631;
    fs<<"}";
    
    fs<<"MH_04_difficult";
    fs<<"{"<<"imu_begin"<<336;
    fs<<"grt_begin"<<2;
    fs<<"first_frame"<<35;
    fs<<"total_frames"<<1976;
    fs<<"}";
    
    fs<<"MH_05_difficult";
    fs<<"{"<<"imu_begin"<<287;
    fs<<"grt_begin"<<8;
    fs<<"first_frame"<<30;
    fs<<"total_frames"<<2221;
    fs<<"}";
    
    fs<<"V1_02_medium";
    fs<<"{"<<"imu_begin"<<201;
    fs<<"grt_begin"<<2;
    fs<<"first_frame"<<21;
    fs<<"total_frames"<<1671;
    fs<<"}";
    
    fs<<"V1_03_difficult";
    fs<<"{"<<"imu_begin"<<369;
    fs<<"grt_begin"<<2;
    fs<<"first_frame"<<37;
    fs<<"total_frames"<<2094;
    fs<<"}";
    
    fs<<"V2_02_medium";
    fs<<"{"<<"imu_begin"<<257;
    fs<<"grt_begin"<<7;
    fs<<"first_frame"<<26;
    fs<<"total_frames"<<2309;
    fs<<"}";
    
    fs<<"V2_03_difficult";
    fs<<"{"<<"imu_begin"<<267;
    fs<<"grt_begin"<<24;
    fs<<"first_frame"<<28;
    fs<<"total_frames"<<1895;
    fs<<"}";
    
    
    fs.release();
}


#endif /* write_config_parameters_h */
