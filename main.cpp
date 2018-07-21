//
//  main.cpp
//  ekf_slam
//
//  Created by 谭智丹 on 16/9/20.
//  Copyright © 2016年 谭智丹. All rights reserved.


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>

#include "system.hpp"
#include "write_config_parameters.h"
#include "PangoViewer.hpp"

#define intervalN 10

using namespace cv;
using namespace std;


void LineAlignment(ifstream& ifs, int N)
{
    string aline;
    for (int i=0; i<N; i++) {
        getline(ifs, aline);
    }
}


void ReadOneGrtLine(ifstream& ifs, v3d& p, v4d& q, v3d& v, v3d& bw, v3d& ba) {
    string aline, s;
    
    getline(ifs, aline);
    stringstream stream(aline);
    
    getline(stream, s, ','); // timestamp
    
    for (int i=0; i<3; i++) {  // position
        getline(stream, s, ',');
        p[i] = stof(s);
    }
    for (int i=0; i<4; i++) {  // quaternion
        getline(stream, s, ',');
        q[i] = stof(s);
    }
    for (int i=0; i<3; i++) {  // velocity
        getline(stream, s, ',');
        v[i] = stof(s);
    }
    for (int i=0; i<3; i++) {  // gyroscope bias
        getline(stream, s, ',');
        bw[i] = stof(s);
    }
    for (int i=0; i<3; i++) {  // accelerometer bias
        getline(stream, s, ',');
        ba[i] = stof(s);
    }
    
}

bool ReadIMUInterval(ifstream& ifs, vector<v3d>& wms, vector<v3d>& ams) {
    string aline, s;
    v3d v;
    
    for (int i=0; i<intervalN; i++) {
        if( !getline(ifs, aline) ) {
            cout << "No IMU data any more!" << endl;
            return false;
        }
        
        stringstream stream(aline);
        
        getline(stream, s, ',');    // timestamp
        for (int j=0; j<3; j++) {   // gyroscope measurements
            getline(stream, s, ',');
            v[j] = stof(s);
        }
        wms.push_back(v);
        
        for (int j=0; j<3; j++) {   // accelerometer measurements
            getline(stream, s, ',');
            v[j] = stof(s);
        }
        ams.push_back(v);
    }
    
    return true;
}


void GetGrtPoses(ifstream &ifs, vector<v3d> &grt_poses, v3d &r0, int N) {
    
    string line, s;
    
    for (int i=1; i<10*N; i+=1) {
        getline(ifs, line);
        
        if (i % 10 != 0) continue;
        
        stringstream stream(line);
        
        getline(stream, s, ','); // timestamp
        
        v3d v;
        for (int j=0; j<3; j++) {
            getline(stream, s, ',');
            v(j) = stof(s);
        }
        
        v -= r0;
        v(2) = 0;
        grt_poses.push_back(v);
    }
    
}


//-----------------------------------------------------------------------------------------------------------


int main(int argc, const char * argv[]) {
    
    // Choose a dataset & set the first frame
    string dataset = "MH_05_difficult";
    string head_name = "/Users/tanzhidan/Documents/data/euroc/" + dataset + "/cam0/";
    int iBegin = 1;
    int MaxStep = 1200;
    bool half_resolution = false;
    
    // Open a file to store the results
    ofstream fout("/Users/tanzhidan/Documents/data/euroc/" + dataset + "/poses.csv");
    if (!fout)
    {
        cout << "File not opened!" << endl;
        return -1;
    }
    
    
    // --Create the EKF-SLAM system
    system::system slam(half_resolution);
    slam.LoadParameters(dataset);
    
    
    
    //---- IMU Data & Ground truth data -------------
    
    ifstream imu_file( "/Users/tanzhidan/Documents/data/euroc/" + dataset + "/imu0/data.csv");
    ifstream grt_file( "/Users/tanzhidan/Documents/data/euroc/" + dataset + "/gtruth/data.csv");
    string aline;
    
    
    // -set to the right line
    int iFrame = slam.first_frame + iBegin - 1;
    int iImu   = 10 * (iBegin-1) + slam.imu_begin;
    int iGrt   = 10 * (iBegin-1) + slam.grt_begin;
    LineAlignment(imu_file, iImu);
    LineAlignment(grt_file, iGrt);
    
    int init_frame = iFrame;
    
    //------------------------------------------------@
    
    
    ////////////////////////////////////////////////////////////////////////////
    
    //---- Initialize the system ---------------------
    
    // -get the initial velocity & orientation
    v3d init_p, init_v, bias_a, bias_w;
    v4d vq;
    ReadOneGrtLine(grt_file, init_p, vq, init_v, bias_w, bias_a);
    
    slam.Initialize(vq, init_v, bias_w, bias_a);
    
    
    // -save the initial state
    Eigen::VectorXd Xrqv(10);
    slam.GetXrqv(Xrqv);
    for (int i=0; i<Xrqv.size(); i++) {
        fout << Xrqv(i) << ",";
    }
    fout << "\n";
    
    
    // -read the first image
    char tail_name[20];
    sprintf(tail_name, "%06d.png", iFrame);
    Mat I = imread(head_name+tail_name, CV_LOAD_IMAGE_UNCHANGED);
    if (half_resolution) {
        resize(I, I, Size(I.cols/2, I.rows/2));
    }
    //---------------------------------------------------@
    
    
    
    
    //----------Pangolin------------------------------------------
    pangolin::CreateWindowAndBind("Main", 1024,768);
    glEnable(GL_DEPTH_TEST);
    
    pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(1024,768,500,500,512,384,0.1,1000);
    pangolin::OpenGlRenderState s_cam(proj, pangolin::ModelViewLookAt(0.0, 0.0, 30.0, 0,0,0, 1.0, 0.0, 0.0));
    
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f)
    .SetHandler(&handler);
    
    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    
    PangoViewer pango;
    
    // -------poses
    v3d curr_pos(0,0,0);
    vector<v3d> poses;
    poses.push_back(curr_pos);
    
    // ---Get ground truth poses
    vector<v3d> grt_poses;
    grt_poses.push_back(curr_pos);
    GetGrtPoses(grt_file, grt_poses, init_p, MaxStep);
    
    //----------------------------------------------------------------------------
    
    
    // add first keyframe
    slam.AddFirstKeyFrame(0);
    
    //---------------------- MAIN LOOP -----------------------
    vector<v3d> currentPoints;

    clock_t Ts, Te, Tu=0;
    
    for (int iStep=1; iStep<MaxStep+1; iStep++) {
        
        cout << iStep << endl;
        
        Ts = clock();

        // -- Map management ---------------------
        // Delete features, update features_info.
        // Add new features.
        slam.ManageMap(I, iStep);
        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~To Do:
         Keep tracking of some backup points, so that when adding new features,
         we can choose the points with well estimated depth.
         This can have significant influence when the depth range is large,
         like in the outdoor enviroment.
         */
        
    
        // -- Read IMU data ---
        // Read all imu readings between last frame and current frame
        vector<v3d> w_interval, a_interval;
        a_interval.reserve(intervalN);
        w_interval.reserve(intervalN);
        ReadIMUInterval(imu_file, w_interval, a_interval);
        
        
        // -- Propagation -----------------------
        // We use pre-integrate method.
        slam.Propagate(w_interval, a_interval, iStep);
        /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~To Do:
         Use Runge-Kutta method to integrate the imu readings. It may perform
         better when the angular velocity/acceleration is large.
         */
        
        Te = clock();
        Tu = Te - Ts;
        
        
        // -- Grab a new frame --------------------
        iFrame++;
        sprintf(tail_name, "%06d.png", iFrame);
        I = imread(head_name+tail_name, CV_LOAD_IMAGE_UNCHANGED);
        if (half_resolution) {
            resize(I, I, Size(I.cols/2, I.rows/2));
        }
        
        Ts = clock();
        // -- Search compatible matches ----------------------
        // Firstly we calculate h, H, S for each feature,
        // then search around the predicted location
        slam.SearchMatches(I);
        
        
        // -- Update -------------------------
        // 1-point Ransac, get low innovation inliers, update.
        // Then rescue high innovation inliers, update.
        // Iterate if neccessary (seems not neccessary)
        slam.BigUpdate(iStep);
        
        
        // -- Supervise -----------------------
        // We need to evaluate the slam's performance,
        // in case it diverges. Then we may re-initialize the system.
        ////// slam.SuperVise();
        
        Te = clock();
        Tu += Te - Ts;
        
        // -- Save & Display -------------------
        slam.GetXrqv(Xrqv);
        for (int i=0; i<Xrqv.size(); i++) {
            fout << Xrqv(i) << ",";
        }
        fout << "\n";
        
        curr_pos = Xrqv.head(3);
        poses.push_back(curr_pos);
        
        
        // ------------Pangolin-------------
        // clear screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // get Twc
        pango.GetOpenGlMatrix(Xrqv, Twc);
        
        // activate
        //s_cam.Follow(Twc); //-------
        d_cam.Activate(s_cam);
        
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        
        
        // Draw trajectory---
        
        glPointSize(3);
        glColor3f(0, 1.0, 1.0);
        glBegin(GL_POINTS);
        
        for (int i=0; i<poses.size(); i++) {
            v3d pp = poses[i];
             pp(2) = 0;
             pangolin::glVertex(pp);
        }
        glEnd();
        
        glLineWidth(4);
        glColor3f(1, 1, 0);
        glBegin(GL_LINES);
        for (int i=0; i<iStep-1; i++) {
            pangolin::glVertex(grt_poses[i]);
            pangolin::glVertex(grt_poses[i+1]);
        }
        glEnd();
        
        // Draw map points---
        pango.DrawMapPoints(slam.map_points);
        
        // Draw current points----
        currentPoints.clear();
        slam.GetActivePoints(currentPoints);
        pango.DrawActivePoints(currentPoints, poses.back());
        
        
        // Finish
        pangolin::FinishFrame();
        
        slam.DrawOnImage(I);
        
        if (iStep < 2) {
            //slam.DrawOnImage(I);
            waitKey();
        }
        
        cout << "time:\t" << (double)Tu/CLOCKS_PER_SEC << endl << endl;
    }
    
    fout.close();
    
    /*
    //--------------------------Loop Closures-------------------------/
    ifstream loop_file( "/Users/tanzhidan/Documents/data/euroc/" + dataset + "/loop.csv");
    vector<int> loops[2];
    vector<float> duvs[2];
    while (1) {
        string aline, s;
        if( !getline(loop_file, aline) ) {
            cout << "No IMU data any more!" << endl;
            break;
        }
        
        stringstream stream(aline);
        
        getline(stream, s, ',');
        loops[0].push_back(stoi(s));
        
        getline(stream, s, ',');
        loops[1].push_back(stoi(s));
        
        getline(stream, s, ',');
        duvs[0].push_back(stof(s));
        
        getline(stream, s, ',');
        duvs[1].push_back(stof(s));
    }
    
    
    int lastiquey = 0;
    int nLoopTogether = 0;
    for (size_t i=0; i<loops[0].size(); i++) {
        int iquery = loops[0][i];
        int imatch = loops[1][i];
        float du = duvs[0][i];
        float dv = duvs[1][i];
        
        sprintf(tail_name, "%06d.png", iquery);
        Mat I_query = imread(head_name + tail_name, 0);
        
        sprintf(tail_name, "%06d.png", imatch);
        Mat I_match = imread(head_name + tail_name, 0);
        
        if (half_resolution) {
            resize(I_query, I_query, Size(I_query.cols/2, I_query.rows/2));
            resize(I_match, I_match, Size(I_match.cols/2, I_match.rows/2));
            du /= 2.0;
            dv /= 2.0;
        }
        
        iquery -= init_frame;
        imatch -= init_frame;
        if (imatch < 0 || iquery < 0 || iquery >= MaxStep || imatch >= MaxStep)
            continue;
        
        if (iquery - lastiquey > 40) {
            
            if (nLoopTogether > 3) {
                cout << "IQUERY: " << iquery << endl;
                
                //slam.Fuse(5, lastiquey);
                slam.FinalOptimization(true, true, lastiquey); // Use imu factor & use robust loss function
                slam.FinalOptimization(true, false, lastiquey);
                vector<v3d> nousePoints;
                slam.PrintPoses(init_p, poses, nousePoints, false);
                
                // ------------Pangolin-------------
                // clear screen
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                
                // get Twc
                pango.GetOpenGlMatrix(Xrqv, Twc);
                
                // activate
                // s_cam.Follow(Twc); //-------
                d_cam.Activate(s_cam);
                
                glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
                
                
                // Draw trajectory---
                
                glPointSize(3);
                glColor3f(0, 1, 1);
                glBegin(GL_POINTS);
                for (int i=0; i<poses.size(); i++) {
                    v3d pp = poses[i];
                    pp(2) = 0;
                    pangolin::glVertex(pp);
                }
                glEnd();
                
                glLineWidth(4);
                glColor3f(1, 1, 0.4);
                glBegin(GL_LINES);
                for (int i=0; i<MaxStep-1; i++) {
                    pangolin::glVertex(grt_poses[i]);
                    pangolin::glVertex(grt_poses[i+1]);
                }
                glEnd();
                
                
                // Draw map points---
                pango.DrawMapPoints(nousePoints);
                
                // Finish
                pangolin::FinishFrame();
                
                waitKey();
                
            }
            
            nLoopTogether = 0;
        } else {
            nLoopTogether++;
        }
        
        slam.IHaveALoop(I_query, I_match, iquery, imatch, du, dv);
        lastiquey = iquery;
    }
    
    
    slam.FinalOptimization(true, true, MaxStep); // Use imu factor & use robust loss function
    slam.FinalOptimization(true, false, MaxStep);

    vector<v3d> points;
    points.clear();
    slam.PrintPoses(init_p, poses, points, false);
    
    // ------------Pangolin-------------
    // clear screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // get Twc
    pango.GetOpenGlMatrix(Xrqv, Twc);
    
    // activate
    // s_cam.Follow(Twc); //-------
    d_cam.Activate(s_cam);
    
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    
    
    // Draw trajectory---
    
     glPointSize(3);
     glColor3f(0, 1, 1);
     glBegin(GL_POINTS);
     for (int i=0; i<poses.size(); i++) {
         v3d pp = poses[i];
         pp(2) = 0;
         pangolin::glVertex(pp);
     }
     glEnd();
    
    glLineWidth(4);
    glColor3f(1, 1, 0);
    glBegin(GL_LINES);
    for (int i=0; i<MaxStep-1; i++) {
        pangolin::glVertex(grt_poses[i]);
        pangolin::glVertex(grt_poses[i+1]);
    }
    glEnd();
    
    
    // Draw map points---
    pango.DrawMapPoints(points);
    
    // Finish
    pangolin::FinishFrame();
    */
    
    waitKey();
    
    return 0;
}
