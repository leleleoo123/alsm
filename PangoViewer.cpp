//
//  PangoViewer.cpp
//  aslam
//
//  Created by 谭智丹 on 17/1/5.
//
//

#include "PangoViewer.hpp"

void PangoViewer::DrawMapPoints(std::vector<Eigen::Vector3d>& mPoints)
{
    glEnable(GL_POINT_SMOOTH);
    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3f(1,0,1);
    
    for (size_t i=0; i<mPoints.size(); i++) {
        Eigen::Vector3d pt = mPoints[i];
        pt(2) = 0;
        pangolin::glVertex(pt);
    }
    
    glEnd();
    glDisable(GL_POINT_SMOOTH);
}


void PangoViewer::DrawActivePoints(std::vector<Eigen::Vector3d>& aPoints, Eigen::Vector3d& cam)
{
    glEnable(GL_POINT_SMOOTH);
    glPointSize(10);
    glBegin(GL_POINTS);
    glColor3f(0, 1, 0);
    
    for (size_t i=0; i<aPoints.size(); i++) {
        Eigen::Vector3d pt = aPoints[i];
        pt(2) = 0;
        pangolin::glVertex(pt);
    }
    glEnd();
    
    cam(2) = 0;
    glLineWidth(1);
    glColor3f(0.75, 0.996, 0.242);
    glBegin(GL_LINES);
    for (int i=0; i<aPoints.size(); i++) {
        Eigen::Vector3d pt = aPoints[i];
        pt(2) = 0;
        pangolin::glVertex(cam);
        pangolin::glVertex(pt);
    }
    glEnd();
    glDisable(GL_POINT_SMOOTH);
}



void PangoViewer::GetOpenGlMatrix(Eigen::VectorXd &rq, pangolin::OpenGlMatrix &Twc)
{
    Eigen::Vector3d t_Wc = rq.head(3);
    //Eigen::Quaterniond q_Cw(rq(3), rq(4), rq(5), rq(6));
    //Eigen::Matrix3d R_Wc = q_Cw.matrix();
    
    Twc.SetIdentity();
    
    /*
    Twc.m[0] = R_Wc(0,0);
    Twc.m[1] = R_Wc(1,0);
    Twc.m[2] = R_Wc(2,0);
    Twc.m[3] = 0.0;
    
    Twc.m[4] = R_Wc(0,1);
    Twc.m[5] = R_Wc(1,1);
    Twc.m[6] = R_Wc(2,1);
    Twc.m[7] = 0.0;
    
    Twc.m[8] = R_Wc(0,2);
    Twc.m[9] = R_Wc(1,2);
    Twc.m[10] = R_Wc(2,2);
    Twc.m[11] = 0.0;*/
    
    Twc.m[12] = t_Wc(0);
    Twc.m[13] = t_Wc(1);
    Twc.m[14] = 0;
    Twc.m[15] = 1.0;
    
}