//
//  PangoViewer.hpp
//  aslam
//
//  Created by 谭智丹 on 17/1/5.
//
//

#ifndef PangoViewer_hpp
#define PangoViewer_hpp

#include <stdio.h>
#include <pangolin/pangolin.h>
#include <Eigen/Geometry>


class PangoViewer {
    
public:
    
    void DrawMapPoints(std::vector<Eigen::Vector3d>& mPoints);
    
    void DrawActivePoints(std::vector<Eigen::Vector3d>& aPoints, Eigen::Vector3d& cam);
    
    void GetOpenGlMatrix(Eigen::VectorXd &rq, pangolin::OpenGlMatrix &M);
    
};


#endif /* PangoViewer_hpp */
