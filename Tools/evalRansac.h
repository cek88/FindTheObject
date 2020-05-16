#pragma once
#include "randomGrabber.h"
#include "icpEvaluator.h"

#define D_THRESHOLD 0.01f
#define REQUIRED_PROBABILITY 0.9f
//get the couple
struct coupleTriangles{
    triangle t1;
    triangle t2;
};

class evalRansac
{
private:
    //TODO check if necessary
    randomGrabber rg;
    float inlierRatio;
    void getInlierRatio(std::vector<triangle> globe, std::vector<triangle> scene);
    
    bool getSimilarity(triangle t1, triangle t2){
        if(std::abs((t1.p1-t1.p2).norm() - (t2.p1-t2.p2).norm()) > D_THRESHOLD)
            return false;
        if(std::abs((t1.p1-t1.p3).norm() - (t2.p1-t2.p3).norm()) > D_THRESHOLD)
            return false;
        if(std::abs((t1.p3-t1.p2).norm() - (t2.p3-t2.p2).norm()) > D_THRESHOLD)
            return false;
        return true;
    };
    
    std::vector<coupleTriangles> matchingTriangles;
    coupleTriangles ct;
    uint numIterations;
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> measures;
    Eigen::Vector3d dumb;
    std::vector<Eigen::Vector3d> tri1;
    std::vector<Eigen::Vector3d> tri2;
    float evalNumIterations(uint numInliers){
        if(std::pow(inlierRatio, numInliers) < 0.00000001f){
            std::cout << std::pow(inlierRatio, numInliers) << " ci da un numero infinito d'iterazioni\n";
        }
        return (std::log((1-REQUIRED_PROBABILITY)/std::log(1-(std::pow(inlierRatio, numInliers)))));
    };

    icpEvaluator iEv;
    Eigen::VectorXd first_x_guess = Eigen::VectorXd::Zero(6); 
    //first_x_guess << 0.5f, 0.5f, 0.5f, M_PI/3, M_PI, M_PI/2;

public:
    evalRansac(){
        for(uint i=0; i<3;i++){
            tri1.push_back(dumb);
            tri2.push_back(dumb);
        }
    };
    float retInlierRatio(std::vector<triangle> globe, std::vector<triangle> scene){
        getInlierRatio(globe, scene);
        std::cout << "num iterations required: " << evalNumIterations(3) << std::endl;
        return inlierRatio;
    };
    std::vector<coupleTriangles> returnTriangles(){
        return matchingTriangles;
    };
    void doIcp(){
        if(points.size()==0 || measures.size()==0){
            std::cout << "in order to perform icp, you have to run getInlierRatio()\n";
        }
        else{
            iEv.getAlignment(50, points, measures);
        }
    };
    Eigen::Matrix3d getRotationMatrix(){
        return iEv.getR();
    };
    Eigen::Vector3d getTranslation(){
        return iEv.getT();
    };
    std::vector<float> getErrStats(){
        return iEv.getChi();
    };
    ~evalRansac(){};
};
