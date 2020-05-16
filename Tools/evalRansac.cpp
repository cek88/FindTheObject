#include "evalRansac.h"

void evalRansac::getInlierRatio(std::vector<triangle> globe, std::vector<triangle> scene){
    inlierRatio = 0.f;
    for(auto const g : globe){
        for(auto const s : scene){
            if(getSimilarity(g,s)){
                //mathing points stored to evaluate icp
               
                //triangles
                ct.t1 = g;
                ct.t2 = s;
                tri1.at(0) = g.p1;
                //tri1.at(1) = g.p2;
                //tri1.at(2) = g.p2;
                tri2.at(0) = s.p1;
                //tri2.at(1) = s.p2;
                //tri2.at(2) = s.p2;
                //store inside thi struture the matching triangles
                iEv.doICPLinearRelax(first_x_guess, tri1, tri2);
                if(iEv.getCurrentChi() < 0.005f){
                    points.push_back(g.p1);
                    //points.push_back(g.p2);
                    //points.push_back(g.p3);
                    measures.push_back(s.p1);
                    //measures.push_back(s.p2);
                    //measures.push_back(s.p3);
                    matchingTriangles.push_back(ct);
                    ++inlierRatio;
                    //inlierRatio+=3;
                    break;    
                }
            }
        }
    }

    //std::cout << "[CEK DEBUG] " << inlierRatio << std::endl;

    inlierRatio /= (points.size());
}