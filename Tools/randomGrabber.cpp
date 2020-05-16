#include "randomGrabber.h"

void randomGrabber::getTriangles(uint numberOfTriangles,  std::vector<Eigen::Vector3d> points){
    vectorTriangles.erase(vectorTriangles.begin(), vectorTriangles.end());
    std::cout <<"Computing triangles \n";
    //for number of triangles times
    for(uint i=0; i<numberOfTriangles; i++){
        //get current generate random poinst and make sure they don't overlap
        currRandNum = getRandomElement(points.size());
        //check if is outside
        while(currRandNum >= points.size())
            currRandNum = getRandomElement(points.size());
        currCandidate1 = getRandomElement(points.size());
        //check if is outside
        while(currCandidate1 >= points.size())
            currCandidate1 = getRandomElement(points.size());
        while(currRandNum == currCandidate1)
            currCandidate1 = getRandomElement(points.size());
        currCandidate2 = getRandomElement(points.size());
        //check if is outside
        while(currCandidate2 >= points.size())
            currCandidate2 = getRandomElement(points.size());
        while(currRandNum == currCandidate2 || currCandidate2 == currCandidate1)
            currCandidate2 = getRandomElement(points.size());
        //std::cout << "[CEK DEBUG] id: " << currRandNum << "|" << currCandidate1 << "|" << currCandidate2 << std::endl;
        //verify that first point is enough far from the first
        if( std::abs(computeDistance(points.at(currRandNum), points.at(currCandidate1))- avgDistance) < avgDistance/10 ||
            std::abs(computeDistance(points.at(currRandNum), points.at(currCandidate2))- avgDistance) < avgDistance/10 ||
            std::abs(computeDistance(points.at(currCandidate2), points.at(currCandidate1))- avgDistance) < avgDistance/10){
            continue;
        }
        currTri.p1 = points.at(currRandNum);
        currTri.p2 = points.at(currCandidate1);
        currTri.p3 = points.at(currCandidate2);
        
        vectorTriangles.push_back(currTri);
    }
}