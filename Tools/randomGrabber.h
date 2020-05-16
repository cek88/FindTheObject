#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <random>

#define A_DIST 0.0535945f

struct triangle
{
    Eigen::Vector3d p1, p2, p3;
};

/*
*   class wich implements a random triplette taker 
*/
class randomGrabber
{
private:
    float avgDistance = A_DIST;
    //get the average distance
    //int divid=0;
    // float getAvgDistance( std::vector<Eigen::Vector3d> points){
    //     avgDistance = 0;
    //     divid = 0;
    //     for(uint i=0; i<500;i++){
    //         for(uint j=i+1; j<500;j++){
    //             avgDistance += computeDistance(points.at(i), points.at(j));
    //             std::cout << divid << std::endl;
    //             divid++;
    //         }
    //     }
    //     avgDistance /= divid;
    //     std::cout << "[CEK DEBUG] avg: " << avgDistance << "--------------------------\n";
    // };

    //simple euclidean distance between points
    float computeDistance(Eigen::Vector3d p1, Eigen::Vector3d p2){
        return (p1 - p2).norm();
    };

    uint getRandomElement(int size){
        return rand()%(size + 1)-1;
    }

    //method wich gets a set of equidistant triangles
    std::vector<triangle> vectorTriangles;
    void getTriangles(uint numberOfTriangles,  std::vector<Eigen::Vector3d> points);
    uint currRandNum;
    uint currCandidate1, currCandidate2;
    float currDistance;
    triangle currTri;
public:
    randomGrabber(){
        //random seed
        srand(time(NULL));
    };
    //method wich returns current triangles
    std::vector<triangle> returnTriangles(uint numberOfTriangles,  std::vector<Eigen::Vector3d> points){

        std::cout <<"starting recovery of triangles \n";

        getTriangles(numberOfTriangles,  points);
        return vectorTriangles;
    };

    //~randomGrabber();
};
