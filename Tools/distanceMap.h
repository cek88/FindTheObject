#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>

#define MAX_DISTANCE 500.f
#define K = 4

struct node
{
    float distance;
    uint id;
};


class distanceMap
{
private:
    //vector storing closest element to each
    std::vector<node> bestFriends;
    std::vector<Eigen::Vector2f>* points;
    //compute nearest neighbor
    void computeBestFriend();
    //simple euclidean distance between points
    float computeDistance(Eigen::Vector2f p1, Eigen::Vector2f p2){
        return (p1 - p2).norm();
    };
    //stack for points
    node currNode;
    float currDistance;

public:
    distanceMap(std::vector<Eigen::Vector2f>* input){
        points = input;
        computeBestFriend();
    };

    //find point at distance
    uint findPointAtDistance(uint pointId, float distance);

    ~distanceMap();
};


