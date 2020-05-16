#include "distanceMap.h"

void distanceMap::computeBestFriend(){
    for(auto const p : *points){
        currNode.distance = MAX_DISTANCE;
        for(uint i=0; i < points->size(); i++){
            currDistance = computeDistance(p, points->at(i));
            if(currNode.distance > currDistance && currDistance !=0){
                currNode.distance = currDistance;
                currNode.id = i;
            }
        }
        bestFriends.push_back(currNode);
    }
}

uint distanceMap::findPointAtDistance(uint pointId, float distance){
    
}