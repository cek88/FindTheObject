/*
Simple implemntation of a parser for txt files
*/
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <string>

class dataParser{
    public:
        //constructor of Parser
        dataParser(){
            auxiliaryVector = Eigen::Vector3d::Zero();
            parsedPoints.push_back(auxiliaryVector);
            posInVector = 0;
        }
        /*
        *   method which parses the text files
        *   @input string: filepath
        *   @output vector of 3d points [double]
        */
        std::vector<Eigen::Vector3d> parseFile(const char filePath[]);
        

    private:
        //output vector containing the point parsed
        std::vector<Eigen::Vector3d> parsedPoints;
        //auxiliary Vector3f
        Eigen::Vector3d auxiliaryVector;
        //file opener
        std::ifstream file;
        std::string line;
        char charifiedString[];
        char * token;
        uint posInVector;
        char delim[3] = " ";
};