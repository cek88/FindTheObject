#include "Parser.h"


std::vector<Eigen::Vector3d> dataParser::parseFile(const char filePath[]){
    //erase all points stored 
    parsedPoints.erase(parsedPoints.begin(), parsedPoints.end());
    //open file
    file.open(filePath);
    //check if the file exists
    if(file.is_open()){
        while(getline(file, line)){
            //std::cout << "[CEK DEBUG PARSER.CPP] line: " << line << std::endl;
            posInVector = 0;
            token = strtok(const_cast<char*>(line.c_str()), delim);
            while(token != nullptr){
                //std::cout << "[CEK DEBUG PARSER.CPP] token: " << token << std::endl;
                if(posInVector == 0 )
                    auxiliaryVector.x() = std::stod(token);
                else if(posInVector == 1 )
                    auxiliaryVector.y() = std::stod(token);
                else if(posInVector == 2 )
                    auxiliaryVector.z() = std::stod(token);
                else
                    std::cout << "[PARSER.CPP / dataParser - Parsing line] THIS SHOULD NOT HAPPEN" << std::endl;
                ++posInVector;
                token = strtok(nullptr, delim);   
            }
            parsedPoints.push_back(auxiliaryVector);
        }
    }
    else{
        std::cout << "[PARSER:CPP] cannot open the file: " << filePath << std::endl;
    }
    file.close();
    return parsedPoints;
}
