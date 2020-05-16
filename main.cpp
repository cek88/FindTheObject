//#pragma once
#include "Parser/Parser.h"
//#include "Tools/randomGrabber.h"
#include "Tools/evalRansac.h"
#include <fstream>

//#include "Plotter/Plotter.h"
//#include "Point_Registrator/Registrator.h"
#define pathToSavePointsGlobe "/home/cek/MARR/RANSAC/Data/matchingPointsGlobe.txt"
#define pathToSavePointsScene "/home/cek/MARR/RANSAC/Data/matchingPointsScene.txt"
#define pathToSaveRotoTranslated "/home/cek/MARR/RANSAC/Data/matchingPointsGlobeInScene.txt"
#define pathToChi "/home/cek/MARR/RANSAC/Data/chi_stats.txt"


int main(int argc, char const *argv[])
{
    if(argv[2] == "-h"){
         std::cerr << "Usage: " ;
         std::cout << "./FindTheObject -objectpath  \"path 1\" -scenepath   \"path 2\" "<< std::endl;
        std::cout << "-objectpath  \"path to the object to find\" "<< std::endl;
        std::cout << "-scenepath   \"path to the scene \""<< std::endl;
        return 1;
    }
    if(argc != 5) { // We expect 5 arguments: the tranche number and the source path th
        std::cout << "ERROR: required 4 arguments!" << std::endl;
        std::cerr << "Usage: ";
        std::cout << "./FindTheObject -objectpath  \"path 1\" -scenepath \"path 2\" "<< std::endl;
        std::cout << "-objectpath  \"path to the object to find\" "<< std::endl;
        std::cout << "-scenepath   \"path to the scene \""<< std::endl;
        return 1;
    }
    std::vector<Eigen::Vector3d> point;
    std::vector<Eigen::Vector3d> scene;
    if(std::string(argv[1]) == "-objectpath"){
        if(std::string(argv[3]) == "-scenepath"){
            dataParser dP;
            point = dP.parseFile(argv[2]);
            scene = dP.parseFile(argv[4]);
        }
        else{
            std::cout << "Wrong input on II argument \ntry:     ./FindTheObject -h " << std::endl;
            return -1;}
    }
    else{
        std::cout << "Wrong input, try:     ./FindTheObject -h " << std::endl;
        return -1;
    }

    std::cout << "we've got: \n-" << point.size() << " points for globe \n-" << scene.size() << " poists for scene \n";
    /*  DEBUG PARSER
    for(auto const p : point){
        std::cout << "point: \n" << p << std::endl;
    }
    */

    /*
    *   DEBUG GRABBER
    */
    randomGrabber rg;
    std::vector<triangle> trisPoint;
    std::vector<triangle> trisScene;
    std::vector<triangle> curr_trisPoint;
    std::vector<triangle> curr_trisScene;
    //TODO check if it has to be embedded into a method
    float iR = 0;
    std::vector<coupleTriangles> matchingTriangles;
    evalRansac er;
    uint iter = 0;
    float best_iR = 0;
    //trying to find best inlier Ratio
    while(iR < 0.2 && iter < 1){
        curr_trisPoint = rg.returnTriangles(uint(point.size()/5), point);
        curr_trisScene = rg.returnTriangles(uint(scene.size()/10), scene);
        // std::cout << "we got " << trisPoint.size() << " triangles for the object, " << trisScene.size() 
        //             << " triangles for the scene\n";
        iR = er.retInlierRatio(curr_trisPoint, curr_trisScene);
        std::cout << "inlier Ratio: "<< iR << std::endl;
        if(iR > best_iR){
            best_iR = iR;
            trisPoint = curr_trisPoint;
            trisScene = curr_trisScene;
        }
        ++iter;
    } 
    if(iter == 10){
        iR = er.retInlierRatio(trisPoint, trisScene);
    }

    std::cout << "finding matching\n" ;
    matchingTriangles = er.returnTriangles();
    std::cout << "matching DONE\n";

    std::vector<Eigen::Vector3d> transfPoints;
    std::cout << "ICP evaluation\n";
    er.doIcp();
    std::cout << "ICP DONE \n";
    Eigen::Matrix3d rotMatrix = er.getRotationMatrix();
    Eigen::Vector3d translVector = er.getTranslation();
    std::vector<float> chiStats = er.getErrStats();
    std::cout << "chi number: " << chiStats.size() << std::endl;
    std::cout << "Applying Homogeneous transformation to measurements\n";
    for(auto const mes : scene){
        //transfPoints.push_back(rotMatrix*mes + translVector);
        transfPoints.push_back(mes);
    }
    /*
    *   print on files
    */
    std::ofstream fileGlobe;
    std::ofstream fileScene;
    std::ofstream fileGlobInScene;
    std::ofstream fileChi;
    fileGlobe.open(pathToSavePointsGlobe);
    fileScene.open(pathToSavePointsScene);
    fileGlobInScene.open(pathToSaveRotoTranslated);
    fileChi.open(pathToChi);
    for(auto const elem : matchingTriangles){
        //write points globe
        fileGlobe << elem.t1.p1.x() << " " << elem.t1.p1.y() << " " << elem.t1.p1.x() << "\n";
        fileGlobe << elem.t1.p2.x() << " " << elem.t1.p2.y() << " " << elem.t1.p2.x() << "\n";
        fileGlobe << elem.t1.p3.x() << " " << elem.t1.p3.y() << " " << elem.t1.p3.x() << "\n";
        //write points scene
        fileScene << elem.t2.p1.x() << " " << elem.t2.p1.y() << " " << elem.t2.p1.x() << "\n";
        fileScene << elem.t2.p2.x() << " " << elem.t2.p2.y() << " " << elem.t2.p2.x() << "\n";
        fileScene << elem.t2.p3.x() << " " << elem.t2.p3.y() << " " << elem.t2.p3.x() << "\n";
    }
    for(auto const elem : transfPoints){
        fileGlobInScene << elem(0) << " " << elem(1) << " " << elem(2) << "\n";
    }
    for(uint i=0; i<chiStats.size();i++){
        fileChi << chiStats.at(i) <<"\n";
    }
    fileChi.close();
    fileGlobe.close();
    fileScene.close();
    fileGlobInScene.close();

    

    return 0;
}
