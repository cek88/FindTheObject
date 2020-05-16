#include "icpEvaluator.h"

void icpEvaluator::errorAndJacobianLinearRelaxation(Eigen::Vector3d p, Eigen::Vector3d z){
    //for each row of M
    for(uint i=0; i<3; i++){
        //for each i+j element 
        for(uint j=i*3; j<i*3+3; j++){
            //fill the m matrix with p
            m(i, j)=p(j-i*3);
        }
    }
    J = Eigen::MatrixXd::Zero(3,12);
    //compute h value
    h = m * R_guess + t_guess;
    e = h - z;
    J.block(0,0,3,9) = m;
    J.block(0,8, 3,3) = Eigen::Matrix3d::Identity();
}  

void icpEvaluator::errorAndJacobianStandard(Eigen::VectorXd x, Eigen::Vector3d p, Eigen::Vector3d z){
    evalRotMatrices(x(3), x(4), x(5));
    J_std = Eigen::MatrixXd::Zero(3,6);
    h = Rx*Ry*Rz * p + x.head(3);
    e = h - z;
    J_std.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    J_std.block(0,3,3,1) = dotRx * Ry * Rz * p;
    J_std.block(0,4,3,1) = Rx * dotRy * Rz * p;
    J_std.block(0,5,3,1) = Rx * Ry * dotRz * p;
    //std::cout << "curr Jacobian\n"<< J_std << "\n curr error:\n"<< e <<std::endl;
}  

void icpEvaluator::doICPstd(std::vector<Eigen::Vector3d> points, 
            std::vector<Eigen::Vector3d> measurements){
    if(x_guess.size() != 6){
        std::cout << "Error, x_guess size is: " << x_guess.size() << std::endl;
    }
    // curr_x_guess = x_guess;
    //std::cout << "pre comp\n" << x_guess << std::endl;
    //v2t(x_guess);
    H = Eigen::MatrixXd::Zero(6,6);
    b = Eigen::VectorXd::Zero(6);
    chi = 0;
    //for each point 
    for(uint i=0; i<points.size();i++){
        //first step, compute linear relaxation
        errorAndJacobianStandard(x_guess, points.at(i), measurements.at(i));
        H += J_std.transpose()*J_std;
        b += J_std.transpose()*e;  
        chi += e.transpose() * e;
    }
    //std::cout << "H dim: " << H.rows() << "|" << H.cols()<< "\n";
    chi/=points.size();
    H = H + D_FACTOR * ones;
    std::cout << "current chi: " << chi << std::endl;
    chi_stats.push_back(chi);
    dX = -H.colPivHouseholderQr().solve(b);
    //std::cout << "dX:\n" << dX << std::endl;
    // for(uint i=0;i<x_guess.size();i++){
    //     x_guess(i) += dX(x_guess.size()-i); 
    // }
    x_guess += dX;
    R.block(0,0,3,3) = Rx*Ry*Rz;
    evalRotMatrices(x_guess(3), x_guess(4), x_guess(5));
    for(uint i=0; i<measurements.size(); i++){
        measurements.at(i) = R.block(0,0,3,3) * measurements.at(i) + x_guess.head(3);   
    }
    //chi_stats.push_back(chi);
    
}

void icpEvaluator::doICPLinearRelax(Eigen::VectorXd x_guess, std::vector<Eigen::Vector3d> points, 
            std::vector<Eigen::Vector3d> measurements){
    if(x_guess.size() != 6){
        std::cout << "Error, x_guess size is: " << x_guess.size() << std::endl;
    }
    // curr_x_guess = x_guess;
    //std::cout << "pre comp\n" << x_guess << std::endl;
    v2t(x_guess);
    H = Eigen::MatrixXd::Zero(12,12);
    b = Eigen::VectorXd::Zero(12);
    avgError = Eigen::Vector3d::Zero();
    chi = 0;
    //for each point 
    for(uint i=0; i<points.size();i++){
        //first step, compute linear relaxation
        errorAndJacobianLinearRelaxation(points.at(i), measurements.at(i));
        H += J.transpose()*J;
        b += J.transpose()*e;  
        chi += e.transpose()*e;
    }
    //std::cout << "current chi: " << chi << std::endl;
    //std::cout << "Average error:\n" << avgError << std::endl;
    dX = -H.colPivHouseholderQr().solve(b);
    //std::cout << "dX\n" << dX << std::endl;
    for(uint i=0; i<3;i++){
        for(uint j=0; j<3;j++){
            R(i,j) = dX((i+3)+j) + R_guess(i+j); 
            R_guess(i+j) += dX((i+3)+j);
        }
    }
    //evaluate t
    curr_pos_in_t = 0;
    for(uint i=dX.size()-3; i<dX.size();i++){
        //std::cout << "r: " << curr_pos_in_t << "/"<<t.size() <<"| dX: " << i << "/" << dX.size()<< std::endl; 
        t(curr_pos_in_t) = dX(i) + t_guess(curr_pos_in_t);
        //update t_guess
        t_guess(curr_pos_in_t) += dX(i);
        curr_pos_in_t++;
    }
    //compute singular value decomposition
    svd.compute(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
    U = svd.matrixU();
    V = svd.matrixV();
    s = svd.singularValues();
    R = U*V.transpose();
    //std::cout << s << std::endl;
    if(std::abs(s.norm()-1 >0.8f))
        chi = 100; 
    // R.block(0,0,3,3) = Rx*Ry*Rz;
    // evalRotMatrices(x_guess(3), x_guess(4), x_guess(5));
    for(uint i=0; i<measurements.size(); i++){
        measurements.at(i) = R.block(0,0,3,3) * measurements.at(i) + t_guess;   
    }
}


Eigen::Vector3d icpEvaluator::getAlignment(uint nRounds, std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector3d> measurements){
    for(uint i=0; i<nRounds;i++){
        std::cout << "ICP round n: " << i+1 << std::endl;
        if(i==0){
            //compute ICP linear relaxation only for the first round
            doICPLinearRelax(x_guess, points, measurements);
            std::cout << "t\n" << t_guess << "\nR\n" << R << std::endl;
            x_guess = Eigen::VectorXd(6);
            //update x_guess for std ICP
            x_guess.head(3) = t_guess;
            //from rot matrix to angles
            tempR = R.block(0,0,3,3);
            x_guess.tail(3) = tempR.eulerAngles(0,1,2);
            // x_guess(3) = std::atan2(R(2,1), R(2,2));
            // x_guess(4) = std::atan2(-R(2,0), std::sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
            // x_guess(5) = std::atan2(R(1,0),R(0,0));
            std::cout << "chi value: " << chi << std::endl;
        }else{
            //std::cout << x_guess << std::endl; 
            dX = Eigen::VectorXd(6);
            doICPstd(points, measurements);
        }
        //std::cout << "[CEK DEBUG] current error: \n" << e << std::endl; 
    }
    // std::cout << "applying Homogeneous transf to each measured point\n";
    // //r = R.block(0,0,3,3);
    // //fill a vector with points subject to the homogeneous transformation
    // for(auto const p : measurements){
    //     changedPoints.push_back((R.block(0,0,3,3)*p) + t);
    // }
    return e;
}