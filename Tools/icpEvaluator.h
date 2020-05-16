#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#define _USE_MATH_DEFINES
#define c(theta) std::cos(theta)
#define s(theta) std::sin(theta)
#define D_FACTOR 2.5f//3.f
class icpEvaluator
{
private:
    Eigen::Matrix3d Rx, Ry, Rz, dotRx, dotRy, dotRz;
    //evaluation of rotation matrices 
    void evalRotMatrices(float alpha_x, float alpha_y, float alpha_z){
        Rx <<   1, 0, 0,
                0, c(alpha_x), s(alpha_x),
                0, -s(alpha_x), c(alpha_x);
        Ry << c(alpha_y), 0, s(alpha_y),
              0, 1, 0,
              -s(alpha_y), 0, c(alpha_y);
        Rz << c(alpha_z), -s(alpha_z), 0,
              s(alpha_z), c(alpha_z), 0,
              0, 0, 1;
        dotRx << 0, 0, 0,
                0, -s(alpha_x), -c(alpha_x),
                0, c(alpha_x), -s(alpha_x);
        dotRy << -s(alpha_y), 0, c(alpha_y),
                 0, 0, 0,
                 -c(alpha_y), 0, -s(alpha_y);
        dotRz << -s(alpha_z), -c(alpha_z), 0,
                 c(alpha_z), -s(alpha_z), 0,
                 0, 0, 0;
    };

    //as the ICP Octave code
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(3,9);
    Eigen::Vector3d e = Eigen::Vector3d::Zero();
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3,12);
    Eigen::Vector3d h = Eigen::Vector3d::Zero();
    void errorAndJacobianLinearRelaxation(Eigen::Vector3d p, Eigen::Vector3d z); 
    Eigen::MatrixXd ones = Eigen::MatrixXd::Identity(6,6); 
    Eigen::MatrixXd ones_lr = Eigen::MatrixXd::Identity(12,12);
    Eigen::MatrixXd J_std = Eigen::MatrixXd::Zero(3,6);
    std::vector<float> chi_stats;
    float chi;
    // Eigen::MatrixXd H_std = Eigen::MatrixXd::Zero(6,6);
    // Eigen::VectorXd b_std = Eigen::VectorXd::Zero(6);
    void errorAndJacobianStandard(Eigen::VectorXd x, Eigen::Vector3d p, Eigen::Vector3d z);
    void doICPstd(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector3d> measurements);

    Eigen::Matrix3d tempR = Eigen::Matrix3d::Zero();
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4,4);
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12,12);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd dX = Eigen::VectorXd::Zero(12);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    Eigen::MatrixXd U, s, V;
    
    
    //complete x guess Hom
    Eigen::MatrixXd X_guess = Eigen::MatrixXd::Identity(4,4); 
    //translation of x_guess
    Eigen::Vector3d t_guess = Eigen::Vector3d::Zero();
    //rotation of x_guess
    Eigen::VectorXd R_guess = Eigen::VectorXd::Zero(9);
    uint currPos = 0;
    void v2t(Eigen::VectorXd x){
        evalRotMatrices(x(3), x(4), x(5));
        X_guess.block(0,0,3,3) = Rx*Ry*Rz;
        //std::cout << "x_guess\n"<<X_guess << std::endl;
        currPos = 0;
        for(uint i=0; i<3; i++){
            for(uint j=0; j<3; j++){  
                R_guess(currPos) = X_guess(i,j);
                currPos++;
            }
        }
        for(uint i=0; i<t_guess.size();i++){
            t_guess(i) = x(i);
        } 
        //std::cout << "rot vector:\n" << R_guess << "\n r:guess:\n" << t_guess << std::endl;
        //= X_guess.block(3,3,4,1);
    };
    
    Eigen::VectorXd x_guess = Eigen::VectorXd(6);
    Eigen::VectorXd curr_x_guess;
    uint curr_pos_in_t = 0;
    Eigen::Vector3d avgError = Eigen::Vector3d::Zero();
    // std::vector<Eigen::Vector3d> changedPoints; 
    // Eigen::Matrix3d r = Eigen::Matrix3d::Zero(); 
public:
    icpEvaluator(){};
    Eigen::Vector3d getAlignment(uint nRounds, std::vector<Eigen::Vector3d> points, 
                    std::vector<Eigen::Vector3d> measurements);
    Eigen::Matrix3d getR(){
        evalRotMatrices(x_guess(3),x_guess(3),x_guess(5));
        return Rx*Ry*Rz;
    };
    Eigen::Vector3d getT(){
        return x_guess.head(3);
    };
    //icp Linear relaxation
    void doICPLinearRelax(Eigen::VectorXd x_guess, std::vector<Eigen::Vector3d> points, 
            std::vector<Eigen::Vector3d> measurements);
    float getCurrentChi(){
        return chi;
    }
    std::vector<float> getChi(){
        return chi_stats;
    };
    ~icpEvaluator(){};
};
