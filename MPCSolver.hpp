//
//  MPCSolver.hpp
//  reactiveWholeBodyHumanoids
//

#ifndef MPCSOLVER_HPP
#define MPCSOLVER_HPP
//#include <utils/Utils.hpp>
#include <Eigen/Core>
#include <vector>
//#include <utils/ReactiveWholeBodyHumanoidMacros.hpp>
#include <sys/time.h>

namespace mpcSolver{

    class MPCSolver{
	public:

    	// Constructor (see implementation in MPCSolver.cpp)
        MPCSolver(double delta, double predictionTime, Eigen::VectorXd& comPosition,
                  const double singleSupportDuration, const double doubleSupportDuration, const double thetaMaxFootsteps);

        // Solve function used in Controller::update (see implementation in MPCSolver.cpp)
        void solve(const Eigen::VectorXd& comPosition, const Eigen::VectorXd &supportFootPosition, const Eigen::VectorXd& supportFootPositionPre, double footContraintSquareWidth,
                   double deltaXMax, double deltaYIn, double deltaYOut, bool supportFoot,
                   double simulationTime, double vRefX, double vRefY, double omegaRef);

        // Boolean variable denoting change of support foot
        bool supportFootHasChanged();

        // Vectors for Optimal CoM position, ZMP prediction, CoM velocity, foot steps
        Eigen::VectorXd getOptimalCoMPosition();
        Eigen::MatrixXd getOptimalZMPPrediction();
        Eigen::VectorXd getOptimalCoMVelocity();
        Eigen::VectorXd getOptimalFootsteps();

        // Function to add an obstacle
        void addObstacle(double posX, double posY, double radius);

    private:

        // Parameters
        int N,S,D,F,M, iterF,iter, footstepCounter;
        double singleSupport,doubleSupport, thetaMax;

        // Definition of P and p
        Eigen::VectorXd p;
        Eigen::MatrixXd P;

        Eigen::MatrixXd Aeq;
        Eigen::VectorXd beq;

        //Matrices for Constraints
        Eigen::MatrixXd Aleq;
        Eigen::VectorXd bleq;

        //Matrices for ZMP constraint
        Eigen::MatrixXd Icf;
        Eigen::MatrixXd Ic;
        Eigen::MatrixXd Cc;
        Eigen::VectorXd Ccf;
        Eigen::MatrixXd zmpRotationMatrix;
        Eigen::MatrixXd rCosZmp;
        Eigen::MatrixXd rSinZmp;
        Eigen::MatrixXd identity;
        Eigen::MatrixXd AZmp;
        Eigen::VectorXd bZmpMax;
        Eigen::VectorXd bZmpMin;
        Eigen::VectorXd bZmpLeft;
        Eigen::VectorXd bZmpRight;

        // Matrices for Footsteps constraints

        Eigen::MatrixXd Iprev;
        Eigen::MatrixXd AFootsteps;
        Eigen::VectorXd bFootstepsMax;
        Eigen::VectorXd bFootstepsMin;
        Eigen::VectorXd pFr;
        Eigen::VectorXd pFl;
        Eigen::VectorXd pF;
        Eigen::MatrixXd footstepsRotationMatrix;
        Eigen::MatrixXd rCosFootsteps;
        Eigen::MatrixXd rSinFootsteps;
        Eigen::VectorXd predictedRotations;

        // Matrices for cost functions
        Eigen::MatrixXd costFunctionH;
        Eigen::VectorXd costFunctionF;

        double qZd = 1;
        double qVx = 10;
        double qVy = 10;


        Eigen::MatrixXd identityCostFunctionH;

        Eigen::MatrixXd Vu;
        Eigen::MatrixXd Vs;

        Eigen::MatrixXd A_upd;
        Eigen::VectorXd B_upd;

        Eigen::VectorXd vArcX;
        Eigen::VectorXd vArcY;

        Eigen::VectorXd costFunctionF1;
        Eigen::VectorXd costFunctionF2;

        Eigen::VectorXd stateX;
        Eigen::VectorXd stateY;

        Eigen::VectorXd optimalCoMPosition;
        Eigen::VectorXd optimalCoMVelocity;
        Eigen::VectorXd optimalZMPPosition;
        Eigen::VectorXd optimalFootsteps;

        Eigen::VectorXd zOptimalX;
        Eigen::VectorXd zOptimalY;

        double obstaclePositionX;
        double obstaclePositionY;
        double obstacleRadius;
        double obstacleThetaObstacle;
        bool obstacleIsOn;

        double delta;

   };

}

#endif
