//
//  MPCSolver.cpp
//  reactiveWholeBodyHumanoids
//

#include "MPCSolver.hpp"
//#include <utils/Utils.hpp>
#include "qpOASES/qpOASES.hpp"
#include <stdlib.h>
#include <iostream>
#include "utils.cpp"

#ifdef VERBOSE_TASK
#define OUTPUT(x) std::cout<<x<<std::endl
#else
#define OUTPUT(x)
#endif

using namespace mpcSolver;

// Contructor called during initialization, i.e., when Controller constructor is called in main.cpp
MPCSolver::MPCSolver(double deltaT, double predictionTime, Eigen::VectorXd& comPosition,
                     const double singleSupportDuration, const double doubleSupportDuration, const double thetaMaxFootsteps){

    double omega=sqrt(9.81/comPosition(2));		// frequency of . . .

    delta = deltaT;								// . . .

    // Parameters
    singleSupport=singleSupportDuration;		// Duration of single support phase
    doubleSupport=doubleSupportDuration;		// Duration of double support phase
    N=round(predictionTime/delta);
    S=round(singleSupportDuration/delta);
    D=round(doubleSupportDuration/delta);
    F=(S+D)*2;
    M=ceil(N/(S+D));
    iterF=0;
    iter=0;

    p =  Eigen::VectorXd::Ones(N);
    P =  Eigen::MatrixXd::Ones(N,N)*delta;

    for(int i=0; i<N;++i){
        for(int j=0;j<N;++j){
            if (j>i) P(i,j)=0;
        }
    }


    // Matrices for stability constraint
    double lambda = exp(-omega*delta);
    Eigen::VectorXd b(N);

    for(int i=0;i<N;++i){
        b(i)=pow(lambda,i);
    }

    Aeq.resize(2,(N*2)+(M*2));
    Aeq.setZero();
    Aeq.block(0,0,1,N) =((1/omega)*((1-lambda)/(1-pow(lambda,N))))*b.transpose();
    Aeq.block(1,N+M,1,N) =((1/omega)*((1-lambda)/(1-pow(lambda,N))))*b.transpose();

    //    std::cout<<"AEQ: "<<std::endl;
    //    std::cout<< Aeq <<std::endl;

    // Built inside solve()
    beq.resize(2);
    beq.setZero();

    // Matrices for Constraints
    Aleq.resize(4*(N+M),2*(N+M));
    bleq.resize(4*(N+M));
    Aleq.setZero();
    bleq.setZero();

    // Matrices for ZMP constraint
    Icf = Eigen::MatrixXd::Zero(S+D,S+D);

    for(int i=0;i<S;++i){
        Icf(i,i)=1;
    }

    // Built inside solve()
    Ic.resize(N,N);
    Ic.setZero();

    // Built inside solve()
    Cc.resize(N,M);
    Cc.setZero();

    Ccf = Eigen::VectorXd::Ones(S+D);

    // Built inside solve()
    zmpRotationMatrix.resize(2*N,2*N);
    zmpRotationMatrix.setZero();

    rCosZmp.resize(N,N);
    rCosZmp.setZero();

    rSinZmp.resize(N,N);
    rSinZmp.setZero();

    identity = Eigen::MatrixXd::Identity(S+D,S+D);

    // Built inside solve()
    AZmp.resize(2*N,2*(N+M));
    bZmpMax.resize(2*N);
    bZmpMin.resize(2*N);

    AZmp.setZero();
    bZmpMax.setZero();
    bZmpMin.setZero();

    bZmpLeft.resize(2*N);
    bZmpRight.resize(2*N);

    // Matrices for Footsteps constraints

    // AFootsteps is built here, bFootstep inside solve()
    AFootsteps.resize(2*M,2*(M+N));
    bFootstepsMax.resize(2*M);
    bFootstepsMin.resize(2*M);

    AFootsteps.setZero();
    bFootstepsMax.setZero();
    bFootstepsMin.setZero();

    Iprev = Eigen::MatrixXd::Identity(M,M);

    for (int i=0; i<M-1; ++i){
        Iprev(i+1,i) = -1;
    }

    // pFr and pFl are built inside solve()
    pFr.resize(M);
    pFl.resize(M);
    pF.resize(M);

    pFr.setZero();
    pFl.setZero();
    pF.setOnes();

    footstepsRotationMatrix.resize(2*M,2*M);
    footstepsRotationMatrix.setZero();

    rCosFootsteps.resize(M,M);
    rCosFootsteps.setZero();

    rSinFootsteps.resize(M,M);
    rSinFootsteps.setZero();

    //    rCosFootsteps(0,0) = 1;
    //    rSinFootsteps(0,0) = 0;

    //    for(int i=1; i<M; ++i){
    //        rCosFootsteps(i,i) = cos(i*thetaNextFoot);
    //        rSinFootsteps(i,i) = sin(i*thetaNextFoot);
    //    }

    //    footstepsRotationMatrix << rCosFootsteps,-rSinFootsteps,
    //                               rSinFootsteps, rCosFootsteps;

    AFootsteps.block(0,N,M,M) = Iprev;
    AFootsteps.block(M,2*N+M,M,M) = Iprev;

    predictedRotations.resize(M+1);
    predictedRotations.setZero();

    //    AFootsteps=footstepsRotationMatrix*AFootsteps;

    // Matrices for cost functions are built inside solve()
    costFunctionH.resize(2*(N+M),2*(N+M));
    costFunctionF.resize(2*(N+M));

    costFunctionH.setZero();
    costFunctionF.setZero();

    identityCostFunctionH = Eigen::MatrixXd::Identity(N,N);

    Vu.resize(N,N);
    Vs.resize(N,3);
    Vu.setZero();
    Vs.setZero();

    double ch = cosh(omega*delta);
    double sh = sinh(omega*delta);

    A_upd.resize(3,3);
    B_upd.resize(3);
    A_upd<<ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
    B_upd<<delta-sh/omega,1-ch,delta;


    Eigen::RowVectorXd Vu_newline(N);
    Eigen::RowVectorXd Vs_newline(3);
    Eigen::RowVectorXd A_midLine(3);

    A_midLine<<omega*sh,ch,-omega*sh;

    for(int i=1;i<=N;++i) {
        Vu_newline.setZero();
        Vs_newline.setZero();
        Vu_newline(i-1) = 1-ch;
        if(i>1) {
            for(int j=0;j<=i-2;++j) {
                Vu_newline.segment(j,1) = A_midLine*(matrixPower(A_upd,(i-j-2)))*B_upd;
            }
        }
        Vs_newline = A_midLine*(matrixPower(A_upd,(i-1)));
        Vu.row(i-1) = Vu_newline;
        Vs.row(i-1) = Vs_newline;
    }


    Eigen::MatrixXd VuTVu=Vu.transpose()*Vu;

    costFunctionH.block(0,0,N,N) = qZd*identityCostFunctionH + qVx*VuTVu;
    costFunctionH.block(N+M,N+M,N,N) = qZd*identityCostFunctionH + qVy*VuTVu;


    // Built inside solve(), ?
    vArcX.resize(N);
    vArcY.resize(N);


    // Built inside solve(), ?
    costFunctionF1.resize(N+M);
    costFunctionF2.resize(N+M);

    costFunctionF1.setZero();
    costFunctionF2.setZero();

    // Built inside solve(), ?
    stateX.resize(3);
    stateY.resize(3);

    stateX.setZero();
    stateY.setZero();





    // Initially set optimalCoMPosition to actual CoM position
    optimalCoMPosition.resize(3);
    optimalCoMPosition(0)=comPosition(0);
    optimalCoMPosition(1)=comPosition(1);
    optimalCoMPosition(2)=comPosition(2);

    // Initially set optimalCoMVelocity to 0
    optimalCoMVelocity.resize(3);
    optimalCoMVelocity.setZero();

    // Initially set first 2 components of optimalZMPPosition equal to optimalCoMPosition; third component is 0
    optimalZMPPosition.resize(3);
    optimalZMPPosition.setZero(3);
    optimalZMPPosition(0) = optimalCoMPosition(0);
    optimalZMPPosition(1) = optimalCoMPosition(1);

    optimalFootsteps.resize(3);

    footstepCounter=0;

    obstaclePositionX=0;
    obstaclePositionY=0;
    obstacleRadius=0;
    obstacleThetaObstacle=0;
    obstacleIsOn=false;

    // Angle of . . .
    thetaMax = thetaMaxFootsteps;

    zOptimalX = Eigen::VectorXd::Zero(N);
    zOptimalY = Eigen::VectorXd::Zero(N);

}

//
void MPCSolver::solve(const Eigen::VectorXd& comPosition, const Eigen::VectorXd& supportFootPosition, const Eigen::VectorXd& supportFootPositionPre, double footContraintSquareWidth,
                                 double deltaXMax, double deltaYIn, double deltaYOut, bool supportFoot,
                                 double simulationTime, double vRefX, double vRefY, double omegaRef){

    // mSolver->solve(mSolver->getOptimalCoMPosition(), MPCSupportFootPosition, MPCSupportFootPosition, 0.04, 0.05, 0.10, 0.15, !supportFoot, counter*0.02, 0.0, 0.0, 0.0);



	// comPosition: optimalCoMPosition; in first instance this is the fixed value in Controller's constructor
	// supportFootPosition and supportFootPositionPre: MPCSupportFootPosition, i.e., distance between feet (sign depends on supportFoot boolean variable)


    double omega=sqrt(9.81/optimalCoMPosition(2));
    Cc.setZero();

    //    std::cout<<"iterF: "<<iterF<< " footstepCounter: " << footstepCounter << std::endl;

    // Translation that fixes reference frames issue.
    if(iterF == 0 && footstepCounter > 0){
        optimalCoMPosition(0)-=supportFootPositionPre(0);
        optimalCoMPosition(1)-=supportFootPositionPre(1);
        optimalZMPPosition(0)-=supportFootPositionPre(0);
        optimalZMPPosition(1)-=supportFootPositionPre(1);

        Eigen::MatrixXd newFootstepsRotationMatrix(3,3);
        newFootstepsRotationMatrix<<cos(supportFootPositionPre(5)), sin(supportFootPositionPre(5)),0,
                                    -sin(supportFootPositionPre(5)), cos(supportFootPositionPre(5)),0,
                                                              0,                         0, 1;

        // Change of coordinates (?) of optimalCoMPosition, optimalCoMVelocity, optimalZMPPosition
        optimalCoMPosition=newFootstepsRotationMatrix*optimalCoMPosition;
        optimalCoMVelocity=newFootstepsRotationMatrix*optimalCoMVelocity;
        std::cout<<"Did I crash?"<< optimalZMPPosition << std::endl;
        optimalZMPPosition=newFootstepsRotationMatrix*optimalZMPPosition;

    }



    Eigen::MatrixXd footstepsH(M,M);
    Eigen::VectorXd footstepsF(M);
    footstepsH.setZero();
    footstepsF.setZero();
    double qOmega=1;
    double qObstacle=0.05;
    double orientationGain=0.0;
    double distance=1;

    // NOT our case
    if(obstacleIsOn){
      distance = sqrt(pow(obstaclePositionX,2)+pow(obstaclePositionY,2));
      if(vRefX<0){
        if(!(obstacleThetaObstacle>-M_PI/2 && obstacleThetaObstacle<M_PI/2)){
          orientationGain=fabs(obstacleThetaObstacle)-M_PI/2;
        }
      }
      else{
        if((obstacleThetaObstacle>-M_PI/2 && obstacleThetaObstacle<M_PI/2)){
          orientationGain=-fabs(obstacleThetaObstacle)+M_PI/2;
        }
      }
    }


    Eigen::MatrixXd footstepsIdentity = Eigen::MatrixXd::Identity(M,M);
    footstepsH=qOmega*(Iprev.transpose()*Iprev)+qObstacle*footstepsIdentity;
    Eigen::VectorXd footstepsOnes = Eigen::VectorXd::Ones(M);
    double thetaAlign=wrapToPi(obstacleThetaObstacle-sign(obstacleThetaObstacle)*M_PI/2);
    footstepsF=-qOmega*omegaRef*(singleSupport+doubleSupport)*Iprev.transpose()*footstepsOnes
                - orientationGain*qObstacle*(thetaAlign/pow(distance,2))*footstepsOnes;

    qpOASES::real_t thetaOpt[M];

    qpOASES::Options optionsRotations;
    optionsRotations.printLevel=qpOASES::PL_NONE;
    qpOASES::QProblemB qpRotations(M);
    qpRotations.setOptions(optionsRotations);

    qpOASES::int_t nWSRRotations = 300;
    //    gettimeofday(&start, NULL);

    qpOASES::real_t footstepsHqpOASES[M*M];
    qpOASES::real_t footstepsFqpOASES[M];

    for(int i=0;i<M;++i){
      for(int j=0;j<M;++j){
        footstepsHqpOASES[i*M+j] = footstepsH(i,j);
      }
      footstepsFqpOASES[i] = footstepsF(i);
    }

    qpRotations.init(footstepsHqpOASES,footstepsFqpOASES,0,0,nWSRRotations);
    qpRotations.getPrimalSolution(thetaOpt);

    predictedRotations(0)=0;
    if(footstepCounter==0){
        for(int i=1;i<M;++i){
            predictedRotations(1) = 0;
            predictedRotations(i+1) = thetaOpt[i-1];
        }
    }
    else{
        for(int i=1;i<M+1;++i){
            predictedRotations(i) = thetaOpt[i-1];
        }
    }


    for(int i=0;i<M;++i){
      if(angDiff(predictedRotations(i+1),predictedRotations(i))>=thetaMax){
        std::cout<<"SATURAZIONE +THETAMAX"<<std::endl;
        predictedRotations(i+1)=wrapToPi(predictedRotations(i)+thetaMax);
      }
      else if(angDiff(predictedRotations(i+1),predictedRotations(i))<=-thetaMax){
        std::cout<<"SATURAZIONE -THETAMAX"<<std::endl;
        predictedRotations(i+1)=wrapToPi(predictedRotations(i)-thetaMax);
      }
    }

    // std::cout << predictedRotations << std::endl;

    //Matrices for stability constraint

    beq<<optimalCoMPosition(0)+(optimalCoMVelocity(0)/omega)-optimalZMPPosition(0),optimalCoMPosition(1)+(optimalCoMVelocity(1)/omega)-optimalZMPPosition(1);

    //    beq<<comPosition(0)+(optimalCoMVelocity(0)/omega)-optimalZMPPosition(0),comPosition(1)+(optimalCoMVelocity(1)/omega)-optimalZMPPosition(1);






    //Matrices for ZMP constraint
    if((int)simulationTime/delta<S+D){
        Ic.block(0,0,S+D-iterF,S+D-iterF) = Eigen::MatrixXd::Zero(S+D-iterF,S+D-iterF);
    }
    else{
        Ic.block(0,0,S+D-iterF,S+D-iterF) = Icf.block(iterF,iterF,S+D-iterF,S+D-iterF);
    }
    for(int i=0; i<M-1; ++i){
        Ic.block(S+D-iterF+(i*(S+D)),S+D-iterF+(i*(S+D)),S+D,S+D) = Icf;
    }

    Ic.block(S+D-iterF+((M-1)*(S+D)),S+D-iterF+((M-1)*(S+D)),iterF,iterF) = Icf.block(0,0,iterF,iterF);

    for(int i=0; i<M-1; ++i){
        Cc.block(S+D-iterF+(i*(S+D)),i,S+D,1) = Ccf;
    }

    Cc.block(S+D-iterF+((M-1)*(S+D)),M-1,iterF,1) = Ccf.block(0,0,iterF,1);

    rCosZmp.block(0,0,S+D-iterF,S+D-iterF) = identity.block(iterF,iterF,S+D-iterF,S+D-iterF);
    rSinZmp.block(0,0,S+D-iterF,S+D-iterF) = 0*identity.block(iterF,iterF,S+D-iterF,S+D-iterF);

    for(int i=0; i<M-1; ++i){
        rCosZmp.block(S+D-iterF+(i*(S+D)),S+D-iterF+(i*(S+D)),S+D,S+D) = identity*cos(predictedRotations(i+1));
        rSinZmp.block(S+D-iterF+(i*(S+D)),S+D-iterF+(i*(S+D)),S+D,S+D) = identity*sin(predictedRotations(i+1));
    }

    rCosZmp.block(S+D-iterF+((M-1)*(S+D)),S+D-iterF+((M-1)*(S+D)),iterF,iterF) = identity.block(0,0,iterF,iterF)*cos(predictedRotations(M));
    rSinZmp.block(S+D-iterF+((M-1)*(S+D)),S+D-iterF+((M-1)*(S+D)),iterF,iterF) = identity.block(0,0,iterF,iterF)*sin(predictedRotations(M));


    zmpRotationMatrix << rCosZmp,rSinZmp,
                         -rSinZmp,rCosZmp;

    AZmp.block(0,0,N,N) = Ic*P;
    AZmp.block(0,N,N,M) = -Ic*Cc;
    AZmp.block(N,N+M,N,N) = Ic*P;
    AZmp.block(N,2*N+M,N,M) = -Ic*Cc;

    Eigen::MatrixXd AZmpRot(2*N,2*(M+N));
    AZmpRot = zmpRotationMatrix*AZmp;

    bZmpLeft<<Ic*p*footContraintSquareWidth/2,Ic*p*footContraintSquareWidth/2;

    bZmpRight<<Ic*p*optimalZMPPosition(0),Ic*p*optimalZMPPosition(1);
    bZmpRight=zmpRotationMatrix*bZmpRight;

    bZmpMax=bZmpLeft-bZmpRight;
    bZmpMin=bZmpLeft+bZmpRight; //Probably add something for first step!



    // Matrices for Footsteps constraints
    rCosFootsteps(0,0) = 1;
    rSinFootsteps(0,0) = 0;

    for(int i=1; i<M; ++i){
        rCosFootsteps(i,i) = cos(predictedRotations(i));
        rSinFootsteps(i,i) = sin(predictedRotations(i));
    }

    footstepsRotationMatrix << rCosFootsteps, rSinFootsteps,
                               -rSinFootsteps, rCosFootsteps;

    Eigen::MatrixXd AFootstepsRot(2*M,2*(M+N));
    AFootstepsRot=footstepsRotationMatrix*AFootsteps;

    pFl.setZero();
    pFr.setZero();

    if(supportFoot==true){
        for(int i=0;i<M;++i){
            if(i%2==0) pFr(i) = 1;
            else pFl(i) = 1;
        }
    }
    else{
        for(int i=0;i<M;++i){
            if(i%2==0) pFl(i) = 1;
            else pFr(i) = 1;
        }
    }


    bFootstepsMax<<pF*deltaXMax,-pFl*deltaYIn+pFr*deltaYOut;
    bFootstepsMin<<pF*deltaXMax,pFl*deltaYOut-pFr*deltaYIn;

    Aleq<<AZmp,-AZmp, AFootsteps*0,-AFootsteps*0;
    bleq<<bZmpMax,bZmpMin,bFootstepsMax*0,bFootstepsMin*0;






    // Matrices for cost functions
    for(int i=0;i<N;++i){
        vArcX(i) = vRefX*cos(i*omegaRef*delta);
        vArcY(i) = vRefX*sin(i*omegaRef*delta);  // Insert vRefY?
    }







    // States X,Y are x and y components (ground) of optimalCoMPosition, optmalCoMVelocity and optimalZMPPosition
    stateX<<optimalCoMPosition(0),optimalCoMVelocity(0),optimalZMPPosition(0);
    stateY<<optimalCoMPosition(1),optimalCoMVelocity(1),optimalZMPPosition(1);

    //    stateX<<comPosition(0),optimalCoMVelocity(0),optimalZMPPosition(0);
    //    stateY<<comPosition(1),optimalCoMVelocity(1),optimalZMPPosition(1);


    costFunctionF1.block(0,0,N,1) = (qVx*Vu.transpose())*((Vs*stateX)-vArcX);
    costFunctionF2.block(0,0,N,1) = (qVy*Vu.transpose())*((Vs*stateY)-vArcY);

    // Cost functions (unknown the process to compute them)
    costFunctionF<<costFunctionF1,costFunctionF2;

    int nStability = 2;
    int nZMP = 2*N;
    int nFootsteps = 2*M;
    int nInitial = 2;
    int nVariables = 2*(N+M);
    int nDoubleSupport = 2;

    qpOASES::real_t H[nVariables*nVariables];
    qpOASES::real_t g[nVariables];
    qpOASES::real_t A[(nStability+nZMP+nFootsteps+nInitial+nDoubleSupport)*nVariables];
    qpOASES::real_t lb[nStability+nZMP+nFootsteps+nInitial+nDoubleSupport];
    qpOASES::real_t ub[nStability+nZMP+nFootsteps+nInitial+nDoubleSupport];

    Eigen::MatrixXd firstConstraint(nInitial,nVariables);
    Eigen::VectorXd firstConstraintRight(nInitial);
    firstConstraint.setZero();
    firstConstraintRight.setZero();

    if(simulationTime<singleSupport+doubleSupport){
        firstConstraint(0,N) = 1;
        firstConstraint(1,2*N+M) = 1;
        firstConstraintRight(0)=supportFootPosition(0);
        firstConstraintRight(1)=supportFootPosition(1);
    //        std::cout<<"firstConstraint "<< firstConstraint <<std::endl;
    //        std::cout<<"firstConstraintRight "<< firstConstraintRight <<std::endl;
    //        std::cout<<"Position swingFoot: "<< supportFootPosition.transpose() <<std::endl;
    }

    Eigen::MatrixXd doubleSupportConstraint(nDoubleSupport,nVariables);
    Eigen::VectorXd doubleSupportConstraintRight(nDoubleSupport);
    doubleSupportConstraint.setZero();
    doubleSupportConstraintRight.setZero();

    if(iterF>S){
      doubleSupportConstraint(0,N) = 1;
      doubleSupportConstraint(1,2*N+M) = 1;
      doubleSupportConstraintRight(0)=supportFootPosition(0);
      doubleSupportConstraintRight(1)=supportFootPosition(1);
    }


    for(int i=0;i<nVariables;++i){
        for(int j=0;j<nVariables;++j){
            H[i*nVariables+j] = costFunctionH(i,j);
        }
        g[i] = costFunctionF(i);
    }

    for(int i=0;i<nStability;++i){
        for(int j=0;j<nVariables;++j){
            A[i*nVariables+j] = Aeq(i,j);
        }
        lb[i] = beq(i);
        ub[i] = beq(i);
    }

    for(int i=0;i<nFootsteps;++i){
        for(int j=0;j<nVariables;++j){
            A[(i+nStability)*nVariables+j] = AFootstepsRot(i,j);
        }
        ub[i+nStability] = bFootstepsMax(i);
        lb[i+nStability] = -bFootstepsMin(i);
    }

    for(int i=0;i<nZMP;++i){
        for(int j=0;j<nVariables;++j){
            A[(i+nFootsteps+nStability)*nVariables+j] = AZmpRot(i,j);
        }
        ub[i+nFootsteps+nStability] = bZmpMax(i);
        lb[i+nFootsteps+nStability] = -bZmpMin(i);
    }

    for(int i=0;i<nInitial;++i){
        for(int j=0;j<nVariables;++j){
            A[(i+nZMP+nFootsteps+nStability)*nVariables+j] = firstConstraint(i,j);
        }
        lb[i+nZMP+nFootsteps+nStability] = firstConstraintRight(i);
        ub[i+nZMP+nFootsteps+nStability] = firstConstraintRight(i);
    }

    for(int i=0;i<nDoubleSupport;++i){
        for(int j=0;j<nVariables;++j){
            A[(i+nZMP+nFootsteps+nStability+nInitial)*nVariables+j] = doubleSupportConstraint(i,j);
        }
        lb[i+nZMP+nFootsteps+nStability+nInitial] = doubleSupportConstraintRight(i);
        ub[i+nZMP+nFootsteps+nStability+nInitial] = doubleSupportConstraintRight(i);
    }



    //    struct timeval start, end;

    qpOASES::real_t xOpt[nVariables];

    qpOASES::Options options;
    options.printLevel=qpOASES::PL_NONE;
    qpOASES::QProblem qp(nVariables, nStability+nZMP+nFootsteps+nInitial+nDoubleSupport);
    qp.setOptions(options);

    qpOASES::int_t nWSR = 300;
    //    gettimeofday(&start, NULL);

    qp.init(H,g,A,0,0,lb,ub,nWSR);
    qp.getPrimalSolution(xOpt);

    //    gettimeofday(&end, NULL);
    //    printf("%ld\n", ((end.tv_sec * 1000000 + end.tv_usec)
    //              - (start.tv_sec * 1000000 + start.tv_usec)));

    Eigen::VectorXd zDotOptimalX(N);
    Eigen::VectorXd zDotOptimalY(N);
    Eigen::VectorXd footstepsOptimalX(M);
    Eigen::VectorXd footstepsOptimalY(M);
    Eigen::VectorXd decisionVariables(2*(N+M));

    for(int i=0;i<2*(N+M);++i){
        decisionVariables(i) = xOpt[i];
    }


    zDotOptimalX = (decisionVariables.head(N));
    zDotOptimalY = (decisionVariables.segment(N+M,N));

    footstepsOptimalX = decisionVariables.segment(N,M);
    footstepsOptimalY = decisionVariables.segment(2*N+M,M);

    zOptimalX = p*optimalZMPPosition(0) + P*zDotOptimalX;
    zOptimalY = p*optimalZMPPosition(1) + P*zDotOptimalY;

    // PRINT ON FILE FOR DEBUGGING
    // std::ofstream logZMPx;
    // logZMPx.open("/home/dado/Software/Matlab/MPCPlot/Zmpx.txt", std::ofstream::app);
    // logZMPx<<zOptimalX << "\n";
    // logZMPx.close();
    //
    // std::ofstream logZMPy;
    // logZMPy.open("/home/dado/Software/Matlab/MPCPlot/Zmpy.txt", std::ofstream::app);
    // logZMPy<<zOptimalY << "\n";
    // logZMPy.close();
    //
    // std::ofstream logFootstepsx;
    // logFootstepsx.open("/home/dado/Software/Matlab/MPCPlot/Footstepsx.txt", std::ofstream::app);
    // logFootstepsx<<footstepsOptimalX << "\n";
    // logFootstepsx.close();
    //
    // std::ofstream logFootstepsy;
    // logFootstepsy.open("/home/dado/Software/Matlab/MPCPlot/Footstepsy.txt", std::ofstream::app);
    // logFootstepsy<<footstepsOptimalY << "\n";
    // logFootstepsy.close();
    //----------------------------------------------------------------------//



    // The most important step: how get optimalCoMPosition, optimalCoMVelocity, optimalZMPPosition (and optimalFootSteps)
    // ????
    Eigen::VectorXd stateXNext = A_upd*stateX + B_upd*zDotOptimalX(0);
    Eigen::VectorXd stateYNext = A_upd*stateY + B_upd*zDotOptimalY(0);

    optimalCoMPosition(0)=stateXNext(0);
    optimalCoMPosition(1)=stateYNext(0);

    optimalCoMVelocity(0)=stateXNext(1);
    optimalCoMVelocity(1)=stateYNext(1);
    optimalCoMVelocity(2)=0.0;

    optimalZMPPosition(0) = stateXNext(2);
    optimalZMPPosition(1) = stateYNext(2);

    optimalFootsteps(0) = footstepsOptimalX(0);
    optimalFootsteps(1) = footstepsOptimalY(0);
    optimalFootsteps(2) = predictedRotations(1);

    ++iterF;
    if(iterF>=S+D){
        iterF=0;
        footstepCounter++;
    }
}
bool MPCSolver::supportFootHasChanged(){
    if (iterF==0)
        return true;
    return false;
}
Eigen::VectorXd MPCSolver::getOptimalCoMPosition(){
    return optimalCoMPosition;
}
Eigen::MatrixXd MPCSolver::getOptimalZMPPrediction(){
	Eigen::MatrixXd optimalZMPPrediction(2,N);
	optimalZMPPrediction << zOptimalX.transpose(), zOptimalY.transpose();
    return optimalZMPPrediction;
}
Eigen::VectorXd MPCSolver::getOptimalCoMVelocity(){
    return optimalCoMVelocity;
}
Eigen::VectorXd MPCSolver::getOptimalFootsteps(){
    return optimalFootsteps;
}

// NOT impotant for us
void MPCSolver::addObstacle(double posX, double posY, double radius){
    obstaclePositionX=posX;
    obstaclePositionY=posY;
    obstacleRadius=radius;
    obstacleThetaObstacle=wrapToPi(atan2(posY,posX));
    obstacleIsOn=true;
}
