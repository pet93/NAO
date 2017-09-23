/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.hpp"

//==============================================================================
// Constructor that:
// 1) Set and compute initial position of right and left feet
// 2) Use MPCSolver constructor (?)

Controller::Controller(dart::dynamics::SkeletonPtr _robot,		// input1: robot
                       dart::dynamics::BodyNode* _supportFoot,	// input2: supportFoot
					   dart::dynamics::BodyNode* _swingFoot,	// input3: swingFoot
					   dart::simulation::WorldPtr _world)		// input4: world

// Here ':' is used to initialize variables of input
: mRobot(_robot),
    mSupportFoot(_supportFoot),
	//mSwingFoot(_supportFoot),
	mSwingFoot(_swingFoot),
	mWorld(_world)

{
  assert(_robot != nullptr);
  assert(_supportFoot != nullptr);
  assert(_swingFoot != nullptr);

  // Get number of dofs
  int dof = mRobot->getNumDofs();		// 30 dofs


  // Get CoM position of robot
  Eigen::VectorXd CoMPosition = mRobot->getCOM();

  // MPC CoM position object declaration and resize
  Eigen::VectorXd MPCCoMPosition;
  MPCCoMPosition.resize(3);

  //MPCCoMPosition << CoMPosition(0), -CoMPosition(2), CoMPosition(1); (?) If removed no effects
  MPCCoMPosition << 0.02, -0.05, 0.31;

  // MPCSolver contructor called (implementation in MPCSolver.cpp) -> MPCSolver.cpp
  // The code is quite difficult, I think these parameters are standard (must not change them)
  // In fact this function is an initialization of parameters used next concerning optimal trajectories in MPCSolver
  mSolver = new mpcSolver::MPCSolver(0.02, 0.6, MPCCoMPosition, 0.2, 0.1, M_PI/16);


  // Initial position and orientation of feet (getCOM() here is referred to feet's CoM, not to CoM of whole robot)
  // Vector 'r' is the rotation matrix between left (right) foot wrt WRF
  // NOTE: This step is executed only once

  /*Eigen::MatrixXd r = mRobot->getBodyNode("l_wrist")->getWorldTransform().rotation();
  initialPosL << mRobot->getBodyNode("l_wrist")->getCOM(), atan2(r(1,0),r(0,0)), atan2(-r(2,0),sqrt(r(2,1)*r(2,1)+r(2,2)*r(2,2))), atan2(r(2,1),r(2,2));
  r = mRobot->getBodyNode("r_wrist")->getWorldTransform().rotation();
  initialPosR << mRobot->getBodyNode("r_wrist")->getCOM(), atan2(r(1,0),r(0,0)), atan2(-r(2,0),sqrt(r(2,1)*r(2,1)+r(2,2)*r(2,2))), atan2(r(2,1),r(2,2));
*/

  Eigen::MatrixXd r = mRobot->getBodyNode("l_sole")->getWorldTransform().rotation();
      initialPosL << mRobot->getBodyNode("l_sole")->getCOM(), atan2(r(1,0),r(0,0)), atan2(-r(2,0),sqrt(r(2,1)*r(2,1)+r(2,2)*r(2,2))), atan2(r(2,1),r(2,2));
      r = mRobot->getBodyNode("r_sole")->getWorldTransform().rotation();
      initialPosR << mRobot->getBodyNode("r_sole")->getCOM(), atan2(r(1,0),r(0,0)), atan2(-r(2,0),sqrt(r(2,1)*r(2,1)+r(2,2)*r(2,2))), atan2(r(2,1),r(2,2));



  // Change last position coordinate; if removed no effects. The value inserted in general is not stable
  //initialPosL(2) = 0.0260773;
  //initialPosR(2) = 0.0260773;

  // ?, If removed no effects
  supportFoot = true;

  // Remove position limits
  for (int i = 0; i < dof; ++i)
    _robot->getJoint(i)->setPositionLimitEnforced(false);

  // Set joint damping
  for (int i = 0; i < dof; ++i)
    _robot->getJoint(i)->setDampingCoefficient(0, 0.5);
}

//==============================================================================
// No destructor action

Controller::~Controller()
{
}

//==============================================================================
// Function called by MyWindow.cpp that:
// 1) Compute total momentum
// 2) Set counter
// 3) 'solve' function (?)
// 4) Position and velocity tasks for feet and CoM (another source of command has to impose desired values)
// 5) Inverse kinematics (feedback control law)

void Controller::update(Eigen::Vector3d _targetPosition)		// where _targetPosition affects the code?
{
  using namespace dart;

// Name of main body components' orientation (roll-pitch-yaw)
// VERY IMPORTANT NOTE: These 24 joints are ordered from 6 to 30. First 6 joints (0 to 5) are total robot roll-pitch-yaw and vx, vy, vz
  std::string joints[] =
    {"HeadYaw",
    "HeadPitch",
    "LHipYawPitch",
    "LHipRoll",
    "LHipPitch",
    "LKneePitch",
    "LAnklePitch",
    "LAnkleRoll",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
    "RHipYawPitch",
    "RHipRoll",
    "RHipPitch",
    "RKneePitch",
    "RAnklePitch",
    "RAnkleRoll",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw"};


  std::string leg_joints[] =
      {"LHipYawPitch",
      "LHipRoll",
      "LHipPitch",
      "LKneePitch",
      "LAnklePitch",
      "LAnkleRoll",
      "RHipYawPitch",
      "RHipRoll",
      "RHipPitch",
      "RKneePitch",
      "RAnklePitch",
      "RAnkleRoll"};


  std::string LLeg_joints[] =
        {"LHipYawPitch",
        "LHipRoll",
        "LHipPitch",
        "LKneePitch",
        "LAnklePitch",
        "LAnkleRoll"};

  std::string RLeg_joints[] =
        {"RHipYawPitch",
        "RHipRoll",
        "RHipPitch",
        "RKneePitch",
        "RAnklePitch",
        "RAnkleRoll"};




  // Get Centre of Mass of robot
  Eigen::Vector3d CoMPosition = mRobot->getCOM();

  // Generate vector of total momentum of robot
  Eigen::VectorXd TotalMom;
  TotalMom.resize(3);  // resizing
  TotalMom.setZero();


  for (int i=0; i<7; i++){

	  // Get momenta (linear and angular) of all bodies (7)
	  Eigen::Vector3d LinearMom = (mRobot->getBodyNode(i))->getLinearMomentum();
	  Eigen::Vector3d AngularMom = (mRobot->getBodyNode(i))->getAngularMomentum();

	  // Get i-th component's CoM and compute the rotation matrix between local RF wrt WRF
	  Eigen::Vector3d iCoMPosition = (mRobot->getBodyNode(i))->getCOM();
	  Eigen::Matrix3d iRotation = (mRobot->getBodyNode(i))->getWorldTransform().rotation();

	  // Transform the momentums in world frame
	  LinearMom = iRotation*LinearMom;
	  AngularMom = iRotation*AngularMom;

	  // Compute total momentum
	  TotalMom = TotalMom + (iCoMPosition-CoMPosition).cross(LinearMom) + AngularMom;
  }


  //std::cout << "TotalMom\n" << TotalMom << std::endl;





  // GENERATE TASKS !!!

  // Get time step
  double timeStep = mWorld->getTimeStep();
  	  	  	  	  	  	  	  	  	  	  	  	  // In free time search the difference between 'timeStep' and 'counter'
  // Set time counter
  int counter = mWorld->getSimFrames();

  // Create vectors of actual position of left and right feet (6 components: R3xSO(3))
  // and of CoM (3 components: R3)
  Eigen::VectorXd actualPosL(6);
  Eigen::VectorXd actualPosR(6);
  Eigen::VectorXd actualPosCoM(3);

  // Compute actual position and orientation (wrt WRF) of left and right feet and of CoM
  // The method is the same of that concerning the initial position in the constructor

  /*Eigen::MatrixXd r = mRobot->getBodyNode("l_wrist")->getWorldTransform().rotation();
  actualPosL << mRobot->getBodyNode("l_wrist")->getCOM(), atan2(r(1,0),r(0,0)), atan2(-r(2,0),sqrt(r(2,1)*r(2,1)+r(2,2)*r(2,2))), atan2(r(2,1),r(2,2));
  r = mRobot->getBodyNode("r_wrist")->getWorldTransform().rotation();
  actualPosR << mRobot->getBodyNode("r_wrist")->getCOM(), atan2(r(1,0),r(0,0)), atan2(-r(2,0),sqrt(r(2,1)*r(2,1)+r(2,2)*r(2,2))), atan2(r(2,1),r(2,2));
  actualPosCoM = mRobot->getCOM(); */

  Eigen::MatrixXd r = mRobot->getBodyNode("l_sole")->getWorldTransform().rotation();
    actualPosL << mRobot->getBodyNode("l_sole")->getCOM(), atan2(r(1,0),r(0,0)), atan2(-r(2,0),sqrt(r(2,1)*r(2,1)+r(2,2)*r(2,2))), atan2(r(2,1),r(2,2));
    r = mRobot->getBodyNode("r_sole")->getWorldTransform().rotation();
    actualPosR << mRobot->getBodyNode("r_sole")->getCOM(), atan2(r(1,0),r(0,0)), atan2(-r(2,0),sqrt(r(2,1)*r(2,1)+r(2,2)*r(2,2))), atan2(r(2,1),r(2,2));
    actualPosCoM = mRobot->getCOM();

  // Show time counter
 // std::cout << "counter: " << counter << std::endl;





  // Create a 6D vector of MPC Support Foot Position
  	Eigen::VectorXd MPCSupportFootPosition;
    MPCSupportFootPosition.resize(6);





  // I THINK this instruction needs only for getOmniDirectionalSwingFootTrajectory, i.e., only for feet trajectory temporization!
    if (counter%15==0 && counter>0) {  // %15
      indInitial = counter;
  	  supportFoot = !supportFoot;
  	 // std::cout << "supportFoot is: " << supportFoot << std::endl;
    }

  // Compute distance between feet and choose the sign basing on 'supportFoot' boolean variable
    if (supportFoot==true){
  	  //MPCSupportFootPosition << actualPosR(0), actualPosR(2), actualPosR(1), actualPosR(3), actualPosR(4), actualPosR(5);
  	  MPCSupportFootPosition = actualPosL-actualPosR;
    }
    else {
  	  //MPCSupportFootPosition << actualPosL(0), actualPosL(2), actualPosL(1), actualPosL(3), actualPosL(4), actualPosL(5);
  	  MPCSupportFootPosition = actualPosR-actualPosL;
    }
    //MPCSupportFootPosition.setZero();

    // Function 'solve' of MPCSolver class (implementation in MPCSolver.cpp)  -> MPCSolver.cpp  (no effects if removed)
    // Too difficult to understand. . .
    //mSolver->solve(mSolver->getOptimalCoMPosition(), MPCSupportFootPosition, MPCSupportFootPosition, 0.04, 0.05, 0.10, 0.15, !supportFoot, counter*0.02, 0.0, 0.0, 0.0);







  // Create vectors of desired velocity(linear and angular) and desired pose (R3xSO(3)) for left foot
  Eigen::VectorXd desVelL(6);
  Eigen::VectorXd desPosL(6);

  // Create vectors of desired velocity(linear and angular) and desired pose (R3xSO(3)) for right foot
  Eigen::VectorXd desVelR(6);
  Eigen::VectorXd desPosR(6);

  // Create vectors of desired velocity(linear) and desired position (R3) for CoM
  Eigen::VectorXd desVelCoM(3);
  Eigen::VectorXd desPosCoM(3);

  // Create vectors of total desired velocity, total actual pose, total desired pose
  // (NOT total actual velocity)
  Eigen::VectorXd desVelTotal(15);
  Eigen::VectorXd actualPosTotal(15);
  Eigen::VectorXd desPosTotal(15);








/*

// LEFT FOOT TASK. Well, it works
// IMPORTANT NOTE: even though we are dealing with left foot, we need to impose also desVelR, desPosR, desPosCoM, desVelCoM
//				   The same holds for another tasks
// NOTE: Left foot task can work together with another task such as CoM task
  double vRefSwg = 0.5/8;   // 0.05/2

  desPosL << 0.075, actualPosL(1),actualPosL(2),actualPosL(3),actualPosL(4),actualPosL(5);   // Print error if error==0 stop , make if error==0 des = actual
  //desPosL << actualPosL(0), actualPosL(1),actualPosL(2),actualPosL(3),actualPosL(4),actualPosL(5);   // Print error if error==0 stop , make if error==0 des = actual

  //desVelL << 0,0,vRefSwg,0,0,0; //desVelL << 0,0,0,vRefSwg,0,vRefSwg;
  desVelL << 0,0,0,0,0,0;
  //desPosL = actualPosL + desVelL*timeStep;	// desired pose is simply the actual + desired velocity(=0) * time = actualPosL
  desVelR << 0,0,0,0,0,0;

  desPosR = actualPosR;
  //desPosR = actualPosR + desVelR*timeStep;

  desVelCoM << 0,0,0;
  //desPosCoM = actualPosCoM + desVelCoM*timeStep;
  desPosCoM = actualPosCoM;

*/





/*
// THIS PLANNING PRODUCE MOTION BY EXPLOITING SWINGFOOT TRAJCTORY FUNCTION (to use it I need to know changeRF6)
  if (supportFoot==true){
	  Eigen::MatrixXd swingFootTask = getOmnidirectionalSwingFootTrajectoryMPC(mSolver->getOptimalFootsteps(), changeRF6*MPCSupportFootPosition, counter, 0.01, 10, 5, indInitial, 0.02);
	  desVelL = changeRF6.transpose()*swingFootTask.col(0);
	  desPosL = actualPosR + changeRF6.transpose()*swingFootTask.col(1);
//	  std::cout << changeRF6.transpose()*desPosL << std::endl;
	  desVelR << 0,0,0,0,0,0;					// desired velocity of right foot is 0
	  desPosR = actualPosR + desVelR*timeStep;


	  desVelL << 0,0,0,0,0,0;					// desired velocity of left foot is 0
	  desPosL = actualPosL + desVelL*timeStep;	// desired pose is simply the actual + desired velocity(=0) * time = actualPosL
	  desPosL(1) = 46;							// 2nd component of desired pose of left foot set to 46	  	  	  	  	  	  	  	  	  	  	  	// (no effects since desPosL is initialized again below)
	  desVelR << 0,0,0,0,0,0;					// desired velocity of right foot is 0
	  desPosR = actualPosR + desVelR*timeStep;	// desired pose is simply the actual + desired velocity(=0) * time = actualPosR
	  desPosR(1) = 0; 							// 2nd component of desired pose of right foot set to 0 (no effects as desPosL)

  }
  else {

	  desVelL << 0,0,0,0,0,0;					// desired velocity of left foot is 0
	  desPosL = actualPosL + desVelL*timeStep;	// desired pose is simply the actual + desired velocity(=0) * time = actualPosL
	  desPosL(1) = 0;							// ........... as above 0
	  desVelR << 0,0,0,0,0,0;					// .....
	  desPosR = actualPosR + desVelR*timeStep;	// .....
	  desPosR(1) = 0;						// ..... 0


	  desVelL << 0,0,0,0,0,0;					// desired velocity of left foot is 0
	  desPosL = actualPosL + desVelL*timeStep;
	  Eigen::MatrixXd swingFootTask = getOmnidirectionalSwingFootTrajectoryMPC(mSolver->getOptimalFootsteps(), changeRF6*MPCSupportFootPosition, counter, 0.01, 10, 5, indInitial, 0.02);
	  desVelR = changeRF6.transpose()*swingFootTask.col(0);
	  desPosR = actualPosL + changeRF6.transpose()*swingFootTask.col(1);
//
//	  std::cout << desPosR << std::endl;
  }
*/






  // THIS PLANNING PRODUCE NO MOTION
  // When counter < 1500 time steps: initial positions' 3rd component are initialized as actual ones' 3rd component
  // Desired velocities for both feet are 0 and therefore desired pose of both feet are the initial ones
  // IMPORTANT NOTE: even though initial (=desired) positions and actual positions are the same, an error is generated!!!
  //				 This is the motivation for which robot moves, even though desired velocity is 0, under a feedback control law
  //				 that exploits a position error.
  // NOTE: actualPos CHANGE, initialPos DOESN'T change !!!

  /*if (counter<1500){ // counter<1500
	  initialPosL(2) = actualPosL(2);
	  initialPosR(2) = actualPosR(2);

	  // By adding these further 5 equalities robot doesn't generate any error since initialPos is updated at each step
	  initialPosL(0) = actualPosL(0);
	  initialPosR(0) = actualPosR(0);

	  initialPosL(1) = actualPosL(1);
	  initialPosR(1) = actualPosR(1);

	  initialPosL(3) = actualPosL(3);
	  initialPosR(3) = actualPosR(3);

	  initialPosL(4) = actualPosL(4);
	  initialPosR(4) = actualPosR(4);

	  initialPosL(5) = actualPosL(5);
	  initialPosR(5) = actualPosR(5);

	  desVelL << 0,0,0,0,0,0;
	  desPosL = initialPosL + desVelL*timeStep;
	  desVelR << 0,0,0,0,0,0;
	  desPosR = initialPosR + desVelR*timeStep;

	  std::cout << actualPosR(2) << std::endl;
  } */





/*

// RIGHT FOOT TASK

  desVelR << 0,0,0,0,0,0;

  desPosR << 0.075,actualPosR(1),actualPosR(2),actualPosR(3), actualPosR(4), actualPosR(5) ;

  desVelL << 0,0,0,0,0,0;
  desPosL = actualPosL + desVelL*timeStep;

  desVelCoM << 0,0,0;
  desPosCoM << actualPosCoM(0), actualPosCoM(1), actualPosCoM(2);
  //desPosCoM = actualPosCoM + desVelCoM*timeStep;

	//std::cout << "x is: " << actualPosR(0) << std::endl;

	if(counter>5000)
	std::cout << "X is: " << actualPosR(0) << std::endl;
	std::cout << "Y is: " << actualPosR(1) << std::endl;
	std::cout << "Z is: " << actualPosR(2) << std::endl;
	std::cout << "init is: " << actualPosR(3) << std::endl;
	std::cout << "init is: " << actualPosR(4) << std::endl;
	std::cout << "init is: " << actualPosR(5) << std::endl;

	if(actualPosR(0)>=0.075 || counter==0){
		std::cout << actualPosR << std::endl << std::endl;
		std::cout << desPosR << std::endl;
		std::cout << desPosR(3) - actualPosR(0) << std::endl;
		std::cout << desPosR - actualPosR << std::endl;
		int t=1;
		t=0;
	}

*/






/*

// CoM TASK 1
  	double vRefCoM = 0.05; // 0.05/2
  		//desVelCoM << 0,vRefCoM,0;
  		//desVelCoM << 0,0,vRefCoM;   // CoM moves toward direction of line joining feet
  		desVelCoM << vRefCoM,0,0;     // CoM moves toward direction of feet


	desPosCoM = actualPosCoM + desVelCoM*timeStep;
  	desVelL << 0,0,0,0,0,0;
  	desPosL = actualPosL + desVelL*timeStep;
  	desVelR << 0,0,0,0,0,0;
  	desPosR = actualPosR + desVelR*timeStep;

*/





/*
// CoM TASK 2
    desVelCoM = mSolver->getOptimalCoMVelocity();
    desPosCoM = getSupportFoot()->getCOM() + mSolver->getOptimalCoMPosition();

    desVelL << 0,0,0,0,0,0;
    desPosL = actualPosL + desVelL*timeStep;
    desVelR << 0,0,0,0,0,0;
    desPosR = actualPosR + desVelR*timeStep;



*/





// THIS PLANNING PRODUCES NO MOTION OF COM
  // Set to 0 the desired CoM linear velocity
  //desVelCoM << 0,0,0;
  //desPosCoM = actualPosCoM + desVelCoM*timeStep;
  // Desired position of CoM is the actual + desired velocity(=0) * timeStep = actualPosCoM
  //desPosCoM = actualPosCoM + desVelCoM*timeStep;









/*

  // Experiment 2: CoM circle by means velocity command (see MyWindow::timestepping). Done!
    	double vRefCoM = 0.05/2; // 0.05/2
    	if (counter <= 600)      // 200
    		desVelCoM << 0,0.1*std::cos(0.785*counter),0.1*std::sin(0.785*counter);
    		//desVelCoM << 0,vRefCoM,0;   // CoM moves toward direction of line joining feet
    		//desVelCoM << vRefCoM,0,0;     // CoM moves toward direction of feet
    	//else if (counter<=200)
    	//	desVelCoM << 0,0,-vRefCoM;
    	else
    		desVelCoM << 0,0,0;

  	desPosCoM = actualPosCoM + desVelCoM*timeStep;
    	desVelL << 0,0,0,0,0,0;
    	desPosL = actualPosL + desVelL*timeStep;
    	desVelR << 0,0,0,0,0,0;
    	desPosR = actualPosR + desVelR*timeStep;

*/








/*
  // Experiment 3: CoM motion by means position command. Done
  // NOTE: since the control law exploits a feedback position error and this error converges to 0 (Lyapunov and LaSalle proof)
  //	   it is possible to make the CoM a circle by using position commands instead of velocity commands

      	//desPosCoM << actualPosCoM(0),actualPosCoM(1) + 0.5*std::cos(0.785*counter),actualPosCoM(2) + 0.5*std::sin(0.785*counter);
      	desPosCoM << 0.04, actualPosCoM(1), actualPosCoM(2);
      	//desPosCoM << actualPosCoM(0), actualPosCoM(1), 0.02;

		desVelCoM << 0,0,0;
      	desVelL << 0,0,0,0,0,0;
      	desVelR << 0,0,0,0,0,0;
      	desPosL = actualPosL + desVelL*timeStep;
      	desPosR = actualPosR + desVelR*timeStep;



  // Initial position of CoM is: 0.028, 0.0003, 0.354

if(actualPosCoM(0)>=0.07 || counter > 0){
std::cout << "XCoM: " << actualPosCoM(0) << std::endl;
//std::cout << actualPosCoM(1) << std::endl;
//std::cout << actualPosCoM(2) << std::endl;
//std::cout << "position error: " << desPosCoM - actualPosCoM << std::endl;

} */











    // Experiment 4: CoM circle (hoola-hoop) by means position command. Done
    // NOTE: since the control law exploits a feedback position error and this error converges to 0 (Lyapunov and LaSalle proof)
    //	   it is possible to make the CoM a circle by using position commands instead of velocity commands

  	  //desPosCoM << actualPosCoM(0), actualPosCoM(1), actualPosCoM(2);
        desPosCoM << actualPosCoM(0) + 0.015*std::cos(0.785*counter),actualPosCoM(1) + 0.015*std::sin(0.785*counter), actualPosCoM(2);
        //desPosCoM << 0.028,0, 0.35;

        desVelCoM << 0,0,0;
        desVelL << 0,0,0,0,0,0;
        //desPosL = actualPosL + desVelL*timeStep;
        desPosL << actualPosL(0),actualPosL(1),0.1 + 0.07*std::sin(0.3*counter),actualPosL(3),actualPosL(4),actualPosL(5);
        //desPosL << actualPosL(0),actualPosL(1),0.2 ,actualPosL(3),actualPosL(4),actualPosL(5);

        desVelR << 0,0,0,0,0,0;

        desPosR = actualPosR + desVelR*timeStep;
        //desPosR << actualPosR(0),actualPosR(1),0.1,actualPosR(3),actualPosR(4),actualPosR(5);
        //desPosR << actualPosR(0),actualPosR(1),actualPosR(2),actualPosR(3),actualPosR(4),actualPosR(5);








/*
  // NEW
  // Experiment 5: circle with right foot. DONE

  	desVelR << 0,0,0,0,0,0;
  	desPosR << actualPosR(0)+ 0.4*std::cos(0.785*counter),actualPosR(1)+ 0.4*std::sin(0.785*counter),actualPosR(2),actualPosR(3),actualPosR(4),actualPosR(5);	// desired pose is simply the actual + desired velocity(=0) * time = actualPosL
  	//desPosR << actualPosR(0),actualPosR(1),actualPosR(2),0.05 + 0.05*std::cos(0.785*counter),0.05+ 0.05*std::sin(0.785*counter),actualPosR(5);	// desired pose is simply the actual + desired velocity(=0) * time = actualPosL


    desVelL << 0,0,0,0,0,0;
    desPosL = actualPosL + desVelL*timeStep;

    desVelCoM << 0,0,0;
    //desPosCoM = actualPosCoM + desVelCoM*timeStep;
    desPosCoM << 0.028,0, 0.35;

*/







 /* PROJECT

 	 	desPosCoM <<
      	desVelCoM <<
      	desVelL <<
      	desPosL =
      	desVelR <<
      	desPosR =


 */








  // Total desired velocities and poses(desired and actual) are grouped in 3 vectors
  desVelTotal << desVelL, desVelR, desVelCoM;
  actualPosTotal << actualPosL, actualPosR, actualPosCoM;
  desPosTotal << desPosL, desPosR, desPosCoM;







  // KINEMATIC INVERSION




  // NOTE: Class vectorXd generates a x-dimensional vector, e.g., vector(5) or vector = ...
  //       Class vector6d generates a 6-dimensional vector




  // VERY IMPORTANT NOTE: I think that 12 columns (joints) of these Jacobians are divided as:
  // Whole robot roll-pitch-yaw, vx, vy, vz (0 to 5) and "L/RHipYawPitch",L/RHipRoll,L/RHipPitch,L/RKneePitch,L/RAnklePitch,
  // L/RAnkleRoll (6 to 11)

  // Get Jacobian of swing foot
  Eigen::MatrixXd JLeftFootWithPose = mSwingFoot->getJacobian();
  //Eigen::MatrixXd JLeftFootWithPose = mRobot->getJacobian(mSwingFoot);
  //std::cout << JLeftFootWithPose << std::endl << std::endl;


  // Get Jacobian of support foot
  Eigen::MatrixXd JRightFootWithPose = mSupportFoot->getJacobian();
  //Eigen::MatrixXd JRightFootWithPose = mRobot->getJacobian(mSupportFoot);
  //std::cout << JRightFootWithPose << std::endl << std::endl;






  // VERY IMPORTANT NOTE: the method getJacobian() returns a Jacobian desfined as J = [Ja,Jl]
  // Since we are treating with magnitudes, including error vector, whose components start with linear ones and then angular ones
  // We have to 'invert' the Jacobian to finally obtain J = [Jl,Ja]
  Eigen::MatrixXd JAngularRight =  JRightFootWithPose.block<3,12>(0,0);
  Eigen::MatrixXd JLinearRight =  JRightFootWithPose.block<3,12>(3,0);
  JRightFootWithPose << JLinearRight,
		  	  	  	  	JAngularRight;

  Eigen::MatrixXd JAngularLeft =  JLeftFootWithPose.block<3,12>(0,0);
  Eigen::MatrixXd JLinearLeft =  JLeftFootWithPose.block<3,12>(3,0);
  JLeftFootWithPose << JLinearLeft,
  		  	  	  	  	JAngularLeft;


  // IMPORTANT NOTE: we need to change the sign of all elements of 3rd raw of Jl, since Jl is defined to obtain linear velocity
  // along z-axis in the downward direction, the opposite of our task, asking for positive z in upward direction
  for (int i=0;i<12;i++){
	  JRightFootWithPose(2,i) = -JRightFootWithPose(2,i);
  }

  for (int i=0;i<12;i++){
  	  JLeftFootWithPose(2,i) = -JLeftFootWithPose(2,i);
  }






  // Get linear Jacobian of CoM (concerning just linear velocity)
  // NOTE: CoM motions make use of all 30 joints
  Eigen::MatrixXd JCoMWithPose = mRobot->getCOMLinearJacobian();










  // Create Jacobian for left and right feet: 6 rows (3 for v, 3 for w), 6 column (6 legs joints) ???
  Eigen::MatrixXd JLeftFoot(6,6);
  Eigen::MatrixXd JRightFoot(6,6);

  // Create Jacobian for CoM: 3 rows (linear velocity v), 24 column (24 joints) ???
  Eigen::MatrixXd JCoM(3,24);








  // Initialize Jacobians: extract joint components
  JLeftFoot << JLeftFootWithPose.block<6,6>(0,6);
  JRightFoot << JRightFootWithPose.block<6,6>(0,6);
  JCoM << JCoMWithPose.block<3,24>(0,6);

  // VERY IMPORTANT
  // Create Jacobians of CoM such that its motion is due ONLY by Left foot or Right foot
  Eigen::MatrixXd JCoML(3,6);
  Eigen::MatrixXd JCoMR(3,6);

  JCoML = JCoMWithPose.block<3,6>(0,8);
  JCoMR = JCoMWithPose.block<3,6>(0,19);








  // Compute total Jacobian WITHOUT considering first 6 joints
  // NOTE: this method produces an error in computation. The solution is put some 0 in the whole Jacobian (see below)
 /* Eigen::MatrixXd JTotal(15,24);
  JTotal << Eigen::MatrixXd::Zero(6,2),  JLeftFoot,  Eigen::MatrixXd::Zero(6,16),
		  	Eigen::MatrixXd::Zero(6,13), JRightFoot, Eigen::MatrixXd::Zero(6,5) ,
			JCoM; */








  // Compute total Jacobian WITH considering also first 6 joints
 /* Eigen::MatrixXd JTotalWithPose(15,30);
  JTotalWithPose << JLeftFootWithPose.block<6,6>(0,0), Eigen::MatrixXd::Zero(6,2),  JLeftFoot,  Eigen::MatrixXd::Zero(6,16),
		  			JRightFootWithPose.block<6,6>(0,0), Eigen::MatrixXd::Zero(6,13), JRightFoot, Eigen::MatrixXd::Zero(6,5) ,
					JCoMWithPose;

  // VERY IMPORTANT
  // This stacked Jacobian makes both feet to be moved ONLY by leg's joints, while CoM is moved by ALL joints
  // In practice it YES works only for feet motion, NOT for CoM motion (?)
  JTotalWithPose << Eigen::MatrixXd::Zero(6,8),  JLeftFoot,  Eigen::MatrixXd::Zero(6,16),
  		  			Eigen::MatrixXd::Zero(6,19), JRightFoot, Eigen::MatrixXd::Zero(6,5) ,
  					JCoMWithPose;

  // This stacked Jacobian makes both feet to be moved ONLY by leg's joints, while CoM is moved ONLY by left foot
  // NOT work for CoM motion, YES for right foot motion, NOT left foot motion
  // Obviously: it's a question of used joints: to move CoM as well as left foot I use left leg joints for both,
  // so if I ask left foot to not move while I ask CoM to make hoola-hoop, system "crashes".
  // Instead, to move right foot I use right leg joints and I have not any problem
  JTotalWithPose << Eigen::MatrixXd::Zero(6,8),  JLeftFoot,  Eigen::MatrixXd::Zero(6,16),
    		  		Eigen::MatrixXd::Zero(6,19), JRightFoot, Eigen::MatrixXd::Zero(6,5) ,
					Eigen::MatrixXd::Zero(3,8),  JCoML,  Eigen::MatrixXd::Zero(3,16); */




/*
  Eigen::MatrixXd JTotalWithPose(3,30);

  // Jacobian for CoM moved by BOTH feet
  // Hoola-hoop: quite good
  JTotalWithPose << Eigen::MatrixXd::Zero(3,8),  JCoML,  Eigen::MatrixXd::Zero(3,5), JCoMR, Eigen::MatrixXd::Zero(3,5);

  // Jacobian for CoM moved ONLY by left foot
  // Hool-Hoop: quite good
  JTotalWithPose << Eigen::MatrixXd::Zero(3,8),  JCoML,  Eigen::MatrixXd::Zero(3,16); */





  // This kind of Jacobian is that we need for project
  // NOTE: It works better when swing foot is up, and support foot on ground, as our task requires for
  // NOTE: This is a clear example of how two different tasks can work together.

  // Support foot is left foot, swing foot is right foot
  Eigen::MatrixXd JTotalWithPose(9,30);
  JTotalWithPose <<   Eigen::MatrixXd::Zero(6,19), JRightFoot, Eigen::MatrixXd::Zero(6,5),
  		  	  	  	  Eigen::MatrixXd::Zero(3,8),  JCoML,  Eigen::MatrixXd::Zero(3,16);

  // Support foot is right foot, swing foot is left foot
  JTotalWithPose <<   Eigen::MatrixXd::Zero(6,8), JLeftFoot, Eigen::MatrixXd::Zero(6,16),
    		  	  	  Eigen::MatrixXd::Zero(3,19),  JCoMR,  Eigen::MatrixXd::Zero(3,5);








  //Eigen::MatrixXd JLPinv = (JLeftFoot.transpose())*(JLeftFoot*JLeftFoot.transpose()).inverse();
  //Eigen::MatrixXd JRPinv = (JRightFoot.transpose())*(JRightFoot*JRightFoot.transpose()).inverse();
  //Eigen::MatrixXd JCoMPinv = (JCoM.transpose())*(JCoM*JCoM.transpose()).inverse();






  // Pseudoinverse of Jacobians (total and total with pose)
  //Eigen::MatrixXd JTotalPinv = (JTotal.transpose())*(JTotal*JTotal.transpose()).inverse();
  Eigen::MatrixXd JTotalWithPosePinv = (JTotalWithPose.transpose())*(JTotalWithPose*JTotalWithPose.transpose()).inverse();






  //Eigen::VectorXd qLDot = JLPinv*desVelL;// + 0.5*desPos);
  //Eigen::VectorXd qRDot = JRPinv*desVelR;// + 0.5*desPos);
  //Eigen::VectorXd qCoMDot = JCoMPinv*desVelCoM;
  //  Eigen::VectorXd qDot = JTotalPinv*(desVelTotal + kInv*(desPosTotal - actualPosTotal));






  	Eigen::VectorXd desVelTotall(9);
    Eigen::VectorXd actualPosTotall(9);
    Eigen::VectorXd desPosTotall(9);

  	desVelTotall << desVelL, desVelCoM;
    actualPosTotall << actualPosL, actualPosCoM;
    desPosTotall << desPosL, desPosCoM; /*

   Eigen::VectorXd desVelTotall(3);
        Eigen::VectorXd actualPosTotall(3);
        Eigen::VectorXd desPosTotall(3);

      	desVelTotall << desVelCoM;
        actualPosTotall << actualPosCoM;
        desPosTotall << desPosCoM; */







  // Inverse kinematics control law to obtain joint velocities by imposing desired velocities and a feedback loop position error !!!
  // NOTE: if we remove the feedback loop the robot doesn't move, since desVelTotal is (0,0,0)!!!!
//  double kKin = 0.5;		// coefficient Kp
  double kKin = 0.5;		// coefficient Kp

  //Eigen::VectorXd qDot = JTotalPinv*(desVelTotal+ kKin*(desPosTotal - actualPosTotal));
  //Eigen::VectorXd qDot = JTotalWithPosePinv*(desVelTotal+ kKin*(desPosTotal - actualPosTotal));
  Eigen::VectorXd qDot = JTotalWithPosePinv*(desVelTotall + kKin*(desPosTotall - actualPosTotall));

  std::cout << "error is: " << desPosTotall - actualPosTotall << std::endl;





// Show error
//std::cout << "position error: " << desPosR - actualPosR << std::endl;






//  for (int i=0; i<24; ++i){
//	  mRobot->setVelocity(mRobot->getDof(joints[i])->getIndexInSkeleton(),0);
//  }
//
//  mRobot->setVelocity(mRobot->getDof("LAnklePitch")->getIndexInSkeleton(),0.1);










// I think I have to provide velocity to ALL joints; otherwise robot falls (I don't know why)
  for (int i=0; i<24; ++i){
  	  mRobot->setVelocity(mRobot->getDof(joints[i])->getIndexInSkeleton(),qDot(i+6));
  	  //mRobot->setVelocity(i+6,qDot(i+6));
  	 }

    for (int i=0; i<6; ++i){
  	  mRobot->setVelocity(i,qDot(i));
  	  //mRobot->setVelocity(i,0)
    }

}





/*	"HeadYaw",
    "HeadPitch",
    "LHipYawPitch",
    "LHipRoll",
    "LHipPitch",
    "LKneePitch",
    "LAnklePitch",
    "LAnkleRoll",
    "LShoulderPitch",
    "LShoulderRoll",
    "LElbowYaw",
    "LElbowRoll",
    "LWristYaw",
    "RHipYawPitch",
    "RHipRoll",
    "RHipPitch",
    "RKneePitch",
    "RAnklePitch",
    "RAnkleRoll",
    "RShoulderPitch",
    "RShoulderRoll",
    "RElbowYaw",
    "RElbowRoll",
    "RWristYaw"*/


//==============================================================================
// Simply return the robot

dart::dynamics::SkeletonPtr Controller::getRobot() const
{
  return mRobot;
}

//==============================================================================
// A puntatore of MPCSolver class

mpcSolver::MPCSolver* Controller::getSolver() const
{
  return mSolver;
}

//==============================================================================
// Return support foot or swing foot

dart::dynamics::BodyNode* Controller::getSupportFoot() const
{
	/*if (supportFoot)
		return mRobot->getBodyNode("l_wrist");
	else
		return mRobot->getBodyNode("r_wrist"); */

	if (supportFoot)
			return mRobot->getBodyNode("l_sole");
		else
			return mRobot->getBodyNode("r_sole");
}

//==============================================================================
// Get CoM

Eigen::VectorXd Controller::getEndEffector() const
{
	return mRobot->getCOM();
}

//==============================================================================
// Function that computes and returns swingfoot pose and velocity (linear and angular) BASING ON temporization (ind,indInitial)
// IMPORTANT NOTE: Swing foot changes at each walk step, so swing foot trajectory concerns trajectory of ENTIRE FOOT PATH
// The code in quite difficult to understand (?)


Eigen::MatrixXd Controller::getOmnidirectionalSwingFootTrajectoryMPC(Eigen::VectorXd optimalFootstep, Eigen::VectorXd swingFootInitialPosition, int ind, double stepHeight, int S, int D, int indInitial, double delta){
																	//MPC(mSolver->getOptimalFootsteps(), changeRF6*MPCSupportFootPosition, counter, 0.01, 10, 5, indInitial, 0.02);

		// 'ind' is: counter

		// 'indInitial' is: =counter when timestep is a multiple of 15, <counter when timestep is not a multiple of 15

		// 'S' is set to: 10 (we can change it)

		// 'optimalFootstep' is computed by MPCSolver by means getOptimalFootsteps() function

		// swingFootInitialPosition is computed as changeRF6(?)* +/- distance of feet.
	    //I THINK that change RF6 is a transformation matrix from WRF to RF attached to support foot (in this way MPCSupportFootPosition assumes sense)


        double m_exp=1;
        double scale = 0.7694;
        double tf = (indInitial + S)*delta;
        double k=1;
        double time=ind*delta;
        double ti=indInitial*delta;

        // Create vectors for swing foot pose (6D) and velocity (6D)
        Eigen::VectorXd swingFootPosition(6);
        Eigen::VectorXd swingFootVelocity(6);


        // No possible in the code of update(...)
        if (ind<indInitial){

        	//
        	swingFootPosition(0) = swingFootInitialPosition(0);
            swingFootPosition(1) = swingFootInitialPosition(1);
            swingFootPosition(2) = 0*swingFootInitialPosition(2);
            swingFootPosition(3) = 0*swingFootInitialPosition(3);
            swingFootPosition(4) = 0*swingFootInitialPosition(4);
            swingFootPosition(5) = 0*swingFootInitialPosition(5);

            // No velocity (linear and angular) of swingfoot
            swingFootVelocity(0) = 0;
            swingFootVelocity(1) = 0;
            swingFootVelocity(2) = 0;
            swingFootVelocity(3) = 0;
            swingFootVelocity(4) = 0;
            swingFootVelocity(5) = 0;
        }

        else if (ind>=indInitial+S){
            swingFootPosition(0) = optimalFootstep(0);
            swingFootPosition(1) = optimalFootstep(1);
            swingFootPosition(2) = 0*swingFootInitialPosition(2);
            swingFootPosition(3) = 0*swingFootInitialPosition(3);
            swingFootPosition(4) = 0*swingFootInitialPosition(4);
            swingFootPosition(5) = optimalFootstep(2);
            swingFootVelocity(0) = 0;
            swingFootVelocity(1) = 0;
            swingFootVelocity(2) = 0;
            swingFootVelocity(3) = 0;
            swingFootVelocity(4) = 0;
            swingFootVelocity(5) = 0;
        }

        else{
            swingFootPosition(0) = swingFootInitialPosition(0) + (delta)/(tf-time) * k *(optimalFootstep(0)-swingFootInitialPosition(0));
            swingFootPosition(1) = swingFootInitialPosition(1) + (delta)/(tf-time) * k *(optimalFootstep(1)-swingFootInitialPosition(1));
            swingFootPosition(2) = ((sin(M_PI-(time-ti)/(tf-ti)*M_PI)*pow(sin(0.5*(M_PI-(time-ti)/(tf-ti)*M_PI)),m_exp)/scale))*stepHeight;
            swingFootPosition(3) = swingFootInitialPosition(3);
            swingFootPosition(4) = swingFootInitialPosition(4);
            swingFootPosition(5) = swingFootInitialPosition(5) + (delta)/(tf-time) * k*(optimalFootstep(2)-swingFootInitialPosition(5));

            swingFootVelocity(0) = k*(optimalFootstep(0)-swingFootInitialPosition(0))/(tf-time);
            swingFootVelocity(1) = k*(optimalFootstep(1)-swingFootInitialPosition(1))/(tf-time);
            swingFootVelocity(2) = (M_PI/(tf-ti))*(stepHeight/scale)*
                    (-cos(M_PI-(time-ti)/(tf-ti)*M_PI)*pow(sin(0.5*(M_PI-(time-ti)/(tf-ti)*M_PI)),m_exp)
                     -sin(M_PI-(time-ti)/(tf-ti)*M_PI)*0.5*m_exp*pow(sin(0.5*(M_PI-(time-ti)/(tf-ti)*M_PI)),(m_exp-1))*cos(0.5*(M_PI-(time-ti)/(tf-ti)*M_PI)));

            swingFootVelocity(3) = 0;
            swingFootVelocity(4) = 0;
            swingFootVelocity(5) = k*(optimalFootstep(2)-swingFootInitialPosition(5)/(tf-time));
        }

        Eigen::MatrixXd task(6,2);


        // Function returns the pose and velocity of swingFoot
        task << swingFootPosition, swingFootVelocity;
        return task;


    }

//==============================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/)
{
}

