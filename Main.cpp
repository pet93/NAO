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

// Inclusions
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/dynamics/SmartPointer.hpp>

#include "MyWindow.hpp"
#include <limits.h>
#include <stdlib.h>


// Set initial parameters about roll-pitch-yaw orientations (OUT OF MAIN)
void setInitialConfiguration(dart::dynamics::SkeletonPtr naoRobot)
{
	naoRobot->setPosition(5,0.35); // initial height
	//naoRobot->setPosition(5,0.6); // initial height

	naoRobot->setPosition(naoRobot->getDof("HeadYaw")->getIndexInSkeleton(), 0 );
	naoRobot->setPosition(naoRobot->getDof("HeadPitch")->getIndexInSkeleton(), 0 );
	naoRobot->setPosition(naoRobot->getDof("RShoulderPitch")->getIndexInSkeleton(), 80*M_PI/180 );
	naoRobot->setPosition(naoRobot->getDof("RShoulderRoll")->getIndexInSkeleton(), -10*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("RElbowYaw")->getIndexInSkeleton(), 50*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("RElbowRoll")->getIndexInSkeleton(), 2*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("RWristYaw")->getIndexInSkeleton(), 0 );
	naoRobot->setPosition(naoRobot->getDof("LShoulderPitch")->getIndexInSkeleton(), 80*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("LShoulderRoll")->getIndexInSkeleton(), 10*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("LElbowYaw")->getIndexInSkeleton(), -50*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("LElbowRoll")->getIndexInSkeleton(), -2*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("LWristYaw")->getIndexInSkeleton(), 0 );

	naoRobot->setPosition(naoRobot->getDof("RHipYawPitch")->getIndexInSkeleton(), 0 );
	naoRobot->setPosition(naoRobot->getDof("RHipRoll")->getIndexInSkeleton(), 0 );
	naoRobot->setPosition(naoRobot->getDof("RHipPitch")->getIndexInSkeleton(), -20*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("RKneePitch")->getIndexInSkeleton(), 34*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("RAnklePitch")->getIndexInSkeleton(), -14*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("RAnkleRoll")->getIndexInSkeleton(), 0 );

	naoRobot->setPosition(naoRobot->getDof("LHipYawPitch")->getIndexInSkeleton(), 0 );
	naoRobot->setPosition(naoRobot->getDof("LHipRoll")->getIndexInSkeleton(), 0 );
	naoRobot->setPosition(naoRobot->getDof("LHipPitch")->getIndexInSkeleton(), -20*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("LKneePitch")->getIndexInSkeleton(), 34*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("LAnklePitch")->getIndexInSkeleton(), -14*M_PI/180.0 );
	naoRobot->setPosition(naoRobot->getDof("LAnkleRoll")->getIndexInSkeleton(), 0 );
}

int main(int argc, char* argv[])
{
	// Array of names of main robot components' orientation
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


  // Create and initialize the world
  dart::simulation::WorldPtr world(new dart::simulation::World);
  assert(world != nullptr);
  char *full_path1 = realpath("ground.urdf", NULL);
  char *full_path2 = realpath("nao.urdf", NULL);
  std::string path_ground(full_path1);
  std::string path_nao(full_path2);
  std::cout << full_path1 << std::endl;
  std::cout << full_path2 << std::endl;

  // Load skeletons
  dart::utils::DartLoader dl;
  dart::dynamics::SkeletonPtr ground   = dl.parseSkeleton(path_ground);
  dart::dynamics::SkeletonPtr naoRobot = dl.parseSkeleton(path_nao);





// In SERVO mode still velocity is input, output is joint acceleration. The constraint solver will try to track the desired velocity within the joint force limit
// In practice we are simulating a real servo motor
  for (int i=0; i<24; ++i){
	  naoRobot->getJoint(joints[i])->setActuatorType(dart::dynamics::detail::SERVO);


   }

  //for (int i=0; i<24; ++i){
  //	  naoRobot->getJoint(joints[i])->setActuatorType(dart::dynamics::detail::VELOCITY);
  //}

//  for (int i=0; i<6; ++i){
//  	  naoRobot->getJoint(i)->setActuatorType(dart::dynamics::detail::FORCE);
//  }




  dart::constraint::ContactConstraint* servo;
  //servo->setConstraintForceMixing(1);
  //std::cout << servo->getConstraintForceMixing() << std::endl;



  double sufficient_force   = 1e+5;
  double posUpperLimit = 286.14;
  double posLowerLimit = -286.14;

 for (int i=0; i<24; i++) {
	  naoRobot->getJoint(joints[i])->setForceUpperLimit(0, sufficient_force);
	  naoRobot->getJoint(joints[i])->setForceLowerLimit(0, -sufficient_force);

	  naoRobot->getJoint(joints[i])->setDampingCoefficient(0, 0.0);
	  naoRobot->getJoint(joints[i])->setSpringStiffness(0, 0.0);
	  naoRobot->getJoint(joints[i])->setCoulombFriction(0, 0.0);
	  naoRobot->getJoint(joints[i])->setPositionLimitEnforced(true);

	  naoRobot->getJoint(joints[i])->setPositionUpperLimit(0, posUpperLimit);
	  naoRobot->getJoint(joints[i])->setPositionLowerLimit(0, posLowerLimit);

	  naoRobot->getJoint(joints[i])->setVelocityUpperLimit(0, posUpperLimit);
	  naoRobot->getJoint(joints[i])->setVelocityLowerLimit(0, posLowerLimit);

	  naoRobot->getJoint(joints[i])->setAccelerationUpperLimit(0, posUpperLimit);
	  naoRobot->getJoint(joints[i])->setAccelerationLowerLimit(0, posLowerLimit);

  }



  for (int i=0; i<24; ++i) {
  	  std::cout << naoRobot->getJoint(joints[i])->getAccelerationUpperLimit(0) << std::endl;
  }











  // Use the function setInitialConfiguration explained on top
  setInitialConfiguration(naoRobot);

  world->addSkeleton(ground);
  world->addSkeleton(naoRobot);

  //dart::dynamics::Shape line(dart::dynamics::Shape::LINE_SEGMENT);

  // Create and initialize the world: gravity
  Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  //Eigen::Vector3d gravity(0.0, 0.0, 0.0);

  world->setGravity(gravity);

  // And timestep
  world->setTimeStep(2.0/100);







  // Create puntatori supportFoot and swingFoot of BodyNode class
  //dart::dynamics::BodyNode* supportFoot = naoRobot->getBodyNode("l_wrist");
  //dart::dynamics::BodyNode* swingFoot = naoRobot->getBodyNode("r_wrist");

  dart::dynamics::BodyNode* supportFoot = naoRobot->getBodyNode("l_sole");
  dart::dynamics::BodyNode* swingFoot = naoRobot->getBodyNode("r_sole");







  // Call MyWindow constructor (MyWindow.cpp) passing as argument Controller constructor (Controller.cpp)
  // The order of execution is: Controller, MyWindow
  MyWindow window(new Controller(naoRobot, supportFoot, swingFoot, world)); // -> Controller.cpp & -> MyWindow.cpp
  window.setWorld(world);









  // Window of visualization
        glutInit(&argc, argv);
        window.initWindow(640, 480, "NAO Simulation");
        glutMainLoop();

    return 0;
  }






  // NOTE: Function with 'const' after declaration are executed without an explicit invocation but
  //       by means a chain of invocations starting from constructors, that are the only methods invoked by main.cpp

  // NOTE: By pressing spacebar we access to SimWindow and we start the simulation by invoking:
  //	   timeStepping() in MyWindow.cpp and consequently update(target) in Controller.cpp.
  //       How this can happens is hidden by the system

  // NOTE: Coordinates position:
  //	  Z(2) axis points toward up
  //      Y(1) axis points toward 'left'
  //	  X(0) axis points toward 'dritto'
  //	  0 : parallel to initial feet direction (x)
  //	  1 : parallel to line joining feet (y)
  //	  2 : height (z)
  //

  // NOTE: The interaction between Controller and MPCSolver is in:
  //	   Controller constructor: MPCSolver constructor
  //	   Update function into Controller.cpp: solve, getOptimalCOMPosition, getOptimalCOMVelocity
  //	   getOmniDirectionalSwingFoot function into Controller.cpp: getOptimalFootsteps

  // NOTE: Ways to generate a walk:
  //	  	  1) getOmniDirectionalSwingFootTrajectoryMPC for feet (changeRF6 can change things?)
  //		  2) Internal commands in Update function
  //		  3) MPCSolver for CoM/ZMP trajectories

  // NOTE: MPCSolver to get optimal trajectories of CoM-ZMP (?)
  //       - generate optimal CoM/ZMP trajectory, i.e., in such a way that ZMP is ALWAYS inside SP
  //	   - if ZMP trajectory is computed, convert it to CoM trajectory
  //	   - 'match' CoM trajectory with feet trajectory
