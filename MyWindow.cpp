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

#include "MyWindow.hpp"

#include <iostream>

//==============================================================================
// Constructor

MyWindow::MyWindow(Controller* _controller)
  : SimWindow(),							// By this command the class MyWindow inherits methods of class SimWindow
    mController(_controller),
    mCircleTask(false)						// circle task (?)
{
  assert(_controller != nullptr);

  // Set the initial target positon to the initial position of the end effector (CoM)
  mTargetPosition = mController->getEndEffector();
}

//==============================================================================
MyWindow::~MyWindow()
{
}

//==============================================================================
void MyWindow::timeStepping()	// Called by an unknown method due to pressing spacebar
 {
  if (mCircleTask)
  {
    static double time = 0.0;
    const double dt = 0.0005;
    const double radius = 0.6;
    Eigen::Vector3d center = Eigen::Vector3d(0.0, 0.1, 0.0);

    mTargetPosition = mController->getEndEffector();
  //mTargetPosition[0] = radius * std::sin(time);
  //mTargetPosition[1] = 0.25 * radius * std::sin(time);
  //mTargetPosition[2] = radius * std::cos(time);

    time += dt;
  }
  //mTargetPosition = mController->getEndEffector();
  // Update the controller and apply control force to the robot

  mController->update(mTargetPosition);		// the sense of the argument ?  	->Controller.cpp

  // Step forward the simulation
  mWorld->step();
}

//==============================================================================
// Function of initialization of the world
// This function is updated at each step

void MyWindow::drawWorld() const
{
  // Draw the target position
  if (mRI)
  {

	Eigen::Matrix3d changeRF;
	changeRF << 1,0,0,
				0,1,0,
				0,0,1;


    // Draw CoM: point attached to CoM (standard is red)
	mRI->pushMatrix();
    mRI->translate(mController->getRobot()->getCOM());
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    //mRI->setPenColor(Eigen::Vector3d(0.3, 0.2, 0.2));
    mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
    mRI->popMatrix();

    // Draw optimal CoM
    mRI->pushMatrix();
    mRI->translate(mController->getSupportFoot()->getCOM() + changeRF.transpose()*(mController->getSolver()->getOptimalCoMPosition()));
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
    mRI->popMatrix();

    // Draw foot
    mRI->pushMatrix();
    mRI->translate(mController->getSupportFoot()->getCOM());
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
    mRI->popMatrix();

    // Draw predicted footstep
    mRI->pushMatrix();
    mRI->translate(mController->getSupportFoot()->getCOM() +
    		 	   changeRF.transpose()*Eigen::Vector3d(mController->getSolver()->getOptimalFootsteps()(0), mController->getSolver()->getOptimalFootsteps()(1), 0));
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.8));
    mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
    mRI->popMatrix();

    // Draw ZMP prediction
	Eigen::MatrixXd ZMPPredictionRel = mController->getSolver()->getOptimalZMPPrediction();
	for(int i; i<30; ++i){
		mRI->pushMatrix();
		Eigen::Vector3d ZMPPredictionPoint = Eigen::Vector3d(ZMPPredictionRel(0,i), 0.0, -ZMPPredictionRel(1,i)) + mController->getSupportFoot()->getCOM();
		mRI->translate(ZMPPredictionPoint);
		mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
		mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
		mRI->popMatrix();
	}

	// Draw a point

	mRI->pushMatrix();
	mRI->translate(Eigen::Vector3d(0.3, 0.0, 0.37));
	mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
	mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
	mRI->popMatrix();



	// Draw a point: this point is visible on simulator and we can use it to test robot performances (at level of Jacobian)
	mRI->pushMatrix();

	//mRI->translate(Eigen::Vector3d(0.0, 1.0, 0.0));
	mRI->translate(Eigen::Vector3d(0.073, 0.05,0.06));

	mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
	//mRI->setPenColor(Eigen::Vector3d(0.7, 0.1, 0.2));

	mRI->drawEllipsoid(Eigen::Vector3d(0.02, 0.02, 0.02));
	mRI->popMatrix();



//    mRI->drawCylinder(0.001,1);
//    mRI->popMatrix();
  }

  // Draw world
  SimWindow::drawWorld();
}

//==============================================================================
// This function manages the pressure of a button

void MyWindow::keyboard(unsigned char _key, int _x, int _y)			// what are 'x' and 'y' ??
{
  double incremental = 0.01;

  switch (_key)
  {
    case 'c':  // print debug information
      if (mCircleTask)
      {
        std::cout << "Circle task [off]." << std::endl;
        mCircleTask = false;
      }
      else
      {
        std::cout << "Circle task [on]." << std::endl;
        mCircleTask = true;
      }
      break;
    case 'q':
      mTargetPosition[0] -= incremental;
      break;
    case 'w':
      mTargetPosition[0] += incremental;
      break;
    case 'a':
      mTargetPosition[1] -= incremental;
      break;
    case 's':
      mTargetPosition[1] += incremental;
      break;
    case 'z':
      mTargetPosition[2] -= incremental;
      break;
    case 'x':
      mTargetPosition[2] += incremental;
      break;
    default:
      // Default keyboard control (SimWindow class)
      SimWindow::keyboard(_key, _x, _y);
      break;
  }

  // Keyboard control for Controller -> Controller.cpp -> Controller::keyboard
  mController->keyboard(_key, _x, _y);

  glutPostRedisplay();
}

