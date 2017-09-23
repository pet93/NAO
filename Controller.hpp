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

// I HAVE NOT TO TOUCH THIS CODE SINCE IT CALLS "Controller.cpp WHERE ALL
// METHODS ARE IMPLEMENTED


#ifndef EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
#define EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_

#include <Eigen/Eigen>

#include "dart/dart.hpp"
#include "MPCSolver.hpp"

/// \brief Operational space controller for 6-dof manipulator
class Controller
{
public:

  // Constructor (the body of this constructor is implemented into Controller.cpp)
  Controller(dart::dynamics::SkeletonPtr _robot,
             dart::dynamics::BodyNode* _supportFoot,
			 dart::dynamics::BodyNode* _swingFoot,
			 dart::simulation::WorldPtr _world);

  // Brief Destructor (it acts when program ends)
  virtual ~Controller();

  // Update (1)
  void update(Eigen::Vector3d _targetPosition);

  // Get robot (2)
  dart::dynamics::SkeletonPtr getRobot() const;

  // Get solver (3)
  mpcSolver::MPCSolver* getSolver() const;

  // Get support foot (4)
  dart::dynamics::BodyNode* getSupportFoot() const;

  // Get end effector of the robot (5)
  Eigen::VectorXd getEndEffector() const;

  // Get Omnidirectional Swing Foot Trajectory MPC (6)
  Eigen::MatrixXd getOmnidirectionalSwingFootTrajectoryMPC(Eigen::VectorXd optimalFootstep, Eigen::VectorXd swingFootInitialPosition, int ind, double stepHeight, int S, int D, int indInitial, double delta);

  // Keyboard control (7)
  virtual void keyboard(unsigned char _key, int _x, int _y);



private: 			// Can use it only inside the class
  // mRobot object
  dart::dynamics::SkeletonPtr mRobot;

  // Punctator "supportFoot" of the robot, belonging to BodyNode class
  dart::dynamics::BodyNode* mSupportFoot;

  // Punctator "swingFoot" of the robot, belonging to BodyNode class
  dart::dynamics::BodyNode* mSwingFoot;

  // mWorld object
  dart::simulation::WorldPtr mWorld;

  // Punctator "mSolver" of the robot, belonging to MPCSolver class
  mpcSolver::MPCSolver* mSolver;

  // Variable for ...
  int indInitial;

  // Boolean variable for support foot
  bool supportFoot;

  // Vectors referring to initial pose of left and right feet
  Eigen::Vector6d initialPosL;
  Eigen::Vector6d initialPosR;
};

#endif  // EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
