// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Supervisor.hpp>
#include <iostream>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

const int TILES_PER_METER = 4;

#pragma region global variables
Robot* robot;
Motor* left_motor;
Motor* right_motor;
#pragma endregion


int main(int argc, char **argv)
{
  // create the Robot instance.
  robot = new Robot();
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  
  //WbDeviceTag left_motor = robot->getDeviceTagFromName("left wheel motor");
  //WbDeviceTag right_motor = robot->getDeviceTagFromName("right wheel motor");
  left_motor = robot->getMotor("left wheel motor");
  right_motor = robot->getMotor("right wheel motor");

  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1)
  {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.
      left_motor->setVelocity(10.0);
      right_motor->setVelocity(-10.0);
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  CleanUp();
  return 0;
}

void CleanUp()
{
    delete robot;
    delete left_motor;
    delete right_motor;
}
