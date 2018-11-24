#ifndef _PTU_H_
#define _PTU_H_

/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2014 Adept Technology

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact
Adept MobileRobots for information about a commercial version of ARIA at
robots@mobilerobots.com or
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
#include "Aria.h"

/** @example dpptuExample.cpp  Shows how to control the Directed Perception
 * pan-tilt unit using ArDPPTU class directly.
 *
 * This program lets you use the keyboard to control the DPPTU.  It uses the same acceleration and slew rates for the pan and tilt axes.
 *
 * It is also possible to specify the type of PTU in program configuration
 * (in the ARIA robot parameter files or program command-line arguments)
 * instead. For an example of that, see cameraPTZExample.cpp instead.

Commands:
_________________

UP,DOWN  -- tilt up/down by one positional increment
LEFT,RIGHT  --  pan left/right by one positional increment
SPACE  -- perform reset calibration
I  -- initialize PTU to default settings
<,>  -- increase/decrease the posIncrement by 0.5 degree
+,-  -- increase/decrease the speed by 1 degree/sec
A  -- awaits the completion of last issued positional command
R  -- change pan/tilt movements to relative or absolute movements
Z  -- move pan and tilt axes to zero
1  -- move to stored position 1 (-90, 0)
2  -- move to stored position 2 (90, 0)
3  -- move to stored position 3 (0, -45)
4  -- move to stored position 4 (0, 30)
M  -- Enter or Exit monitor (continuous scan) mode
H  -- Halt all motion
S  -- print current variable values
ESC -- quit
*/

// If defined, use this computer serial port. If not defined, use first robot
// aux. serial port.  Most robots have the DPPTU on COM2 if on Linux and COM4 on
// Windows, if not equipped with other accessories which might require those
// ports (e.g. GPS or Laser).
#define SERIAL_PORT ArUtil::COM4


// Determines type of DPPTU to set internal conversion factors. See enum of
// types in ArDPPTU class for possible values.
//#define PTU_TYPE ArDPPTU::PANTILT_PTUD46

// by how much the < and >  keys change the position command increment in this
// example
#define POS_INC_ADJUSTMENT 1

/*
  This class is the core of this demo, it adds itself to the robot given
  as a user task, and contains key handler callbacks to control the PTU.
*/
class PTU
{
public:
  // constructor
  PTU(ArRobot *robot);
  ~PTU(void);

  void up(void);
  void down(void);
  void left(void);
  void right(void);
  void space(void);
  void i(void);
  void plus(void);
  void minus(void);
  void greater(void);
  void less(void);
  void question(void);
  void status(void);
  void a(void);
  void m(void);
  void h(void);
  void r(void);
  void gotoPos(double p, double t);
  float getPan(){
      return myPTU.getPan();
  }

  bool getHavePTU();

  // the callback function
  void drive(void);

protected:
  int myPanValPTU;
  int myTiltValPTU;

  int myDesiredPanPos;
  int myDesiredTiltPos;
  int mySlew;
  int myPosIncrement;
  int mySlewIncrement;

  int POS_INCREMENT_ADJUSTMENT;

  bool myMonitor;
  bool myReset;
  bool myInit;
  bool myAbsolute;
  bool havePTU;


  ArFunctorC<PTU> myPlusCB;
  ArFunctorC<PTU> myMinusCB;
  ArFunctorC<PTU> myGreaterCB;
  ArFunctorC<PTU> myLessCB;


  // the PTU
  ArDPPTU myPTU;


  // whether the PTU has been inited or not
  bool myPTUInited;
  // pointer to the robot
  ArRobot *myRobot;
  // callback for the drive function
  ArFunctorC<PTU> myDriveCB;

  ArSerialConnection *mySerialConnection;
};

#endif


// int main(int argc, char **argv)
// {
//
//
//
//
//   robot.runAsync(true);
//
//   // an object for keyboard control, class defined above, this also adds itself as a user task
//   KeyPTU ptu(&robot);
//
//
//   // turn off the sonar
//   robot.comInt(ArCommands::SONAR, 0);
//
//   printf("Press '?' for available commands\r\n");
//
//   // run, if we lose connection to the robot, exit
//   robot.waitForRunExit();
//
//   Aria::exit(0);
// }
