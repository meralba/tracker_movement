#include "PTU.h"
/*
  Constructor, sets the robot pointer, and some initial values, also note the
  use of constructor chaining on myPTU and myDriveCB.
*/
PTU::PTU(ArRobot *robot) :
  myPlusCB(this, &PTU::plus),
  myMinusCB(this, &PTU::minus),
  myGreaterCB(this, &PTU::greater),
  myLessCB(this, &PTU::less),
  myPTU(robot),
  myDriveCB(this, &PTU::drive),
  mySerialConnection(NULL)
{
#ifdef SERIAL_PORT
  mySerialConnection = new ArSerialConnection;
  ArLog::log(ArLog::Normal, "dpptuExample: connecting to DPPTU over computer serial port %s.", SERIAL_PORT);
  if(mySerialConnection->open(SERIAL_PORT) != 0)
  {
	ArLog::log(ArLog::Terse, "dpptuExample: Error: Could not open computer serial port %s for DPPTU!", SERIAL_PORT);
    //Aria::exit(5);
    havePTU = false;
  }
  havePTU = true;
  myPTU.setDeviceConnection(mySerialConnection);
#endif


  myRobot = robot;
  myRobot->addSensorInterpTask("PTU", 50, &myDriveCB);

  // initialize some variables
  myReset = false;
  myInit = true;
  myDesiredPanPos = 0;
  myDesiredTiltPos = 0;
  myPosIncrement = 1;
  mySlewIncrement = 1;
  myPTUInited = false;
  myMonitor = false;

}

PTU::~PTU()
{
  if(mySerialConnection)
  {
    myPTU.setDeviceConnection(NULL);
    delete mySerialConnection;
  }
}


void PTU::plus(void)
{
  mySlew += mySlewIncrement;

  if (mySlew > myPTU.getMaxPanSlew())
    mySlew = myPTU.getMaxPanSlew();

  status();
}

void PTU::minus(void)
{
  mySlew -= mySlewIncrement;

  if (mySlew < myPTU.getMinPanSlew())
    mySlew = myPTU.getMinPanSlew();

  status();
}

void PTU::greater(void)
{
  myPosIncrement += POS_INCREMENT_ADJUSTMENT;

  if (myPosIncrement > myPTU.getMaxPosPan())
    myPosIncrement = myPTU.getMaxPosPan();

  status();
}

void PTU::less(void)
{
  myPosIncrement -= POS_INCREMENT_ADJUSTMENT;

  if (myPosIncrement < 0)
    myPosIncrement = 0;

  status();
}


void PTU::gotoPos(double p, double t)
{
  myDesiredPanPos = p;
  myDesiredTiltPos = t;
  status();
}

bool PTU::getHavePTU(){
    return havePTU;
}


void PTU::status(void)
{
  ArLog::log(ArLog::Normal, "\r\nStatus:\r\n_________________\r\n");
  ArLog::log(ArLog::Normal, "Last Pan Command      = %.1f deg", myPTU.getLastPanRequest());
  ArLog::log(ArLog::Normal, "Last Tilt Command      = %.1f deg", myPTU.getLastTiltRequest());
  ArLog::log(ArLog::Normal, "Current Pan Position  = %.1f deg", myPTU.getPan());
  ArLog::log(ArLog::Normal, "Current Tilt Position = %.1f deg", myPTU.getTilt());
  ArLog::log(ArLog::Normal, "Pan Slew Rate         = %d deg/sec", myPTU.getPanSlew());
  ArLog::log(ArLog::Normal, "Tilt Slew Rate        = %d deg/sec", myPTU.getTiltSlew());
  ArLog::log(ArLog::Normal, "Position Increment    = %d deg", myPosIncrement);
  if (myAbsolute)
    ArLog::log(ArLog::Normal, "Positional-movements using absolute commands");
  else
    ArLog::log(ArLog::Normal, "Positional-movements using relative commands");
  ArLog::log(ArLog::Normal, "\r\n");
}



// the important function
void PTU::drive(void)
{

  // if the PTU isn't initialized, initialize it here... it has to be
  // done here instead of above because it needs to be done when the
  // robot is connected
  if (!myPTUInited && myRobot->isConnected())
  {
    ArLog::log(ArLog::Normal, "Initializing ArDPPTU...");
    myPTU.init();
    ArLog::log(ArLog::Normal, "Resetting PTU and performing self-calibration...");
    myPTU.resetPan();
    myPTU.awaitExec(); // DPPTU will wait for self-calibration to end before executing the following commands (though they will still be sent)
    mySlew = myPTU.getPanSlew(); //uses only pan slew rate
    myPTU.awaitExec();
    myPTUInited = true;
    myInit = false;
    myAbsolute = true;
  }

  if (myInit == true)  // User hit initialization
  {
    ArLog::log(ArLog::Normal, "Initializing PTU...");
    myPTU.init();
    myInit = false;
    myDesiredPanPos = myPTU.getPan();
    myDesiredTiltPos = myPTU.getTilt();
    mySlew = myPTU.getPanSlew(); //uses only pan slew rate
    myReset = false;
  }

  if (myReset == true) // User hit reset key
  {
    ArLog::log(ArLog::Normal, "Resetting PTU and performing self-calibration...");
    myPTU.resetCalib();
    myPTU.awaitExec();
    myDesiredPanPos = myPTU.getPan();
    myDesiredTiltPos = myPTU.getTilt();
    myReset = false;
  }
  else   // User did nothing, or hit a key that changed myDesiredPanPos, myDesiredTiltPos, or mySlew (so request PTU to move if those changed since last request)
  {

    // Some PTUs can determine their current position (with encoders, etc) and return that.
    // canGetRealPanTilt() will return true in this case, and getPan() and
    // getTilt() will return those received values.  Otherwise, getPan() and
    // getTilt() return the last commanded values.  getLastPanRequest() and
    // getLastTiltRequest() will always return the last commanded values sent by
    // ArDPPTU (so in the case that canGetRealPanTilt() is false, getPan() and
    // getTilt() return the same pair of values as getLastPanRequest() and
    // getLastTiltRequest().  ArDPPTU::canGetRealPanTilt() is initialally false,
    // but once the first set of pan and tilt positions is read back from the
    // PTU device, it becomes true.
    if(myPTU.canGetRealPanTilt())
      printf("Position (%.1f deg, %.1f deg)     [Incr. %d deg]     Press ? for help  \r", myPTU.getPan(), myPTU.getTilt(), myPosIncrement);
    else
      printf("Requested (%.1f deg, %.1f deg)     [Incr. %d deg]     Press ? for help  \r", myPTU.getPan(), myPTU.getTilt(), myPosIncrement);

    if (myDesiredPanPos != myPTU.getLastPanRequest())
    {
      if (myAbsolute)
      	myPTU.pan(myDesiredPanPos);
      else
        myPTU.panRel(myDesiredPanPos - myPTU.getPan());
    }

    if (myDesiredTiltPos != myPTU.getLastTiltRequest())
    {
      if (myAbsolute)
        myPTU.tilt(myDesiredTiltPos);
      else
        myPTU.tiltRel(myDesiredTiltPos - myPTU.getTilt());
    }

    if (mySlew != myPTU.getPanSlew())
    {
      myPTU.panSlew(mySlew);
      myPTU.tiltSlew(mySlew);
    }

  }

}
