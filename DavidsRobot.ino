#include <PID_v1.h>
#include <elapsedMillis.h>
#include <SPI.h>  
#include <Pixy.h>
#include <string.h>



Pixy pixy;   // This is the main Pixy object 

int range = (0.25 * PIXY_MAX_X / 2);
int center = PIXY_MAX_X / 2;
int centerLeft  = center - range;  
int centerRight = center + range;
int turnSpeed = 130;

int logSample = 2;
int frameSample = 2;

int leftTurnCnt;
int rightTurnCnt;
int centeredCnt;
int noTargetCnt;
int iterationCnt = 1;

// These variables are used to keep track of the last command sent to the controller
int pLeftCmd = 0; 
int pRightCmd = 0;

// This structure defines and describes a motor connected through a 
//  L298N Dual H-Bridge Motor Controller. The motor controller requires
//  three pins to control speed and direction plus a common ground (not
//  included in structure).  Functions for initializing and setting
//  pins are provided as well.
struct motor
{
  // define the pin definitions
  int dir1Pin;
  int dir2Pin;
  int speedPin;
  int encoderPin;
  char name[12];

  // create a command 
  int generateCommand(bool motorOn, bool motorForward, int speed)
  {
	  if ((speed < 0) || (speed > 255)) return -100;
	  int iActive  = (motorOn?1:0);
	  int iForward = (motorForward?1:0) << 1;
	  int iSpeed = speed << 2;

	  return (iActive ^ iForward ^ iSpeed);
  }

  // Define the commandMotor function which takes a command for a motor and sets the appropriate
  // values for each pin.
  void commandMotor(int command)
  {
    if (getMotorOn(command))
    {
      int speed = getMotorSpeed(command);
      if (speed < 0) { 
        speed = 0; 
      }
      analogWrite(speedPin, speed);
      digitalWrite(dir1Pin, getMotorForward(command)?LOW:HIGH);
      digitalWrite(dir2Pin, getMotorForward(command)?HIGH:LOW);
    }
    else 
    {
	analogWrite(speedPin, 0); 
    }
  };

  // Define the getMotorSpeed function given a command, returns the speed
  int getMotorSpeed(int Command)
  {
    return Command >> 2;
  }

  //  Define the getMotorOn function given a command returns if the motor should be on (true) or off (false)
  bool getMotorOn(int Command)
  {
    int speed = getMotorSpeed(Command);
    if ((speed < 0) || (speed > 255)) { 
      return false; 
    }

    return (Command & 1) == 1;
  }

  //  Define the getMotorForward function given a command, returns true if the motor should spin forward, false if it is to spin backwords or error
  bool getMotorForward(int Command)
  {
    int speed = getMotorSpeed(Command);
    if ((speed < 0) || (speed > 255)) { 
      return false; 
    }

    return (Command & 2) == 2;
  }
};

// This structure describes the L298N Dual H-Bridge Motor Controller based on
//  its ability to control two motors, A and B.
struct motorController
{
  motor a; // Right Motor if camera forward
  motor b;
};

motorController controller;

// PID variables
double twistSetpoint, twistInput, twistOutput, distSetpoint, distInput, distOutput;
PID pidTwist(&twistInput, &twistOutput, &twistSetpoint, 2, 5, 1, DIRECT);
PID pidDistance(&twistInput, &twistOutput, &twistSetpoint, 2, 5, 1, DIRECT);

void setup()   // Setup runs once per reset
{
  // Initialize serial communication @ 9600 baud:
  Serial.begin(9600);
  Serial.print("Starting...\n");

  // Initialize the camera
  pixy.init();

  //Define L298N Dual H-Bridge Motor Controller Pins
  controller.a.dir1Pin = 2;
  controller.a.dir2Pin = 3;
  controller.a.speedPin = 9;
  controller.a.encoderPin = 6;
  strcpy(controller.a.name, "Right Motor");
  controller.b.dir1Pin = 5;
  controller.b.dir2Pin = 4;
  controller.b.speedPin = 10;
  controller.a.encoderPin = 7;
  strcpy(controller.b.name, "Left Motor");

  // Configure the pins for output  
  pinMode(controller.a.dir1Pin,OUTPUT);
  pinMode(controller.a.dir2Pin,OUTPUT);
  pinMode(controller.a.speedPin,OUTPUT);
  pinMode(controller.a.encoderPin, INPUT);
  pinMode(controller.b.dir1Pin,OUTPUT);
  pinMode(controller.b.dir2Pin,OUTPUT);
  pinMode(controller.b.speedPin,OUTPUT);
  pinMode(controller.b.encoderPin, INPUT);

  twistSetpoint = center;
  pidTwist.SetMode(AUTOMATIC);
  pidTwist.SetOutputLimits(-255, 255);
  pidTwist.SetControllerDirection(DIRECT);

}

void loop() 
{
	/*
  if (digitalRead(controller.a.encoderPin) == HIGH)
  {
    Serial.println("Controller a is HIGH");
  }
  else
  {
    Serial.println("Controller a is LOW");
  }
  */

  static elapsedMillis timeElapsed;
  uint16_t blocks;
  int commandLeft;
  int commandRight;
  int errCnt = 0;
  int blockCenter = -1;
 
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are blocks, examine the first one for it's position and 

  blockCenter = blocks ? pixy.blocks[0].x + (pixy.blocks[0].width / 2) : 0;

  if (blockCenter)
  {
	  twistInput = blockCenter;
  }
  twistInput = twistSetpoint;
  if (pidTwist.Compute())
  {
	  char buf[80];
	  sprintf(buf, "twistInput = %d: twistSetpoint = %d:  twistOutput = %d\n", twistInput, twistSetpoint, twistOutput);
	  Serial.print(buf);
	  // adjOutput controls left vs right.  If output >= 0 then turn right, else turn left
	  int adjOutput = (twistOutput >= 0) ? twistOutput : -twistOutput;

	  controller.a.commandMotor(controller.a.generateCommand(true, true,  adjOutput));
	  controller.b.commandMotor(controller.b.generateCommand(true, true, -adjOutput));
  }
}

void printBlocks(uint16_t blocks)
{
  char buf[32]; 
  // do this (print) every 50 frames because printing every
  // frame would bog down the Arduino
  if (iterationCnt%logSample==0)
  {
    sprintf(buf, "Detected %d:\n", blocks);
    Serial.print(buf);
    for (int j=0; j<blocks; j++)
    {
      sprintf(buf, "  block %d: ", j);
      Serial.print(buf); 
      pixy.blocks[j].print();
    }
  }
}

void BothMotorsForward_OnlyOnce()
{
	static bool tripped = false;
	if (!tripped)
	{
		int command = controller.a.generateCommand(true, true, 255);
		controller.a.commandMotor(command);
		controller.b.commandMotor(command);
	}
	tripped = true;
}

void BothMotorsBackword_OnlyOnce()
{
	static bool tripped = false;
	if (!tripped)
	{
		int command = controller.a.generateCommand(true, false, 255);
		controller.a.commandMotor(command);
		controller.b.commandMotor(command);
	}
	tripped = true;
}
