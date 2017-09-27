/*
  DualStepperMotorTest
  By Austin Clementi
*/

#include <Stepper.h>

//#define DEBUG 1
//#define NORTHERN_MOTOR 1
//#define EASTERN_MOTOR 1

// define structures
struct MOTOR_PINS
{
  int RED;
  int BLUE;
  int GREEN;
  int BLACK;
};

struct ICOORDINATES
{
  int NORTH;
  int EAST;
};

struct FCOORDINATES
{
  float NORTH;
  float EAST;
};

// define constants
const float STEP_SIZE = 0.9;
const int STEP_DELAY = 10;
const MOTOR_PINS stepperNorthPins {8, 9, 10, 11};
const MOTOR_PINS stepperEastPins {8, 9, 10, 11};

// initialize the stepper library
Stepper stepperNorth(360/STEP_SIZE, stepperNorthPins.RED, stepperNorthPins.BLUE, stepperNorthPins.GREEN, stepperNorthPins.BLACK);
Stepper stepperEast(360/STEP_SIZE, stepperEastPins.RED, stepperEastPins.BLUE, stepperEastPins.GREEN, stepperEastPins.BLACK);

// initialise functions
int convertSingleRotation(int, int);
int calcStepperDirection(int next, int current);
void printICoordinates(ICOORDINATES);
void printFCoordinates(FCOORDINATES);
int angleToSteps(int);
ICOORDINATES convertCoordinates (FCOORDINATES);
FCOORDINATES getFCoordinates (void);

void setup()
{
  // initialize serial comunication
  Serial.begin(9600);
}

void loop ()
{
  // initialise coordinate variables
  static ICOORDINATES currentSteps, nextSteps;
  static FCOORDINATES nextAngle;
  static int repeat = 1;
  {
    // get the next coordinates
    nextAngle = getFCoordinates();
    #ifdef DEBUG
    Serial.print("Entered Coordinates");
    printFCoordinates(nextAngle);
    #endif
    
    //convert the coordinates to steps
    nextSteps = convertCoordinates (nextAngle);
    #ifdef DEBUG
    Serial.print("Step Coordinates");
    printICoordinates(nextSteps);
    #endif

    //convert step range to [-179 deg, 180 deg]
    nextSteps.NORTH = convertSingleRotation(nextSteps.NORTH, 360/STEP_SIZE);
    nextSteps.EAST = convertSingleRotation(nextSteps.EAST, 360/STEP_SIZE);
    #ifdef DEBUG
    Serial.print("Step Coordinates within Range");
    printICoordinates(nextSteps);
    #endif

    //calculate actual coordinates
    FCOORDINATES actualAngle;
    actualAngle.NORTH = nextSteps.NORTH * STEP_SIZE;
    actualAngle.EAST = nextSteps.EAST * STEP_SIZE;
    #ifdef DEBUG
    Serial.print("Actual Angle in Degrees");
    printFCoordinates(actualAngle);
    #endif
    
    // print the actual coordinates
    {
      Serial.print("Due to step sizes of ");
      Serial.print(STEP_SIZE);
      Serial.print(" the coordinates will be");
      printFCoordinates(actualAngle);
    }
    #ifdef EASTERN_MOTOR
    // move EAST stepper motor
    int directionEast = calcStepperDirection(nextSteps.EAST, currentSteps.NORTH);
    for( currentSteps.EAST; nextSteps.EAST != currentSteps.EAST ; currentSteps.EAST += directionEast )
    {
      stepperEast.step( directionEast );
      delay(STEP_DELAY);
    }
    #endif

    #ifdef NORTHERN_MOTOR
    //move NORTH stepper motor
    int directionNorth = calcStepperDirection(nextSteps.NORTH, currentSteps.NORTH);
    for( currentSteps.NORTH; nextSteps.NORTH != currentSteps.NORTH ; currentSteps.NORTH += directionNorth )
    {
      stepperNorth.step( directionNorth );
      delay(STEP_DELAY);
    }
    #endif
  }
  return 0;
}

FCOORDINATES getFCoordinates (void)
{
  // initialise coordinate variable
  static FCOORDINATES next;
    /* North */
  // Ask for coordinates
  Serial.print("Enter next latitude (NORTH) coordinant\n");
  //waiting for input
  while (Serial.available()==0);
  // Ask for and recieve coordinates
  next.NORTH = Serial.parseFloat();  //read int
  
    /* East */
  // Ask for coordinates
  Serial.print("Enter next longitude (EAST) coordinant\n");
  //waiting for input
  while (Serial.available()==0);
  // Ask for and recieve coordinates
  next.EAST = Serial.parseFloat();  //read int

  return next;
}

ICOORDINATES convertCoordinates (FCOORDINATES angle)
{
  // initialise coordinate variable
  ICOORDINATES steps;
  steps.NORTH = angleToSteps(angle.NORTH);
  steps.EAST = angleToSteps(angle.EAST);
  return steps;
}

int angleToSteps(int angle)
{
  //initialse variables
  int steps;
  
  //convert to steps
  steps = (long long) angle / STEP_SIZE;
  
  return steps;
}

void printICoordinates(ICOORDINATES C)
{
  Serial.print("[");
  Serial.print(C.NORTH);
  Serial.print(",");
  Serial.print(C.EAST);
  Serial.print("]\n");
}

void printFCoordinates(FCOORDINATES C)
{
  Serial.print("[");
  Serial.print(C.NORTH);
  Serial.print(",");
  Serial.print(C.EAST);
  Serial.print("]\n");
}


int calcStepperDirection(int next, int current)
{
  int stepperDirection = 0;
  if (next > current)
  {
    stepperDirection = 1;
  }
  if (next < current)
  {
    stepperDirection = -1;
  }
  return stepperDirection;
}

int convertSingleRotation(int value, int range)
{
    while(value > range/2)
  {
    value -= range;
  }
  while (value <= -range/2)
  {
    value += range;
  }
  return value;
}

