#include <Wire.h>
#include "Balance.h"

const uint8_t SensorCount = 5; //??A mettre dans les paramètres constant (Balance.h)??
uint16_t sensorValues[SensorCount];
bool sensorBool[SensorCount];
const int32_t tresholds = 1000;
bool useEmitters = true;
int mode = 0; //Mode qui permet de switch suivant la situation (tournant, ligne, T, etc)
int error = 0; //Erreur sur la trajectoire
Balboa32U4LineSensors lineSensors;

char path[100] = " ";
unsigned char pathLength = 0;
int pathIndex = 0;

# define CONT_LINE 0
# define NO_LINE 1
# define RIGHT_TURN 2
# define LEFT_TURN 3
# define FOLLOWING_LINE 4

void calculateTresholds()
{
    lineSensors.setCenterAligned();
    lineSensors.read(sensorValues, useEmitters ? QTRReadMode::On : QTRReadMode::Off);
    int i;
    for(i=0;i<5; i=i+1)
    {
        if(sensorValues[i]>= tresholds)
        {
            sensorBool[i] = true;
        }   
        else
        {
            sensorBool[i] = false;
        }
    }
}

void estimateMode()
{
    /*
    Special case :
    x111x => Line, continue a little and test : end of maze (stop) or intersection (go left)
    x000x => no line, dead end (turn around)
    11xx0 => Left turn (go left)
    0xx11 => Right turn, continue a little and test : only right turn (go right) or straight or right turn (go straight)

    Continuous line (go straight)
    x001x => error 2
    x011x => error 1
    x010x => error 0
    x110x => error -1
    x100x => error -2
    */

    calculateTresholds();

    if     ((sensorBool[0]== 1 )&&(sensorBool[1]== 1 )&&(sensorBool[2]== 1 )&&(sensorBool[3]== 1 )&&(sensorBool[4]== 1 ))  {mode = CONT_LINE; error = 0;}
    else if((sensorBool[0]== 0 )&&(sensorBool[1]== 0 )&&(sensorBool[2]== 0 )&&(sensorBool[3]== 0 )&&(sensorBool[4]== 0 ))  {mode = NO_LINE; error = 0;}
    else if((sensorBool[0]== 0 )&&(sensorBool[3]== 1 )&&(sensorBool[4]== 1 ))                                              {mode = RIGHT_TURN; error = 0;}
    else if((sensorBool[0]== 1 )&&(sensorBool[1]== 1 )&&(sensorBool[4]== 0 ))                                              {mode = LEFT_TURN; error = 0;}
    else if((sensorBool[1]== 0 )&&(sensorBool[2]== 0 )&&(sensorBool[3]== 1 ))                                              {mode = FOLLOWING_LINE; error = 2;}
    else if((sensorBool[1]== 0 )&&(sensorBool[2]== 1 )&&(sensorBool[3]== 1 ))                                              {mode = FOLLOWING_LINE; error = 1;}
    else if((sensorBool[1]== 0 )&&(sensorBool[2]== 1 )&&(sensorBool[3]== 0 ))                                              {mode = FOLLOWING_LINE; error = 0;}
    else if((sensorBool[1]== 1 )&&(sensorBool[2]== 1 )&&(sensorBool[3]== 0 ))                                              {mode = FOLLOWING_LINE; error =- 1;}
    else if((sensorBool[1]== 1 )&&(sensorBool[2]== 0 )&&(sensorBool[3]== 0 ))                                              {mode = FOLLOWING_LINE; error = -2;}

    Serial.print ("  mode: ");
    Serial.print (mode);
    Serial.print ("  error:");
    Serial.println (error);
    Serial.print(sensorBool[0]);Serial.print(sensorBool[1]);Serial.print(sensorBool[2]);Serial.print(sensorBool[3]);Serial.println(sensorBool[4]);
}

void findSolution()
{
    estimateMode();

    switch (mode)
    {
        case CONT_LINE:
            testFurther();
            estimateMode();
            if (mode != CONT_LINE) {goBack(); turnLeft();}
            else
                finishLine();
            break;

        case NO_LINE:
            turnBack();
            break;

        case RIGHT_TURN:
            testFurther();
            estimateMode();
            if (mode == NO_LINE) {turnRight();}
            break;

        case LEFT_TURN:
            testFurther();
            turnLeft();
            break;

        case FOLLOWING_LINE:
            followLine();
            break; 
    }

}

void testFurther()
{   
    Serial.println(" We go a little forward!");
    
    unsigned long sampleTime = millis();
    while(millis() - sampleTime <= 400)
    {
      estimateMode();
      followLine();
    }
}

void goBack()
{
    Serial.println(" We go back a little!");

    unsigned long sampleTime = millis();
    while(millis() - sampleTime <= 300)
    {
      motors.setSpeeds(-45, -45); //in balance mode, use balanceDrive(-5,-5) instead
    }
}

void turnLeft()
{
    Serial.println(" We turn left!");

    unsigned long sampleTime = millis();
    while(millis() - sampleTime <= 310)
    {
      motors.setSpeeds(-100,100); //in balance mode, use balanceDrive(-10,-10) instead
    }

    unsigned long sampleTime2 = millis();
    while(millis() - sampleTime2 <= 200)
    {
      estimateMode();
      followLine();
    }

    keepInMemory('L');
}

void turnRight()
{
    Serial.println(" We turn right!");

    goBack();

    unsigned long sampleTime = millis();
    while(millis() - sampleTime <= 310)
    {
      motors.setSpeeds(100,-100); //in balance mode, use balanceDrive(10,-10) instead
    }

    unsigned long sampleTime2 = millis();
    while(millis() - sampleTime2 <= 200)
    {
      estimateMode();
      followLine();
    }

    keepInMemory('R');
}

void turnBack()
{
    Serial.println(" We turn back!");

    unsigned long sampleTime = millis();
    while(millis() - sampleTime <= 585)
    {
      motors.setSpeeds(100,-100); //in balance mode, use balanceDrive(-10,10) instead
    }

    unsigned long sampleTime2 = millis();
    while(millis() - sampleTime2 <= 200)
    {
      estimateMode();
      followLine();
    }

    keepInMemory('D');
}

void followLine()
{
    Serial.println(" We go forward!");
    const int Kp = 40; 
    const int Ki = 0;
    const int Kd = 0;
    int P;
    int I;
    int D;
    int PID;
    int previousError;

    P = error;
    I = I + error;
    D = error-previousError;
  
    PID = ((Kp*P) + (Ki*I) + (Kd*D))/10;
    previousError = error;

    motorSpeed = 45;

    if (motorSpeed > MOTOR_SPEED_LIMIT)
    {
      motorSpeed = MOTOR_SPEED_LIMIT;
    }
    if (motorSpeed < -MOTOR_SPEED_LIMIT)
    {
      motorSpeed = -MOTOR_SPEED_LIMIT;
    }

    motors.setLeftSpeed( motorSpeed + PID);
    motors.setRightSpeed( motorSpeed - PID); //in balance mode, use balanceDrive(5 + PID/10,5 - PID/10) instead

    Serial.println(motorSpeed + PID);
    Serial.println(motorSpeed - PID);
}

void finishLine()
{
    Serial.println(" We have arrrived!");

    stopMotor();

    Serial.print("Shortest path: ");
    for(int i=0;i<pathLength;i++)
    Serial.print(path[i]);
    Serial.println("");
    Serial.print("Length of path :");
    Serial.println(pathLength);
}

void stopMotor()
{
  motors.setSpeeds(0,0);
}

void keepInMemory(char rightWay)
{
  path[pathLength] = rightWay;
  pathLength ++;
  simplifyPath();
}

void simplifyPath()
{
  if(pathLength < 3 || path[pathLength-2] != 'D')
    return;

  int angleSum = 0;
  int i;
  for(i=1;i<=3;i++)
  {
    switch(path[pathLength-i])
    {
      case 'R':
        angleSum += 90;
  break;
      case 'L':
  angleSum += 270;
  break;
      case 'D':
  angleSum += 180;
  break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  angleSum = angleSum % 360;

  // Replace all of those turns with a single one.
  switch(angleSum)
  {
    case 0:
  path[pathLength - 3] = 'S';
  break;
    case 90:
  path[pathLength - 3] = 'R';
  break;
    case 180:
  path[pathLength - 3] = 'D';
  break;
    case 270:
  path[pathLength - 3] = 'L';
  break;
  }

  // The path is now two steps shorter.
  pathLength -= 2;
} 

void findBestSolution()
{
  estimateMode();
  switch (mode)
  {
    case FOLLOWING_LINE:
      followLine();
      break;    
    case CONT_LINE:
      if (pathIndex >= pathLength) finishLine(); 
      else {switchOptimisation(path[pathIndex]); pathIndex++;}
      break;  
    case LEFT_TURN:
      if (pathIndex >= pathLength) finishLine(); 
      else {switchOptimisation(path[pathIndex]); pathIndex++;}
      break;  
    case RIGHT_TURN:
      if (pathIndex >= pathLength) finishLine(); 
      else {switchOptimisation(path[pathIndex]); pathIndex++;}
      break;   
  }    
}

void switchOptimisation(char dir)
{
  switch(dir)
  {
    case 'L': // Turn Left
       turnLeft();      
       break;   
    
    case 'R': // Turn Right
       turnRight();     
       break;   
       
    case 'D': // Turn Back
       turnBack();     
       break;   
       
    case 'S': // Go Straight
       testFurther(); 
       break;
  }
}
