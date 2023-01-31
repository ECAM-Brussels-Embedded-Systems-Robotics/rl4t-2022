// Maze solving algorithm
// CAREFULL, PLEASE USE SUPPORT TO KEEP THE BALBOA UP OR CHANGE THE CODE ACCORDINGLY !!!!!!!!!

#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balance.h"

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
Balboa32U4ButtonC buttonC;

void setup()
{
  // Uncomment these lines if your motors are reversed.
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);

  ledYellow(0);
  ledRed(1);
  ledRed(0);
}

const char song[] PROGMEM =
  "!O6 T240"
  "l32ab-b>cl8r br b-bb-a a-r gr g-4 g4"
  "a-r gr g-gg-f er e-r d4 e-4"
  "gr msd8d8ml d-4d4"
  "l32efg-gl8r msd8d8ml d-4d4"
  "<bcd-d e-efg- ga-ab- a4 gr";

void playSong()
{
  if (!buzzer.isPlaying())
  {
    buzzer.playFromProgramSpace(song);
  }
}

void loop()
{
  static bool enableSong = false;
  static bool enableMaze = false;
  static bool enableOpti = false;
  buzzer.stopPlaying();

  //CAREFULL !! Uncomment balanceUpdate and put the condition if(isBalancing) if you don't use a support for the Balboa
  //balanceUpdate

  if (enableMaze)
  {
  findSolution();
  }

  if (enableOpti)
  {
  findBestSolution();
  }
  
    if (buttonA.getSingleDebouncedPress())
    {
      enableSong = false;
      enableMaze = true;
      delay(3000);
    }
    else if (buttonB.getSingleDebouncedPress())
    {
      enableSong = false;
      enableOpti = true;
      delay(3000);
    }
    else if (buttonC.getSingleDebouncedPress())
    {
      enableSong = true;
    }
}
