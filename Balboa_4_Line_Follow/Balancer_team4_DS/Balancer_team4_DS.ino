// BELKASSEM Abdellah
// EL AYADI Najim
// TEAM 4

#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balance.h"
#include "Adafruit_TCS34725.h"

bool rouge = false; 
bool vert = false;
bool bleu = false;

const uint8_t SensorCount = 5; //A mettre dans les param (Balance.h)
uint16_t sensorValues[SensorCount];
bool useEmitters = true;
int32_t long count = 0;

int32_t Kpinv = 400;
int32_t Kd = 5;
int32_t error;
int32_t errorP = 0;
int32_t I = 0;
int32_t MVI;
int32_t D;
int32_t MVD;

Balboa32U4LineSensors lineSensors;

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
  balanceSetup();
  ledRed(0);
  lineSensors.setCenterAligned();

  Serial.begin(9600);

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


void FollowLine(){

  int32_t leftSpeed, rightSpeed;
  int32_t avg = 0;
  int32_t sum = 0;
  int32_t pos;
  int32_t Kpinv = 250;
  int32_t Kiinv = 6000;
  int32_t Kd = 5;
  int32_t leftValue;
  int32_t rightValue;
  
  lineSensors.read(sensorValues, useEmitters ? QTRReadMode::On : QTRReadMode::Off);
  
  avg = sensorValues[0]*0 + sensorValues[1]*1 + sensorValues[2]*2 + sensorValues[3]*3 + sensorValues[4]*4;
  sum = sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4];
  leftValue = sensorValues[0] + sensorValues[1] + sensorValues[2];
  rightValue = sensorValues[2] + sensorValues[3] + sensorValues[4];

  pos = (1000*avg)/sum;

  
  error = pos - 2000;

  I += error;

  //saturations
  if(I > 10000){I = I - error;}
  if(I < -10000){I = I - error;}

  MVI = I/Kiinv;
  
  D = error-errorP;
  MVD = D/Kd;  
  
  leftSpeed = 6.5 + (error/Kpinv + MVI + MVD);
  rightSpeed = 6.5 - (error/Kpinv + MVI + MVD);
  errorP = error;
  
  balanceDrive(leftSpeed, rightSpeed);
  
  
  if (leftValue > 7000){
    leftSpeed = -20;
    rightSpeed = 30;
    balanceDrive(leftSpeed, rightSpeed);
  }
  
  if (rightValue > 7000){
    leftSpeed = 30;
    rightSpeed = -20;
    balanceDrive(leftSpeed, rightSpeed);
  }


  
  
}

void detectLine(){
  
  uint16_t time = millis() % 8192;
  int32_t leftSpeed, rightSpeed;
  lineSensors.read(sensorValues, useEmitters ? QTRReadMode::On : QTRReadMode::Off);

  if (sensorValues[0] > 2200 and sensorValues[1] > 2200 and sensorValues[2] > 2200 and sensorValues[3] > 2200 and sensorValues[4] > 2200){
    leftSpeed = -30;
    rightSpeed = 30;
    balanceDrive(leftSpeed, rightSpeed);
  }
  
  else if(sensorValues[0] > 2200 or sensorValues[1] > 2200 or sensorValues[2] > 2200 or sensorValues[3] > 2200 or sensorValues[4] > 2200){
    FollowLine();
  }
  
  else{
    leftSpeed = 6;
    rightSpeed = 6;
    balanceDrive(leftSpeed, rightSpeed);
  }
  
}

void Color(){
 
  Serial.begin(9600);
  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
  int32_t leftSpeed, rightSpeed;
  float red, green, blue;
  
if(count < 200)
{
  count += 1;
  leftSpeed = 5;
  rightSpeed = 5;
  balanceDrive(leftSpeed, rightSpeed);
}
   
else if(count >= 200)
  {
    tcs.getRGB(&red, &green, &blue); 
    if (red > 120 and green < 100 and blue < 100)
      {
      Serial.println("ROUGE ");
      rouge = true; count = 0;
      }

    else if (red < 100 and green > 100 and blue < 100)
      {
      Serial.println("VERT ");
      vert = true; count = 0;
      }

    else if (red < 100 and green < 100 and blue > 120)
      {
      Serial.println("BLEU ");
      bleu = true; count = 0;
      }

    else
      {
      Serial.println("PAS DE COULEUR ");
      rouge = 0; vert = 0; bleu = 0; count = 0;
      }
  }
  
else 
  {
    count = 0;
  }
  
}

void detectColor()
{
  static bool enableSong = false;
  Color();
  int32_t leftSpeed, rightSpeed;

  if (rouge)
  {
    //Serial.println("ROUGE ");
    balanceDrive(0,0);
    uint16_t time = millis() % 8192;
    if (time >= 7000)
    {
      balanceDrive(7,7);
      time = 0;
    }
  }
    
  else if (vert)
  {
    //Serial.println("VERT ");
    balanceDrive(0,15);
    
  }
    
  else if (bleu)
  {
    //Serial.println("BLEU");
    buzzer.play("!>grms>g16>g16>g2");
    balanceDrive(8,8);
  }
    
  else
  {
    //Serial.println("PAS DE COULEUR ");
    detectLine();
  }
}

void driveAround()
{
  uint16_t time = millis() % 8192;
  uint16_t leftSpeed, rightSpeed;
  if (time < 1900)
  {
    leftSpeed = 20;
    rightSpeed = 20;
  }
  else if (time < 4096)
  {
    leftSpeed = 25;
    rightSpeed = 15;
  }
  else if (time < 4096 + 1900)
  {
    leftSpeed = 20;
    rightSpeed = 20;
  }
  else
  {
    leftSpeed = 15;
    rightSpeed = 25;
  }

  balanceDrive(leftSpeed, rightSpeed);
}

void standUp()
{
  motors.setSpeeds(0, 0);
  buzzer.play("!>grms>g16>g16>g2");
  ledGreen(1);
  ledRed(1);
  ledYellow(1);
  while (buzzer.isPlaying());
  motors.setSpeeds(-MOTOR_SPEED_LIMIT, -MOTOR_SPEED_LIMIT);
  delay(400);
  motors.setSpeeds(150, 150);
  for (uint8_t i = 0; i < 20; i++)
  {
    delay(UPDATE_TIME_MS);
    balanceUpdateSensors();
    if(angle < 60000)
    {
      break;
    }
  }
  motorSpeed = 150;
  balanceResetEncoders();
}

void loop()
{
  static bool enableSong = false;
  static bool enableDrive = false;

  balanceUpdate();

  if (isBalancing())
  {
    
    //detectLine();
    detectColor();
    if (enableSong)   { playSong(); }
    if (enableDrive)  { driveAround(); }
  }
  else
  {
    buzzer.stopPlaying();
    balanceDrive(0, 0); // reset driving speeds

    if (buttonA.getSingleDebouncedPress())
    {
      enableSong = false;
      enableDrive = false;
      standUp();
    }
    else if (buttonB.getSingleDebouncedPress())
    {
      enableSong = false;
      enableDrive = true;
      standUp();
    }
    else if (buttonC.getSingleDebouncedPress())
    {
      enableSong = true;
      enableDrive = true;
      standUp();
    }
  }

  // Illuminate the red LED if the last full update was too slow.
  ledRed(balanceUpdateDelayed());

  // Display feedback on the yellow and green LEDs depending on
  // the variable fallingAngleOffset.  This variable is similar
  // to the risingAngleOffset used in Balance.cpp.
  //
  // When the robot is rising toward vertical (not falling),
  // angleRate and angle have opposite signs, so this variable
  // will just be positive or negative depending on which side of
  // vertical it is on.
  //
  // When the robot is falling, the variable measures how far off
  // it is from a trajectory starting it almost perfectly
  // balanced then falling to one side or the other with the
  // motors off.
  //
  // Since this depends on ANGLE_RATE_RATIO, it is useful for
  // calibration.  If you have changed the wheels or added weight
  // to your robot, you can try checking these items, with the
  // motor power OFF (powered by USB):
  //
  // 1. Try letting the robot fall with the Balboa 32U4 PCB up.
  //    The green LED should remain lit the entire time.  If it
  //    sometimes shows yellow instead of green, reduce
  //    ANGLE_RATE_RATIO.
  //
  // 2. If it is tilted beyond vertical and given a push back to
  //    the PCB-up side again, the yellow LED should remain lit
  //    until it hits the ground.  If you see green, increase
  //    ANGLE_RATE_RATIO.
  //
  // In practice, it is hard to achieve both 1 and 2 perfectly,
  // but if you can get close, your constant will probably be
  // good enough for balancing.
  int32_t fallingAngleOffset = angleRate * ANGLE_RATE_RATIO - angle;
  if (fallingAngleOffset > 0)
  {
    ledYellow(1);
    ledGreen(0);
  }
  else
  {
    ledYellow(0);
    ledGreen(1);
  }
}
