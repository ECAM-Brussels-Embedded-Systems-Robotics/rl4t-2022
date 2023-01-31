#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balancer_team2_V1.h"


LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
Balboa32U4ButtonC buttonC;

char report[60];

int32_t count = 0;
String message;             
String spDistance;
int32_t spDist;


void setup()
{
  ledYellow(0);
  ledRed(1);
  balanceSetup();  
  ledRed(0);
  
  Serial1.begin(9600);  // Opening port to read voice
}


const char song[] PROGMEM =
  "!O6 T240"
  "l32ab-b>cl8r";

void playSong()
{
  if (!buzzer.isPlaying())
  {
    buzzer.playFromProgramSpace(song);
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



void avancer()
{
  uint16_t leftSpeed, rightSpeed;
  
  leftSpeed = 10;
  rightSpeed = 10;
  
  balanceDrive(leftSpeed, rightSpeed);  
}

void avancerDe(int32_t setPoint) // Pas operationel !
{
  /* 1m correspond à environ 5105 pulses de l'encodeur*/
  
  uint16_t leftSpeed, rightSpeed;
  
  int32_t distanceLeft, distanceRight;
  int16_t countsLeft = encoders.getCountsLeft(), countsRight = encoders.getCountsRight();

  balanceResetEncoders(); 

  distanceLeft = countsLeft/5105;
  distanceRight = countsRight/5105;
  
  if(distanceLeft < setPoint && distanceRight < setPoint)
  {
    leftSpeed = 10;
    rightSpeed = 10;
  }
  balanceDrive(leftSpeed, rightSpeed);
}

void reculer()
{
  uint16_t leftSpeed, rightSpeed;

  leftSpeed = -10;
  rightSpeed = -10;
  
  balanceDrive(leftSpeed, rightSpeed);
}



void arret()
{
  uint16_t leftSpeed, rightSpeed;

  leftSpeed = 0;
  rightSpeed = 0;
  balanceDrive(leftSpeed, rightSpeed);
}


void tournerGauche()
{
  uint16_t leftSpeed, rightSpeed;
  
  leftSpeed = -5;
  rightSpeed = 5;
  balanceDrive(leftSpeed, rightSpeed);
}


void tournerDroite()
{
  uint16_t leftSpeed, rightSpeed;
  
  leftSpeed = 5;
  rightSpeed = -5;
  balanceDrive(leftSpeed, rightSpeed);
}



void loop()
{
  /* S'occupe de la recuperation du message et de la conversion */
  while (Serial1.available())     // check if there is an available byte to read
  {
    delay(10);                  // Delay added to make thing stable
    char c = Serial1.read();    // Conduct a serial read
      
    if (c == '#')                
    {
      break;                    // Exit the loop when the # is detected after the void
    }

    else if (c=='0' or  c=='1' or c=='2' or c=='3' or c=='4' or c=='5' or c=='6' or c=='7' or c=='8' or c=='9')
    {
      /* Fait la conversion du caractere chiffre en integer */
      spDistance = c;
      spDist = spDistance.toInt();
      
      message += c; 
    }
    
    else
    {
      message += c; 
    }
  }

  
  static bool enableSong = false;
  static bool enableDrive = false;
  int32_t Time;
  balanceUpdate(); 

  //---------------------------------------------- COMMANDES LORS DU BALANCEMENT ----------------------------------------------
  if (isBalancing())
  {
    
    Serial.print("Compte : ");
    Serial.println(count);
    
    if (message.length()>0)
    { 
      Serial.println("Voice command : " + message);
      
      if (message == "avance")
      {
        avancer();
      }

      else if (message == "avance de"+spDist)
      {
        avancerDe(spDist);   
      }
      
      else if (message == "recule" or message == "reviens")
      {
        reculer();                      
      }
  
      
      else if (message == "stop")
      {
        arret();
      }

     else if (message == "gauche")
     {
       if(count < 2750)
       {
        tournerGauche();
        count++;
       }
       else
       {
        arret();
        count = 0;
        message = "";
       }      
     }


      else if (message == "droite")
      {
       if(count < 2750)
       {
        tournerDroite();
        count++;
       }
       else
       {
        arret();
        count = 0;
        message = "";
       }
      }

      
      else
      {
        playSong(); 
        message = ""; 
      }
      
      spDistance = '0';   // Reset distance set point
    }  
  }


  //---------------------------------------------- COMMANDES LORSQUE LE ROBOT EST COUCHÉ ----------------------------------------------
  else
  {
    buzzer.stopPlaying();
    balanceDrive(0, 0); // reset driving speeds

    
   if (message.length()>0)
    {       
      Serial.println("Voice command : " + message);

      if (message == "debout") 
      {
        enableSong = false;
        enableDrive = false;
        standUp();
        balanceResetEncoders();
      }

      else
      {
        playSong();  
      }
      
      message = "";               // Reset voice
      spDistance = '0';   // Reset distance set point
    }
  }
      
  // Illuminate the red LED if the last full update was too slow.
  ledRed(balanceUpdateDelayed());

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
