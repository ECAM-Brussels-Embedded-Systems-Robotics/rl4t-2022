#include <Balboa32U4.h>             //|
#include <Wire.h>                   //|
#include <LSM6.h>                   //|
#include "Balance.h"                //|
                                    //|
LSM6 imu;                           //| Bibliothèques et variables du robot
Balboa32U4Motors motors;            //|
Balboa32U4Encoders encoders;        //|
                                    //|
unsigned int leftSpeed, rightSpeed; //|

unsigned long currentTime=0;        //|
unsigned long previousTime=0;       //| Utilisé pour cadencer le code principal
unsigned long zeroTime;             //|

unsigned long startTime;    // Utilisé pour faire le 180°

// Raw values
int C_droite_Raw;
int C_gauche_Raw;
int C_arriere_Raw;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Normalisation des valeurs pour travailler avec +- les mêmes valeurs peu importe la luminosité de l'ambiance 

// 1) Value after 1 second
int C_droite_Raw_FV;
int C_gauche_Raw_FV;
int C_arriere_Raw_FV;

// 2) Calibrated values
int C_droite_CV;
int C_gauche_CV;
int C_arriere_CV;

// 3) Mapped value 
int C_droite_MV;
int C_gauche_MV;
int C_arriere_MV;

// Fin de normalisation
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Ambiance value
int Val_amb;

// Sensibility
int Sensi = 7;              // To move
int Sensi_nothing = 20;     // To do nothing

void setup()
{
  Serial.begin(9600);
  
  pinMode(6, OUTPUT);   // On déclare le buzzer du balboa en tant que sortie

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ledYellow(0);
  ledRed(1);
  balanceSetup();     // Initialisation de l'IMU + calibration des angles et vérification si erreur lors du relevé des angles 
  ledRed(0);
  // Quand la lumière rouge du robot s'éteint, ça nous indique que le setup est fini
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void loop()
{ 
  // Permet de mettre à jour l'orientation du robot et de continuer à le réguler sans arrêt 
  balanceUpdate();

  // Récupération des valeurs brutes
  C_droite_Raw = analogRead(A0);
  C_gauche_Raw = analogRead(A4);
  C_arriere_Raw = analogRead(A3);

  if (isBalancing())  // Lorsque le robot est debout
  {
      zeroTime = millis() - currentTime;      // Dès que le robot est mis debout, zeroTime commence à 0 et commence à compter

      // On va normaliser les valeurs brutes pour qu'elle soit utilisable dans n'importe quel environnement (lumineux ou sombre)
      RawToMappedValue();
      
      // BIIIP pendant 1,5sec dès que le robot est prêt à être utilisé
      if ((zeroTime > 2000) && (zeroTime <= 3600))
      {
        BipReady();
      }

      // Le robot est prêt à être controlé avec la lumière
      if (zeroTime > 3600)
      {
        RobotControl();
      }
  }
  else      // Lorsque le robot est couché
  {
    currentTime = millis();   // Enregistre le temps actuel en milliseconde  
  }
}

///////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////FONCTIONS/////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
void RawToMappedValue()
{
  // Après 1 secondes, on enregistre la valeur qu'on met dans une nouvelle variable
  if(zeroTime==1000) 
  {
    C_droite_Raw_FV = C_droite_Raw;
    C_gauche_Raw_FV = C_gauche_Raw;
    C_arriere_Raw_FV = C_arriere_Raw;
  }

  // On fait la moyenne des trois valeurs pour avoir la luminosité ambiante
  Val_amb = (C_droite_Raw_FV+C_gauche_Raw_FV+C_arriere_Raw_FV)/3;

  // On soustrait la luminosité ambiante des valeurs brutes pour commencer à travailler autour de 0 
  C_droite_CV = C_droite_Raw - Val_amb;
  C_gauche_CV = C_gauche_Raw - Val_amb;
  C_arriere_CV = C_arriere_Raw - Val_amb;

  // On va mapper les valeurs calibrées pour avoir des valeurs entre 0 et 200 et on va travailler avec ses valeurs 
  C_droite_MV = map(C_droite_CV,0,1200,0,200);
  C_gauche_MV = map(C_gauche_CV,0,1200,0,200);
  C_arriere_MV = map(C_arriere_CV,0,1200,0,200);

  // Si on obtient des valeurs négatives on les met à 0
  if (C_droite_MV < 0)
  {
    C_droite_MV = 0;
  }
  if (C_gauche_MV < 0)
  {
    C_gauche_MV = 0;
  }
  if (C_arriere_MV < 0)
  {
    C_arriere_MV = 0;
  }
}

void BipReady()
{
  if (zeroTime-previousTime >= 200)
  {
    //tone(6, 1000);        // On active le buzzer
    
    if (zeroTime-previousTime >= 500)
    {
      //noTone(6);          // On désactive le buzzer
      previousTime = zeroTime;
    }
  }
}

void RobotControl()
{
  if ((C_droite_MV<Sensi_nothing) && (C_gauche_MV<Sensi_nothing) && (C_arriere_MV<Sensi_nothing)) // Le robot reste debout, il ne fait rien
  {
    Serial.println("rien");
    leftSpeed = 0;
    rightSpeed = 0;
  }
  else if ((C_gauche_MV - C_droite_MV) > Sensi) // Le robot se dirige vers la gauche
  {
    Serial.println("gauche");
    leftSpeed = -25;
    rightSpeed = -10;
  }
  else if ((C_droite_MV - C_gauche_MV) > Sensi) // Le robot se dirige vers la droite
  {
    Serial.println("droite");
    leftSpeed = -10;
    rightSpeed = -25;
  }
  
  else if (C_arriere_MV>C_droite_MV || C_arriere_MV>C_gauche_MV)   // Le robot se tourne de 180° pour se remettre face à la lumière
  {
    startTime = millis();
    while (millis() - startTime <= 850)
   {
      balanceUpdate();
      Serial.println("Faire 180°");
      leftSpeed = 10;
      rightSpeed = -10;
      balanceDrive(leftSpeed,rightSpeed);
    } 
  }
  
  else  // Le robot avance en ligne droite
  {
    Serial.println("tout droit");
    leftSpeed = -15;
    rightSpeed = -15;
  }
  balanceDrive(leftSpeed,rightSpeed);
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////FIN FONCTIONS///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
