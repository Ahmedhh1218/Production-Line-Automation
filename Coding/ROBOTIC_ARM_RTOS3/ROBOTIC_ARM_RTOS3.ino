/*********************************************************************************************************************************
*
* FILE-NAME : ECU2
*
* AUTHOR : TEAM 2
*
* DESCRIPTION : Arduino Nano code that control the suction and robotic arm to store products
*               according to the product type received from ECU1
*
***********************************************************************************************************************************/

#include "Arduino_FreeRTOS.h"
#include "semphr.h"
#include "ezButton.h"
#include "AccelStepper.h"

/*********************************************************************************************************************************
******************************************     Pins Declaration      *************************************************************                                          
**********************************************************************************************************************************/

ezButton X_Switch(9);               // create ezButton object that attach to pin 9 to act as limit switch for x-axis
ezButton Y_Switch(10);              // create ezButton object that attach to pin 10 to act as limit switch for y-axis
ezButton Z_Switch(11);              // create ezButton object that attach to pin 11 to act as limit switch for Z-axis
#define X_Dir 2
#define Y_Dir 3
#define Z_Dir 4
#define X_Step 5
#define Y_Step 6
#define Z_Step 7
#define Suction_DCV 13              // attach the suction DCV pin to pin 13 in shield 
int X_Position[2][4]= {{29450,29450,22950,22950},{29450,29450,22950,22950}};
int Y_Position[2][4]= {{8550,2500,8550,2500},{10000,3750,10000,3750}};
enum Product_Type {Red,Green};
enum Product_No {One,Two,Three,Four};

/*********************************************************************************************************************************
******************************************       Semaphores        ***************************************************************                                          
**********************************************************************************************************************************/

SemaphoreHandle_t Finished_Homing;

/*********************************************************************************************************************************
*****************************************      Setup Function      ***************************************************************                                          
**********************************************************************************************************************************/

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(X_Step,OUTPUT);
  pinMode(X_Dir,OUTPUT);
  pinMode(Y_Step,OUTPUT);
  pinMode(Y_Dir,OUTPUT);
  pinMode(Z_Step,OUTPUT);
  pinMode(Z_Dir,OUTPUT);  
  pinMode(Suction_DCV,OUTPUT);    
  X_Switch.setDebounceTime(50);                                 // set debounce time to 50 milliseconds
  Y_Switch.setDebounceTime(50);                                 // set debounce time to 50 milliseconds
  Z_Switch.setDebounceTime(50);                                 // set debounce time to 50 milliseconds
  Serial.begin(9600);
  Finished_Homing = xSemaphoreCreateBinary();
  xTaskCreate(Homing, "Task1", 100, NULL, 1, NULL);
  xTaskCreate(GoToProduct,"Task2", 100, NULL, 2, NULL);
}

/*********************************************************************************************************************************
*****************************************      LOOP Function      ****************************************************************                                          
**********************************************************************************************************************************/

// the loop function runs over and over again forever
void loop() {
  
}

/*********************************************************************************************************************************
*****************************************      Customized Functions      *********************************************************                                         
**********************************************************************************************************************************/

void Homing (void* ptr)
{
  while(1)
  {
    static int Homing_Counter = 0;    
    X_Switch.loop();                            // MUST call the loop() function first
    Y_Switch.loop();
    Z_Switch.loop();    
    digitalWrite(X_Dir, HIGH);                  // HOMING DIRECTION
    digitalWrite(Y_Dir, HIGH);                  // HOMING DIRECTION    
    digitalWrite(Z_Dir, HIGH);                  // HOMING DIRECTION
    switch(Homing_Counter)
    {
      case 0:
        // Z-Axis Homing Loop
        if(Z_Switch.isPressed() || !Z_Switch.getState())
        {
          Homing_Counter ++;
          digitalWrite(Z_Step,LOW);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);      
        }
        else
        {       
          for (int z = 0;z<10; z++) 
          {   
            digitalWrite(Z_Step,HIGH);
            delayMicroseconds(500);
            digitalWrite(Z_Step,LOW); 
            delayMicroseconds(500);
          }
        }
        break;    
      case 1:
        // Y-Axis Homing Loop 
        if(Y_Switch.isPressed() || !Y_Switch.getState())
        {
          Homing_Counter ++;
          digitalWrite(Y_Step,LOW);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);      
        }
        else
        {       
          for (int y = 0;y<10; y++) 
          {   
            digitalWrite(Y_Step,HIGH);
            delayMicroseconds(500);
            digitalWrite(Y_Step,LOW); 
            delayMicroseconds(500);
          }
        }
        break;
      case 2:
      // X-Axis Homing Loop 
        if(X_Switch.isPressed() || !X_Switch.getState())
        {
          Homing_Counter ++;
          digitalWrite(X_Step,LOW);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);  
          xSemaphoreGive(Finished_Homing);              
        }
        else
        {       
          for (int x = 0;x<10; x++) 
          {   
            digitalWrite(X_Step,HIGH);
            delayMicroseconds(500);
            digitalWrite(X_Step,LOW); 
            delayMicroseconds(500);
          }
        }
        break;
    }    
  }
}

void GoToProduct (void* ptr)
{
  while(1)
  {
    xSemaphoreTake(Finished_Homing,portMAX_DELAY);
    Serial.println("Took Homing Semaphore");
    static int Ready_Counter = 0;     
    digitalWrite(X_Dir, LOW);                  // RETURNING DIRECTION
    digitalWrite(Y_Dir, LOW);                  // RETURNING DIRECTION    
    digitalWrite(Z_Dir, LOW);                  // RETURNING DIRECTION
    for (int x = 0;x<1800; x++) 
    {
      digitalWrite(X_Step,HIGH);
      delayMicroseconds(500);
      digitalWrite(X_Step,LOW); 
      delayMicroseconds(500);
    }
    
    for (int y = 0;y<11500; y++) 
    {   
      digitalWrite(Y_Step,HIGH);
      delayMicroseconds(500);
      digitalWrite(Y_Step,LOW); 
      delayMicroseconds(500);
    }
    for (int z = 0;z<5850;z++)
    {     
      digitalWrite(Z_Step,HIGH);
      delayMicroseconds(500);
      digitalWrite(Z_Step,LOW); 
      delayMicroseconds(500);    
    }
    delay(500);
    digitalWrite(Suction_DCV,LOW);
    delay(500);   
    digitalWrite(Z_Dir, HIGH);    
    digitalWrite(X_Dir, LOW);                  // RETURNING DIRECTION   
    for (int z = 0;z<5850;z++)
    {     
      digitalWrite(Z_Step,HIGH);
      delayMicroseconds(500);
      digitalWrite(Z_Step,LOW); 
      delayMicroseconds(500);    
    }    
    
    for (int x = 0;x<X_Position[Green][One]; x++) 
    {
      digitalWrite(X_Step,HIGH);
      delayMicroseconds(500);
      digitalWrite(X_Step,LOW); 
      delayMicroseconds(500);
    }    
    digitalWrite(Y_Dir, HIGH);                  // HOMING DIRECTION       
    for (int y = 0;y<Y_Position[Green][One]; y++) 
    {   
      digitalWrite(Y_Step,HIGH);
      delayMicroseconds(500);
      digitalWrite(Y_Step,LOW); 
      delayMicroseconds(500);
    }
    digitalWrite(Z_Dir, LOW);       
    for (int z = 0;z<5850;z++)
    {     
      digitalWrite(Z_Step,HIGH);
      delayMicroseconds(500);
      digitalWrite(Z_Step,LOW); 
      delayMicroseconds(500);    
    }    
    delay(500);
    digitalWrite(Suction_DCV,HIGH);
    delay(500);   
    digitalWrite(Z_Dir, HIGH);       
    for (int z = 0;z<5850;z++)
    {     
      digitalWrite(Z_Step,HIGH);
      delayMicroseconds(500);
      digitalWrite(Z_Step,LOW); 
      delayMicroseconds(500);    
    }    
  }
}