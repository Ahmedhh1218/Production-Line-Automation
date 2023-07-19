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

/*********************************************************************************************************************************
******************************************     Pins Declaration      *************************************************************
******************************************    Hardware Definition    *************************************************************                                          
**********************************************************************************************************************************/

ezButton X_Switch(9);               // create ezButton object that attach to pin 9 to act as limit switch for x-axis
ezButton Y_Switch(A0);              // create ezButton object that attach to pin 10 to act as limit switch for y-axis
ezButton Z_Switch(A1);              // create ezButton object that attach to pin 11 to act as limit switch for Z-axis
#define Suction_DCV 13              // attach the suction DCV pin to pin 13 in shield 
#define Upper_Receiver 12

// declaration of the pins controlling the direction and step of each motor (According to data sheet of CNC NANO SHIELD)
#define X_Dir 2
#define Y_Dir 3
#define Z_Dir 4
#define X_Step 5
#define Y_Step 6
#define Z_Step 7

// defining the storing patch and locations in the production line
const int Products_Types=2;         // either Large/Small
const int Products_Count=4;         // 4 products of each type

// locations defined from top left to bottom right
#define First_Row 29450
#define Second_Row 22950
#define First_Col 8550
#define Second_Col 2500
#define Third_Col 3750
#define Fourth_Col 10000
#define Large_Z 5850
#define Small_Z 6850
int X_Position[Products_Types][Products_Count]= {{First_Row,First_Row,Second_Row,Second_Row},{First_Row,First_Row,Second_Row,Second_Row}};  // saved X-locations
int Y_Position[Products_Types][Products_Count]= {{First_Col,Second_Col,First_Col,Second_Col},{Fourth_Col,Third_Col,Fourth_Col,Third_Col}};  // saved Y-locations
enum Product_Type {Small,Large};
enum Product_No {One,Two,Three,Four};

/*********************************************************************************************************************************
***************************************     Functions Declaration      ***********************************************************                                         
**********************************************************************************************************************************/

enum Suction_State{Suction_ON, Suction_OFF};                      // Suction state either ON or OFF
enum Direction{Away, Home};                                       // 2 directions for each axis either towards the Home position or away from it
enum Motor{X_Axis, Y_Axis, Z_Axis};                               // 3 overall motors in the line
void H_Stepper_Actuate(int Motor, bool Direction, int Steps);     // our customized function for actuating any motor in any direction and with the required steps
void H_Stepper_Stall(int Motor);                                  // our customized function for stopping any motor

/*********************************************************************************************************************************
******************************************       Semaphores        ***************************************************************                                          
**********************************************************************************************************************************/

SemaphoreHandle_t Finished_Homing;
SemaphoreHandle_t At_Product;
SemaphoreHandle_t Finished_Storing;

/*********************************************************************************************************************************
*****************************************      Setup Function      ***************************************************************                                          
**********************************************************************************************************************************/

// the setup function runs once when you press reset or power the board
void setup() {
  // All Hardware Pins on this microcontroller are outputs  
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
  At_Product = xSemaphoreCreateBinary();  
  Finished_Storing = xSemaphoreCreateBinary();
  xTaskCreate(Homing, "Task1", 100, NULL, 1, NULL);
  xTaskCreate(GoToProduct,"Task2", 100, NULL, 2, NULL);
  //xTaskCreate(Store,"Task3",100,NULL,3,NULL);
  }

/*********************************************************************************************************************************
*****************************************      LOOP Function      ****************************************************************                                          
**********************************************************************************************************************************/

// the loop function runs over and over again forever
void loop() {
  
}

/*********************************************************************************************************************************
************************************      Customized RTOS Functions      *********************************************************                                         
**********************************************************************************************************************************/

// Function that aims to Home the Robotic Arm at Powering up or Reset
void Homing (void* ptr)
{
  while(1)
  {
    static int Homing_Counter = 0;              // counter to check that an axis has homed
    X_Switch.loop();                            // MUST call the loop() function first
    Y_Switch.loop();                            // MUST call the loop() function first
    Z_Switch.loop();                            // MUST call the loop() function first
    
    switch(Homing_Counter)
    {
      case 0:
      // Z-Axis Homing Loop
      if(Z_Switch.isPressed() || !Z_Switch.getState())
      {
        Homing_Counter ++;
        H_Stepper_Stall(Z_Axis);     
      }
      else
      {       
        H_Stepper_Actuate(Z_Axis, Home, 10);
      }
      break;    
      
      case 1:
      // Y-Axis Homing Loop 
      if(Y_Switch.isPressed() || !Y_Switch.getState())
      {
        Homing_Counter ++;
        H_Stepper_Stall(Y_Axis);      
      }
      else
      {       
        H_Stepper_Actuate(Y_Axis, Home, 10);
      }
      break;
      
      case 2:
      // X-Axis Homing Loop 
      if(X_Switch.isPressed() || !X_Switch.getState())
      {
        Homing_Counter ++;
        H_Stepper_Stall(X_Axis);
        xSemaphoreGive(Finished_Homing); 
                   
      }
      else
      {      
        H_Stepper_Actuate(X_Axis, Home, 2);
      }
      break;      
    }    
  }
}

/*****************************************************************************************************************************
******************************************************************************************************************************/

// Function that aims to stand above the product location in the magazine after being Homed at the power up or reset
void GoToProduct (void* ptr)
{
  while(1)
  {
    xSemaphoreTake(Finished_Homing,portMAX_DELAY);
    Serial.println("Took Homing Semaphore");
    
    H_Stepper_Actuate(X_Axis, Away, 1800);
    H_Stepper_Actuate(Y_Axis, Away, 11500);
    H_Stepper_Actuate(Z_Axis,Away,5850);
    delay(1000);    
    H_Stepper_Actuate(Z_Axis,Home,5850);
    H_Stepper_Actuate(Z_Axis,Away,X_Position[Large][One]);
    H_Stepper_Actuate(Z_Axis,Home,Y_Position[Large][One]);    
  }
}

/*****************************************************************************************************************************
******************************************************************************************************************************/

/*// Function that aims to stand above the product location in the magazine after being Homed at the power up or reset
void Store (void* ptr)
{
  while(1)
  {
    xSemaphoreTake(At_Product,portMAX_DELAY);    
    Serial.println("Took Storing Semaphore");
    
    static int Small_Counter = 0;
    static int Large_Counter = 0;

    if(digitalRead(Upper_Receiver))
    {
      Large_Counter++;      
    }   
    else
    {
      Small_Counter++;
    } 
    switch(Large_Counter)
    {
      case One:
      H_Stepper_Actuate(Z_Axis,Away,Large_Z);
      delay(500);      
      digitalWrite(Suction_DCV,Suction_ON);
      delay(500); 
      H_Stepper_Actuate(Z_Axis,Home,Large_Z); 
      H_Stepper_Actuate(X_Axis,Away,X_Position[Large][One]); 
      H_Stepper_Actuate(Y_Axis,Home,Y_Position[Large][One]);
      delay(500);      
      digitalWrite(Suction_DCV,Suction_OFF);
      delay(500);
      H_Stepper_Actuate(Z_Axis,Home,Large_Z); 
      H_Stepper_Actuate(Y_Axis,Away,Y_Position[Large][One]); 
      H_Stepper_Actuate(X_Axis,Home,X_Position[Large][One]); 
      break; 
    }  
    xSemaphoreGive(At_Product);
  }
}

/*********************************************************************************************************************************
****************************************      Customized  Functions      *********************************************************                                         
**********************************************************************************************************************************/

// our customized function that actuate any motor in the project in both direction with the favoured steps count
void H_Stepper_Actuate(int Motor, bool Direction, int Steps)
{
  switch(Motor)
  {
    case X_Axis:
    switch(Direction)
    {
      case Away:
      digitalWrite(X_Dir, LOW);
      for (int x = 0;x<Steps; x++) 
      {
        digitalWrite(X_Step,HIGH);
        delayMicroseconds(500);
        digitalWrite(X_Step,LOW); 
        delayMicroseconds(500);
      }
      break;
      case Home:
      digitalWrite(X_Dir, HIGH);
      for (int x = 0;x<Steps; x++) 
      {
        digitalWrite(X_Step,HIGH);
        delayMicroseconds(500);
        digitalWrite(X_Step,LOW); 
        delayMicroseconds(500);
      }
      break;
    } 
    break;
    case Y_Axis:
    switch(Direction)
    {
      case Away:
      digitalWrite(Y_Dir, LOW);
      for (int y = 0;y<Steps; y++) 
      {
        digitalWrite(Y_Step,HIGH);
        delayMicroseconds(500);
        digitalWrite(Y_Step,LOW); 
        delayMicroseconds(500);
      }
      break;
      case Home:
      digitalWrite(Y_Dir, HIGH);
      for (int y = 0;y<Steps; y++) 
      {
        digitalWrite(Y_Step,HIGH);
        delayMicroseconds(500);
        digitalWrite(Y_Step,LOW); 
        delayMicroseconds(500);
      }
      break;
    } 
    break;
    case Z_Axis:
    switch(Direction)
    {
      case Away:
      digitalWrite(Z_Dir, LOW);
      for (int z = 0;z<Steps; z++) 
      {
        digitalWrite(Z_Step,HIGH);
        delayMicroseconds(500);
        digitalWrite(Z_Step,LOW); 
        delayMicroseconds(500);
      }
      break;
      case Home:
      digitalWrite(Z_Dir, HIGH);
      for (int z = 0;z<Steps; z++) 
      {
        digitalWrite(Z_Step,HIGH);
        delayMicroseconds(500);
        digitalWrite(Z_Step,LOW); 
        delayMicroseconds(500);
      }
      break;
    } 
    break;   
  }
}

/***************************************************************************************************************************
****************************************************************************************************************************/

// our customized function that stops any motor in the project
void H_Stepper_Stall(int Motor)
{
  switch(Motor)
  {
    case X_Axis:
    digitalWrite(X_Step,LOW);
    delayMicroseconds(500);
    digitalWrite(X_Step,LOW); 
    delayMicroseconds(500);
    break;  
    case Y_Axis:
    digitalWrite(Y_Step,LOW);
    delayMicroseconds(500);
    digitalWrite(Y_Step,LOW); 
    delayMicroseconds(500);
    break;
    case Z_Axis:
    digitalWrite(Z_Step,LOW);
    delayMicroseconds(500);
    digitalWrite(Z_Step,LOW); 
    delayMicroseconds(500);
    break;            
  }
}