/*********************************************************************************************************************************
*
* FILE-NAME : Arduino_Nano_Code
*
* AUTHOR : TEAM 2
*
* DESCRIPTION : Arduino Nano code that control all operations and send to ECU1 to print on LCD the currently functioning process
*
***********************************************************************************************************************************/

#include "Arduino_FreeRTOS.h"
#include "semphr.h"
#include "ezButton.h"

/*********************************************************************************************************************************
******************************************     Pins Declaration      *************************************************************
******************************************    Hardware Definition    *************************************************************                                          
**********************************************************************************************************************************/

/****************************                    Robotic Arm                                  ************************************/

ezButton X_Switch(9);               // create ezButton object that attach to pin 9 to act as limit switch for x-axis
ezButton Y_Switch(A0);              // create ezButton object that attach to pin A0 to act as limit switch for y-axis
ezButton Z_Switch(A1);              // create ezButton object that attach to pin A1 to act as limit switch for Z-axis

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

/****************************                         Feeding & Suction                                 ************************************/

ezButton Feeding_Switch(A3);        // create ezButton object that attach to pin A3 to check that product reached end position of feeding
ezButton Product_Switch(A2);        // create ezButton object that attach to pin A2 to check that product is available in magazine
#define Suction_DCV 11               // attach the suction DCV pin to pin 1 in shield 
#define Feeding_DCV 10               // attach the Feeding DCV pin to pin 0 in shield
#define Upper_Receiver A6           // attach pin A6 to read value of the upper laser receiver

enum Feeder_State{ON,OFF};
int Large_Count =0;
int Small_Count =0;

/*********************************************************************************************************************************
***************************************     Functions Declaration      ***********************************************************                                         
**********************************************************************************************************************************/

enum Suction_State{Suction_ON, Suction_OFF};                      // Suction state either ON or OFF
enum Direction{Away, Home};                                       // 2 directions for each axis either towards the Home position or away from it
enum Motor{X_Axis, Y_Axis, Z_Axis};                               // 3 overall motors in the line
void H_Stepper_Actuate(int Motor, bool Direction, int Steps);     // our customized function for actuating any motor in any direction and with the required steps
void H_Stepper_Stall(int Motor);                                  // our customized function for stopping any motor
void H_Product_Store(bool Product_Type, int Product_Count);       // our customized function for storing any type of product according to count of specific type

/*********************************************************************************************************************************
******************************************       Semaphores        ***************************************************************                                          
**********************************************************************************************************************************/

SemaphoreHandle_t Finished_Homing;
SemaphoreHandle_t At_Product;
SemaphoreHandle_t Feeding_extend;
SemaphoreHandle_t Product_Fed;
SemaphoreHandle_t Sorted_Large;
SemaphoreHandle_t Sorted_Small;

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
  
  pinMode(Feeding_DCV, OUTPUT);                                 // DCV Declared as output
  pinMode(Upper_Receiver, INPUT);                               // Receiver Declared as input
  Product_Switch.setDebounceTime(50);  
  Feeding_Switch.setDebounceTime(50);                           // set debounce time to 50 milliseconds
     
  X_Switch.setDebounceTime(50);                                 // set debounce time to 50 milliseconds
  Y_Switch.setDebounceTime(50);                                 // set debounce time to 50 milliseconds
  Z_Switch.setDebounceTime(50);                                 // set debounce time to 50 milliseconds
  Serial.begin(9600);
  
  Finished_Homing = xSemaphoreCreateBinary();
  At_Product = xSemaphoreCreateBinary();
  Product_Fed = xSemaphoreCreateBinary();
  Sorted_Large = xSemaphoreCreateBinary();
  Sorted_Small = xSemaphoreCreateBinary();

  xTaskCreate(Homing, "Task1", 100, NULL, 1, NULL);
  xTaskCreate(GoToProduct,"Task2", 100, NULL, 2, NULL);
  xTaskCreate(Feeding, "Task3", 100, NULL, 3, NULL);
  xTaskCreate(Sort, "Task4", 100, NULL, 4, NULL);
  //xTaskCreate(Store_Large,"Task5",100,NULL,5,NULL);
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
    Serial.println("Homing");    
    static int Homing_Counter = 0;                        // counter to check that an axis has homed
    X_Switch.loop();                                      // MUST call the loop() function first
    Y_Switch.loop();                                      // MUST call the loop() function first
    Z_Switch.loop();                                      // MUST call the loop() function first
    digitalWrite(Feeding_DCV,OFF);                        // Close the Piston at the beginning due to normally open condition of the DCV
    switch(Homing_Counter)
    {
      case 0:
      // Z-Axis Homing Loop
      if(Z_Switch.isPressed() || !Z_Switch.getState())    // Reached Z-Axis limit switch
      {
        Homing_Counter ++;
        digitalWrite(Z_Step,LOW);
        delayMicroseconds(500);
        digitalWrite(Z_Step,LOW); 
        delayMicroseconds(500);    
      }
      else
      {       
       digitalWrite(Z_Dir, HOME);
        for (int x = 0;x<10; x++) 
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
      if(Y_Switch.isPressed() || !Y_Switch.getState())    // Reached Y-Axis limit switch
      {
        Homing_Counter ++;
        digitalWrite(Y_Step,LOW);
        delayMicroseconds(500);
        digitalWrite(Y_Step,LOW); 
        delayMicroseconds(500);    
      }
      else
      {       
        digitalWrite(Y_Dir, HOME);
        for (int x = 0;x<10; x++) 
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
      if(X_Switch.isPressed() || !X_Switch.getState())    // Reached X-Axis limit switch
      {
        Homing_Counter ++;
        digitalWrite(X_Step,LOW);
        delayMicroseconds(500);
        digitalWrite(X_Step,LOW); 
        delayMicroseconds(500);         
      }
      else
      {       
        digitalWrite(X_Dir, HOME);
        for (int x = 0;x<10; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
      }
      break;
      
      case 3:
      // Finished Homing 
      xSemaphoreGive(Finished_Homing);
      vTaskDelete(NULL);                                  // no need for homing function anymore        
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
    
    digitalWrite(X_Dir, Away);
    for (int x = 0;x<1800; x++) 
    {
      digitalWrite(X_Step,HIGH);
      delayMicroseconds(500);
      digitalWrite(X_Step,LOW); 
      delayMicroseconds(500);
    }   
    digitalWrite(Y_Dir, Away);
    for (int x = 0;x<11500; x++) 
    {
      digitalWrite(Y_Step,HIGH);
      delayMicroseconds(500);
      digitalWrite(Y_Step,LOW); 
      delayMicroseconds(500);
    }
    
    xSemaphoreGive(At_Product);
    vTaskDelete(NULL);
  }
}

/*****************************************************************************************************************************
******************************************************************************************************************************/

// Function that feed a product into the sorting terminal
void Feeding (void* ptr)
{
  while(1)
  {
    xSemaphoreTake(At_Product,portMAX_DELAY);    
    Serial.println("Took Feeding Semaphore");        
    Product_Switch.loop();                                        // MUST call the loop() function first
    Feeding_Switch.loop();
    delay(500);
    if(!Product_Switch.getStateRaw())
    {    
      Serial.println("feeding now");
      digitalWrite(Feeding_DCV,ON);
      delay(1000);
      digitalWrite(Feeding_DCV,OFF);
    }
     xSemaphoreGive(Product_Fed);
    
  }
}

/***********************************************************************************************************************************************
************************************************************************************************************************************************/

// Function that Sort the product available at sorting channel according to type
void Sort (void* ptr)
{
  while(1)  
  {
    xSemaphoreTake(Product_Fed,portMAX_DELAY);
    Serial.println("Received Sorting Semaphore");    
    if(analogRead(Upper_Receiver)==0)
    {
      Small_Count++;
      Serial.println("it's small");
      switch(Small_Count)
      {
        case 1: 
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        } 
        
        digitalWrite(X_Dir, Away);
        for (int x = 0;x<X_Position[Small][One]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(Y_Dir, Away);
        for (int x = 0;x<Y_Position[Small][One]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }

        digitalWrite(Y_Dir, Home);
        for (int x = 0;x<Y_Position[Small][One]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(X_Dir, Home);
        for (int x = 0;x<X_Position[Small][One]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        break;  
        case 2: 
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        } 
        
        digitalWrite(X_Dir, Away);
        for (int x = 0;x<X_Position[Small][Two]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(Y_Dir, Away);
        for (int x = 0;x<Y_Position[Small][Two]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }

        digitalWrite(Y_Dir, Home);
        for (int x = 0;x<Y_Position[Small][Two]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(X_Dir, Home);
        for (int x = 0;x<X_Position[Small][Two]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        break;  
        case 3: 
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        } 
        
        digitalWrite(X_Dir, Away);
        for (int x = 0;x<X_Position[Small][Three]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(Y_Dir, Away);
        for (int x = 0;x<Y_Position[Small][Three]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }

        digitalWrite(Y_Dir, Home);
        for (int x = 0;x<Y_Position[Small][Three]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(X_Dir, Home);
        for (int x = 0;x<X_Position[Small][Three]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        break;  
        case 4: 
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        } 
        
        digitalWrite(X_Dir, Away);
        for (int x = 0;x<X_Position[Small][Four]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(Y_Dir, Away);
        for (int x = 0;x<Y_Position[Small][Four]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Small_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }

        digitalWrite(Y_Dir, Home);
        for (int x = 0;x<Y_Position[Small][Four]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(X_Dir, Home);
        for (int x = 0;x<X_Position[Small][Four]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        break;  
      }
      xSemaphoreGive(At_Product);  
    }  
    else 
    {
      Large_Count++;
      Serial.println("it's large"); 
      switch(Large_Count)
      {
        case 1: 
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        } 
        
        digitalWrite(X_Dir, Away);
        for (int x = 0;x<X_Position[Large][One]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(Y_Dir, Home);
        for (int x = 0;x<Y_Position[Large][One]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }

        digitalWrite(Y_Dir, Away);
        for (int x = 0;x<Y_Position[Large][One]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(X_Dir, Home);
        for (int x = 0;x<X_Position[Large][One]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        break;  
        case 2: 
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        } 
        
        digitalWrite(X_Dir, Away);
        for (int x = 0;x<X_Position[Large][Two]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(Y_Dir, Home);
        for (int x = 0;x<Y_Position[Large][Two]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }

        digitalWrite(Y_Dir, Away);
        for (int x = 0;x<Y_Position[Large][Two]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(X_Dir, Home);
        for (int x = 0;x<X_Position[Large][Two]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        break;  
        case 3: 
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        } 
        
        digitalWrite(X_Dir, Away);
        for (int x = 0;x<X_Position[Large][Three]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(Y_Dir, Home);
        for (int x = 0;x<Y_Position[Large][Three]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }

        digitalWrite(Y_Dir, Away);
        for (int x = 0;x<Y_Position[Large][Three]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(X_Dir, Home);
        for (int x = 0;x<X_Position[Large][Three]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        break;  
        case 4: 
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        } 
        
        digitalWrite(X_Dir, Away);
        for (int x = 0;x<X_Position[Large][Four]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(Y_Dir, Home);
        for (int x = 0;x<Y_Position[Large][Four]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        
        digitalWrite(Z_Dir, Away);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        digitalWrite(Z_Dir, Home);
        for (int x = 0;x<Large_Z; x++) 
        {
          digitalWrite(Z_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Z_Step,LOW); 
          delayMicroseconds(500);
        }

        digitalWrite(Y_Dir, Away);
        for (int x = 0;x<Y_Position[Large][Four]; x++) 
        {
          digitalWrite(Y_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(Y_Step,LOW); 
          delayMicroseconds(500);
        }
        digitalWrite(X_Dir, Home);
        for (int x = 0;x<X_Position[Large][Four]; x++) 
        {
          digitalWrite(X_Step,HIGH);
          delayMicroseconds(500);
          digitalWrite(X_Step,LOW); 
          delayMicroseconds(500);
        }
        break;  
      }   
      xSemaphoreGive(At_Product);     
    } 
  }
}
