/*********************************************************************************************************************************
*
* FILE-NAME : Arduino_Nano_Code
*
* AUTHOR : TEAM 2
*
* DESCRIPTION : Arduino Nano code that control all operations and communicate with 2nd MCU using UART Protocols 
*               to print on LCD connected on 2nd MCU the count of products stored in each storage location
*               and the 2nd MCU act as an bridge for the communication between the 1st and 3rd MCUs to that the 3rd MCU
*               print on an LCD the currently operating process and control the lights that notifies about the operation being done
*
***********************************************************************************************************************************/

#include "Arduino_FreeRTOS.h"
#include "semphr.h"
#include "ezButton.h"

char Homing_M[5]    = "10000"; //String data
char Magazine_M[5]  = "20000";
char Feeding_M[5]   = "30000";
char Sorting_M[5]   = "40000";
char Small_M[5]     = "50000";
char Large_M[5]     = "60000";

/*********************************************************************************************************************************
******************************************     Pins Declaration      *************************************************************
******************************************    Hardware Definition    *************************************************************                                          
**********************************************************************************************************************************/

/****************************                    Robotic Arm                                  ************************************/

ezButton X_Switch(9);               // create ezButton object that attach to pin 9 to act as limit switch for x-axis
ezButton Y_Switch(A0);              // create ezButton object that attach to pin A0 to act as limit switch for y-axis
ezButton Z_Switch(A1);              // create ezButton object that attach to pin A1 to act as limit switch for Z-axis

// declaration of the pins controlling the direction and step of each motor (According to data sheet of CNC NANO SHIELD)
#define X_Dir   2
#define Y_Dir   3
#define Z_Dir   4
#define X_Step  5
#define Y_Step  6
#define Z_Step  7

// defining the storing patch and locations in the production line
const int Products_Types  = 2;         // either Large/Small
const int Products_Count  = 4;         // 4 products of each type

// locations, as stepper motor steps count, defined from top left to bottom right
#define First_Row     29550
#define Second_Row    23300
#define First_Col     8450
#define Second_Col    2400
#define Third_Col     3750
#define Fourth_Col    10000
#define Large_Z       5850
#define Small_Z       6850
int X_Position[Products_Types][Products_Count]= {{First_Row+50,First_Row+50 ,Second_Row ,Second_Row},{First_Row   ,First_Row  ,Second_Row+100 ,Second_Row+100 }};  // saved X-locations
int Y_Position[Products_Types][Products_Count]= {{First_Col   ,Second_Col   ,First_Col  ,Second_Col},{Fourth_Col  ,Third_Col  ,Fourth_Col     ,Third_Col      }};  // saved Y-locations
enum Product_Type {Small,Large};
enum Product_No   {One,Two,Three,Four};

/****************************                         Feeding & Suction                                 ************************************/

ezButton Feeding_Switch(A3);          // create ezButton object that attach to pin A3 to check that product reached end position of feeding
ezButton Product_Switch(A2);          // create ezButton object that attach to pin A2 to check that product is available in magazine
#define Suction_DCV 11                // attach the suction DCV pin to pin 11 in shield 
#define Feeding_DCV 10                // attach the Feeding DCV pin to pin 10 in shield
#define Upper_Receiver A6             // attach pin A6 to read value of the upper laser receiver

enum Feeder_State{OFF,ON};
int Large_Count = 0;
int Small_Count = 0;

/*********************************************************************************************************************************
***************************************     Functions Declaration      ***********************************************************                                         
**********************************************************************************************************************************/

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
  // All Hardware Pins on this microcontroller are outputs except the laser receiver  
  pinMode(X_Step        , OUTPUT);
  pinMode(X_Dir         , OUTPUT);
  pinMode(Y_Step        , OUTPUT);
  pinMode(Y_Dir         , OUTPUT);
  pinMode(Z_Step        , OUTPUT);
  pinMode(Z_Dir         , OUTPUT);    
  pinMode(Suction_DCV   , OUTPUT); 
  pinMode(Feeding_DCV   , OUTPUT);                                 
  pinMode(Upper_Receiver, INPUT );                              // Receiver Declared as input

  // to ensure everything is off on system powering ON
  digitalWrite(Suction_DCV, OFF);
  digitalWrite(Feeding_DCV, OFF);
  
  Product_Switch.setDebounceTime(50);                           // set debounce time to 50 milliseconds
  Feeding_Switch.setDebounceTime(50);                           // set debounce time to 50 milliseconds
     
  X_Switch.setDebounceTime(50);                                 // set debounce time to 50 milliseconds
  Y_Switch.setDebounceTime(50);                                 // set debounce time to 50 milliseconds
  Z_Switch.setDebounceTime(50);                                 // set debounce time to 50 milliseconds

  // Serial initialize
  Serial.begin(9600);

  // Semaphores creation
  Finished_Homing = xSemaphoreCreateBinary();
  At_Product      = xSemaphoreCreateBinary();
  Feeding_extend  = xSemaphoreCreateBinary();
  Product_Fed     = xSemaphoreCreateBinary();
  Sorted_Large    = xSemaphoreCreateBinary();
  Sorted_Small    = xSemaphoreCreateBinary();

  xTaskCreate(Homing          , "Task1", 100, NULL, 1, NULL);
  xTaskCreate(GoToProduct     , "Task2", 100, NULL, 2, NULL);
  xTaskCreate(Feeding_Sensing , "Task3", 100, NULL, 3, NULL);
  xTaskCreate(Feeding         , "Task4", 100, NULL, 4, NULL);
  xTaskCreate(Sorting         , "Task5", 100, NULL, 5, NULL);
  xTaskCreate(Small_Storing   , "Task6", 100, NULL, 6, NULL);
  xTaskCreate(Large_Storing   , "Task7", 100, NULL, 6, NULL);
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
    
    static int Homing_Counter = 0;                        // counter to check that an axis has homed
    X_Switch.loop();                                      // MUST call the loop() function first
    Y_Switch.loop();                                      // MUST call the loop() function first
    Z_Switch.loop();                                      // MUST call the loop() function first    
    switch(Homing_Counter)
    {
      case 0:
      // Z-Axis Homing Loop (Done First to avoid Hitting objects while moving)
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
      // Y-Axis Homing Loop (Second thing done to avoid hitting magazine while moving)
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
      }
      else
      {      
        H_Stepper_Actuate(X_Axis, Home, 2);
      }
      break;  

      case 3:
      xSemaphoreGive(Finished_Homing); 
      vTaskDelete(NULL);
      break;    
    }    
  }
}

/************************************************************************************************************************************************
************************************************************************************************************************************************/

// Function that aims to stand above the product location in the magazine after being Homed at the power up or reset
void GoToProduct (void* ptr)
{
  while(1)
  {
    xSemaphoreTake(Finished_Homing,portMAX_DELAY);
    Serial.write(Magazine_M,5); //Write the serial data  
    delay(1000);  

    H_Stepper_Actuate(X_Axis, Away, 1700);
    H_Stepper_Actuate(Y_Axis, Away, 11500);
    
    xSemaphoreGive(At_Product);
    vTaskDelete(NULL);
  }
}

/**********************************************************************************************************************************************
***********************************************************************************************************************************************/

// Function that checks if there's a product in the magazine or not and check if feeding is appropriate at the moment
void Feeding_Sensing (void* ptr)
{
  while(1)
  {
    xSemaphoreTake(At_Product,portMAX_DELAY);
    Product_Switch.loop();                                    // MUST call the loop() function first
    Feeding_Switch.loop();

    int endposition = Feeding_Switch.getState();
    int available   = Product_Switch.getState();

    if((available == LOW) && (endposition == HIGH))           // product available at the magazine and nothing at the end position
    {
      xSemaphoreGive(Feeding_extend);
    }
  }
}

/***********************************************************************************************************************************************
************************************************************************************************************************************************/

// Function that feed a product into the sorting terminal
void Feeding (void* ptr)
{
  while(1)
  {

    xSemaphoreTake(Feeding_extend, portMAX_DELAY);        
    Serial.write(Feeding_M,5); //Write the serial data  

    digitalWrite(Feeding_DCV,ON);
    delay(1000);
    digitalWrite(Feeding_DCV,OFF);
    xSemaphoreGive(Product_Fed);
    
  }
}

/***********************************************************************************************************************************************
************************************************************************************************************************************************/

// Function that Sort the product available at sorting channel according to type (Large/Small)
void Sort (void* ptr)
{
  while(1)  
  {
    xSemaphoreTake(Product_Fed,portMAX_DELAY);
    Serial.write(Sorting_M,5); //Write the serial data 
    delay(1000);    
    if(analogRead(Upper_Receiver)==0)
    {
      Small_Count++;
      xSemaphoreGive(Sorted_Small);  
    }  
    else 
    {
      Large_Count++;   
      xSemaphoreGive(Sorted_Large);     
    } 
  }
}

/***********************************************************************************************************************************************
************************************************************************************************************************************************/

// Function that store the large products according to the number of the product according to the past processes
void Large_Storing (void* ptr)
{
  while(1)  
  {
    xSemaphoreTake(Sorted_Large,portMAX_DELAY);
    Serial.write(Large_M,5); //Write the serial data 
    delay(1000);    
    H_Product_Store(Large, Large_Count);
    xSemaphoreGive(At_Product);     
  }
}

/***********************************************************************************************************************************************
************************************************************************************************************************************************/

// Function that store the Small products according to the number of the product according to the past processes
void Small_Storing (void* ptr)
{
  while(1)  
  {
    xSemaphoreTake(Sorted_Large,portMAX_DELAY);
    Serial.write(Small_M,5); //Write the serial data 
    delay(1000);    
    H_Product_Store(Small, Small_Count);
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

/***************************************************************************************************************************
****************************************************************************************************************************/

// our customized function for storing any type of product according to count of specific type
// the commenting of the first case applies to all cases in the function but varies in the positions to be gone or stroke of Z-Axis for suction mechanism

void H_Product_Store(bool Product_Type, int Product_Count)
{
  switch(Product_Type)
  {
    case Large:
      switch(Product_Count)
      {
        case 1:
        H_Stepper_Actuate(Z_Axis,Away,Large_Z);                   // Goes down
        delay(500);                                               // waits half a second
        digitalWrite(Suction_DCV,ON);                             // turn on the suction
        delay(500);                                               // waits half a second
        H_Stepper_Actuate(Z_Axis,Home,Large_Z);                   // goes up
        
        H_Stepper_Actuate(X_Axis,Away,X_Position[Large][One]);    // goes to the row of the product to be stored
        H_Stepper_Actuate(Y_Axis,Home,Y_Position[Large][One]);    // goes to the column of the product to be stored
        
        H_Stepper_Actuate(Z_Axis,Away,Large_Z);                   // goes down
        delay(500);                                               // waits half a second
        digitalWrite(Suction_DCV,OFF);                            // turn off the suction
        delay(500);                                               // waits half a second
        H_Stepper_Actuate(Z_Axis,Home,Large_Z);                   // goes up

        H_Stepper_Actuate(Y_Axis,Away,Y_Position[Large][One]);    // return in y axis first
        H_Stepper_Actuate(X_Axis,Home,X_Position[Large][One]);    // then returns in the X-Axis to be standing above the product again
        break; 
        case 2: 
        H_Stepper_Actuate(Z_Axis,Away,Large_Z);
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Large_Z); 
        
        H_Stepper_Actuate(X_Axis,Away,X_Position[Large][Two]); 
        H_Stepper_Actuate(Y_Axis,Home,Y_Position[Large][Two]);  
        
        H_Stepper_Actuate(Z_Axis,Away,Large_Z);
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Large_Z); 

        H_Stepper_Actuate(Y_Axis,Away,Y_Position[Large][Two]); 
        H_Stepper_Actuate(X_Axis,Home,X_Position[Large][Two]); 
        break;
        case 3: 
        H_Stepper_Actuate(Z_Axis,Away,Large_Z);
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Large_Z); 
        
        H_Stepper_Actuate(X_Axis,Away,X_Position[Large][Three]); 
        H_Stepper_Actuate(Y_Axis,Home,Y_Position[Large][Three]);  
        
        H_Stepper_Actuate(Z_Axis,Away,Large_Z);
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Large_Z);  

        H_Stepper_Actuate(Y_Axis,Away,Y_Position[Large][Three]); 
        H_Stepper_Actuate(X_Axis,Home,X_Position[Large][Three]); 
        break;
        case 4: 
        H_Stepper_Actuate(Z_Axis,Away,Large_Z);
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Large_Z); 
        
        H_Stepper_Actuate(X_Axis,Away,X_Position[Large][Four]); 
        H_Stepper_Actuate(Y_Axis,Home,Y_Position[Large][Four]);  
        
        H_Stepper_Actuate(Z_Axis,Away,Large_Z);
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Large_Z);  

        H_Stepper_Actuate(Y_Axis,Away,Y_Position[Large][Four]); 
        H_Stepper_Actuate(X_Axis,Home,X_Position[Large][Four]);
        break;     
      }   
    break;      
    case Small:
      switch(Product_Count)
      {
        case 1: 
        H_Stepper_Actuate(Z_Axis,Away,Small_Z);
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Small_Z); 
        
        H_Stepper_Actuate(X_Axis,Away,X_Position[Small][One]); 
        H_Stepper_Actuate(Y_Axis,Away,Y_Position[Small][One]);  
        
        H_Stepper_Actuate(Z_Axis,Away,Small_Z);
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Small_Z);  

        H_Stepper_Actuate(Y_Axis,Home,Y_Position[Small][One]);  
        H_Stepper_Actuate(X_Axis,Home,X_Position[Small][One]); 
        break;  
        case 2: 
        H_Stepper_Actuate(Z_Axis,Away,Small_Z);
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Small_Z); 
        
        H_Stepper_Actuate(X_Axis,Away,X_Position[Small][Two]); 
        H_Stepper_Actuate(Y_Axis,Away,Y_Position[Small][Two]);  
        
        H_Stepper_Actuate(Z_Axis,Away,Small_Z);
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Small_Z); 
        
        H_Stepper_Actuate(Y_Axis,Home,Y_Position[Small][Two]);  
        H_Stepper_Actuate(X_Axis,Home,X_Position[Small][Two]);       
        break;
        case 3: 
        H_Stepper_Actuate(Z_Axis,Away,Small_Z);
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Small_Z); 
        
        H_Stepper_Actuate(X_Axis,Away,X_Position[Small][Three]); 
        H_Stepper_Actuate(Y_Axis,Away,Y_Position[Small][Three]);  
        
        H_Stepper_Actuate(Z_Axis,Away,Small_Z);
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Small_Z); 
        
        H_Stepper_Actuate(Y_Axis,Home,Y_Position[Small][Three]);  
        H_Stepper_Actuate(X_Axis,Home,X_Position[Small][Three]);       
        break;
        case 4: 
        H_Stepper_Actuate(Z_Axis,Away,Small_Z);
        delay(500);
        digitalWrite(Suction_DCV,ON); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Small_Z); 
        
        H_Stepper_Actuate(X_Axis,Away,X_Position[Small][Four]); 
        H_Stepper_Actuate(Y_Axis,Away,Y_Position[Small][Four]); 
        
        H_Stepper_Actuate(Z_Axis,Away,Small_Z);
        delay(500);
        digitalWrite(Suction_DCV,OFF); 
        delay(500);
        H_Stepper_Actuate(Z_Axis,Home,Small_Z); 
        
        H_Stepper_Actuate(Y_Axis,Home,Y_Position[Small][Four]);  
        H_Stepper_Actuate(X_Axis,Home,X_Position[Small][Four]);             
        break;
      }
    break;    
  }  
}