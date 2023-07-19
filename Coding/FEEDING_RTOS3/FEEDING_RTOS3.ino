/*********************************************************************************************************************************
*
* FILE-NAME : ECU1
*
* AUTHOR : TEAM 2
*
* DESCRIPTION : Arduino Uno code that control the feeding terminal
*
***********************************************************************************************************************************/

#include "Arduino_FreeRTOS.h"
#include "semphr.h"
#include "ezButton.h"

/*********************************************************************************************************************************
******************************************     Pins Declaration      *************************************************************                                          
**********************************************************************************************************************************/

ezButton Feeding_Switch(11);    // create ezButton object that attach to pin 11 to check that product reached end position of feeding
ezButton Product_Switch(10);    // create ezButton object that attach to pin 10 to check that product is available in magazine
#define Feeding_DCV 6           // attach pin 5 to control the relay output to the 5/2 DCV controlling the Pneumatic Piston Cylinder
enum Feeder_State{OFF,ON};

/*********************************************************************************************************************************
******************************************       Semaphores        ***************************************************************                                          
**********************************************************************************************************************************/

SemaphoreHandle_t Feeding_extend;
SemaphoreHandle_t Feeding_retract;

/*********************************************************************************************************************************
*****************************************      Setup Function      ***************************************************************                                          
**********************************************************************************************************************************/

// the setup function runs once when you press reset or power the board
void setup() {  
  pinMode(Feeding_DCV, OUTPUT);                             // DCV Declared as output
  Product_Switch.setDebounceTime(50);  
  Feeding_Switch.setDebounceTime(50);                       // set debounce time to 50 milliseconds
  Serial.begin(9600);
  Feeding_extend = xSemaphoreCreateBinary();
  Feeding_retract = xSemaphoreCreateBinary();
  xTaskCreate(Feeding_Sensing, "Task1", 100, NULL, 1, NULL);
  xTaskCreate(Feeding_Forward, "Task2", 100, NULL, 2, NULL);
  xTaskCreate(Feeding_Backward, "Task3", 100, NULL, 2, NULL);
}

/*********************************************************************************************************************************
*****************************************      LOOP Function      ****************************************************************                                          
**********************************************************************************************************************************/

// the loop function runs over and over again forever
void loop() {
  
}

/*********************************************************************************************************************************
********************************************     Customized RTOS Functions     ***************************************************                                        
**********************************************************************************************************************************/

// Function to check whether the pneumatic piston should extend or retract
void Feeding_Sensing (void* ptr)
{
  while(1)
  {
    Serial.println("loop started");        
    Product_Switch.loop();                                        // MUST call the loop() function first
    Feeding_Switch.loop();
    int endposition = Feeding_Switch.getState();
    int available = Product_Switch.getState();
    
    if((available == LOW) && (endposition == HIGH))               // product available at the magazine and nothing at the end position
      {
        delay(3000);        
        xSemaphoreGive(Feeding_extend);
      }
    else if (Feeding_Switch.isPressed() || endposition == LOW)
      {        
        xSemaphoreGive(Feeding_retract);
      }
  }
}

/***********************************************************************************************************************************************
************************************************************************************************************************************************/

// Function that brings the piston forward to feed products
void Feeding_Forward (void* ptr)
{
  while(1){
    xSemaphoreTake(Feeding_extend,portMAX_DELAY);
    Serial.println("Took Extension Semaphore");
    digitalWrite(Feeding_DCV,ON);
  }
}

/***********************************************************************************************************************************************
************************************************************************************************************************************************/

// Function that brings the piston backward after product is fed
void Feeding_Backward (void* ptr)
{
  while(1){
    xSemaphoreTake(Feeding_retract,portMAX_DELAY);
    Serial.println("Took Retraction Semaphore");
    digitalWrite(Feeding_DCV,OFF);
  }
}

