/*********************************************************************************************************************************
*
* FILE-NAME : ECU1
*
* AUTHOR : TEAM 2
*
* DESCRIPTION : Arduino Uno code that control the feeding and sorting terminals
*
***********************************************************************************************************************************/

#include "Arduino_FreeRTOS.h"
#include "semphr.h"
#include "ezButton.h"

/*********************************************************************************************************************************
******************************************     Pins Declaration      *************************************************************                                          
**********************************************************************************************************************************/

ezButton Product_Switch(A2);    // create ezButton object that attach to pin 10 to check that product is available in magazine
#define Feeding_DCV 10           // attach pin 5 to control the relay output to the 5/2 DCV controlling the Pneumatic Piston Cylinder


/*********************************************************************************************************************************
******************************************       Semaphores        ***************************************************************                                          
**********************************************************************************************************************************/

SemaphoreHandle_t Feeding_extend;

/*********************************************************************************************************************************
*****************************************      Setup Function      ***************************************************************                                          
**********************************************************************************************************************************/

// the setup function runs once when you press reset or power the board
void setup() {  
  pinMode(Feeding_DCV, OUTPUT);                             // DCV Declared as output
  Product_Switch.setDebounceTime(50);  
  Serial.begin(9600);
  Feeding_extend = xSemaphoreCreateBinary();
  xTaskCreate(Feeding_Sensing, "Task1", 100, NULL, 1, NULL);
  //xTaskCreate(Feeding_Forward, "Task2", 100, NULL, 2, NULL);
  // xTaskCreate(Feeding_Backward, "Task3", 100, NULL, 2, NULL);
  // xTaskCreate(Product_Differentiation, "Task4", 100, NULL, 3, NULL);
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

void Feeding_Sensing (void* ptr)
{
  while(1)
  {
    digitalWrite(Feeding_DCV,1);    
    Serial.println("loop started");        
    Product_Switch.loop();                                    // MUST call the loop() function first
    if(!Product_Switch.getStateRaw())
    {    
      Serial.println("feeding now");
      digitalWrite(Feeding_DCV,0);
      delay(1000);
      digitalWrite(Feeding_DCV,1);
    }
  }
}

