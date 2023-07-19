#include "Arduino_FreeRTOS.h"
#include "semphr.h"
#include "ezButton.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>

/*********************************************************************************************************************************
******************************************     Pins Declaration      *************************************************************                                          
**********************************************************************************************************************************/

ezButton Feeding_Switch(11);    // create ezButton object that attach to pin 11 to check that product reached end position of feeding
ezButton Product_Switch(10);    // create ezButton object that attach to pin 10 to check that product is available in magazine
#define Feeding_DCV 6           // attach pin 5 to control the relay output to the 5/2 DCV controlling the Pneumatic Piston Cylinder
#define Suction_DCV 7           // attach pin 6 to control the relay output to the 3/2 DCV controlling suction
#define Lower_Receiver 13       // attach pin 13 to read value of the lower laser receiver
#define Upper_Receiver 12       // attach pin 12 to read value of the upper laser receiver
#define receiver 8              // attach pin 8 to detect remote control signals to fire the line
#define Red_Led A0
#define Green_Led A1
#define ON 0x4EA240AE           // Save Hex value of the ON press
LiquidCrystal_I2C Control_LCD(0x27,16,2);
IRrecv Receiver(receiver);
decode_results Signal;
int Trigger = 0;

/*********************************************************************************************************************************
******************************************       Semaphores        ***************************************************************                                          
**********************************************************************************************************************************/

SemaphoreHandle_t Feeding_extend;
SemaphoreHandle_t Feeding_retract;
SemaphoreHandle_t Start_Differentiation;

/*********************************************************************************************************************************
*****************************************      Setup Function      ***************************************************************                                          
**********************************************************************************************************************************/

// the setup function runs once when you press reset or power the board
void setup() {  
  pinMode(Feeding_DCV, OUTPUT);                             // DCV Declared as output
  pinMode(Suction_DCV, OUTPUT);                             // DCV Declared as output
  pinMode(Lower_Receiver, INPUT);                           // Receiver Declared as input
  pinMode(Upper_Receiver, INPUT);                           // Receiver Declared as input
  Feeding_Switch.setDebounceTime(50);                       // set debounce time to 50 milliseconds
  Serial.begin(9600);
  Receiver.enableIRIn();                                    // enable taking signals from the IR Receiver 
  Control_LCD.begin();                                      // initialize the LCD
  Control_LCD.backlight();                                  // Turn on the backlight
  Control_LCD.print("Welcome!");
  Control_LCD.setCursor(0, 1);
  Control_LCD.print("Team 2 Project");
  Feeding_extend = xSemaphoreCreateBinary();
  Feeding_retract = xSemaphoreCreateBinary();
  Start_Differentiation = xSemaphoreCreateBinary();  
  xTaskCreate(Feeding_Sensing, "Task1", 100, NULL, 1, NULL);
  xTaskCreate(Feeding_Forward, "Task2", 100, NULL, 1, NULL);
  //xTaskCreate(Feeding_Back, "Task3", 100, NULL, 1, NULL);
  //xTaskCreate(Product_Differentiation, "Task4", 100, NULL, 1, NULL);  
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
    Serial.println("loop started");        
    Product_Switch.loop();                                    // MUST call the loop() function first
    Feeding_Switch.loop();
    int endposition = Feeding_Switch.getState();
    int available = Product_Switch.getState();
    if (Feeding_Switch.isPressed())                               // reached the end position
      {
        xSemaphoreGive(Start_Differentiation);
      }
    else if((available == LOW) && (endposition == HIGH))      // product available at the magazine and nothing at the end position
      {
        xSemaphoreGive(Feeding_extend);
      }
  }
}

void Feeding_Forward (void* ptr)
{
  while(1){
    xSemaphoreTake(Feeding_extend,portMAX_DELAY);
    Serial.println("Took Extension Semaphore");
    digitalWrite(Feeding_DCV,HIGH);
  }
}

/*void Feeding_Back (void* ptr)
{
  while(1){
    xSemaphoreTake(Feeding_retract,portMAX_DELAY);
    Serial.println("Took Retraction Semaphore");
    digitalWrite(Feeding_DCV,LOW);  
  }
}

void Product_Differentiation (void* ptr)
{
  while(1){
    xSemaphoreTake(Start_Differentiation,portMAX_DELAY);
    Serial.println("Took Differentiation Semaphore");
    Serial.println("upper sensor");
    Serial.println(digitalRead(Upper_Receiver));
    Serial.println("Lower sensor");
    Serial.println(digitalRead(Lower_Receiver));                
  }  
}*/
