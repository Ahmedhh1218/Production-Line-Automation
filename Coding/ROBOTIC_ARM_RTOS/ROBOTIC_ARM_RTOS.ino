#include "Arduino_FreeRTOS.h"
#include "semphr.h"
#include "ezButton.h"
#include "AccelStepper.h"

ezButton X_Switch(9);  // create ezButton object that attach to pin 4;
ezButton Y_Switch(10);
ezButton Z_Switch(11);
#define X_Dir 2
#define Y_Dir 3
#define Z_Dir 4
#define X_Step 5
#define Y_Step 6
#define Z_Step 7

SemaphoreHandle_t X_Actuate;
SemaphoreHandle_t Y_Actuate;
SemaphoreHandle_t Z_Actuate;
SemaphoreHandle_t X_Stop;
SemaphoreHandle_t Y_Stop;
SemaphoreHandle_t Z_Stop;
SemaphoreHandle_t LCD_Print_Home;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(X_Step,OUTPUT);
  pinMode(X_Dir,OUTPUT);
  pinMode(Y_Step,OUTPUT);
  pinMode(Y_Dir,OUTPUT);
  pinMode(Z_Step,OUTPUT);
  pinMode(Z_Dir,OUTPUT);      
  X_Switch.setDebounceTime(50); // set debounce time to 50 milliseconds
  // Y_Switch.setDebounceTime(50); // set debounce time to 50 milliseconds
  // Z_Switch.setDebounceTime(50); // set debounce time to 50 milliseconds
  Serial.begin(9600);
  X_Actuate = xSemaphoreCreateBinary();
  // Y_Actuate = xSemaphoreCreateBinary();
  // Z_Actuate = xSemaphoreCreateBinary();
  // X_Stop = xSemaphoreCreateBinary();
  // Y_Stop = xSemaphoreCreateBinary();
  // Z_Stop = xSemaphoreCreateBinary();
  LCD_Print_Home = xSemaphoreCreateBinary();
  xTaskCreate(Homing_Sensing, "Task1", 100, NULL, 1, NULL);
  xTaskCreate(X_Actuation, "Task2", 100, NULL, 2, NULL);
  // xTaskCreate(Y_Actuation, "Task3", 100, NULL, 3, NULL);
  // xTaskCreate(Z_Actuation, "Task4", 100, NULL, 4, NULL);
  // xTaskCreate(X_Stopping, "Task5", 100, NULL, 3, NULL);
  // xTaskCreate(Y_Stopping, "Task6", 100, NULL, 3, NULL);
  // xTaskCreate(Z_Stopping, "Task7", 100, NULL, 3, NULL);
}

// the loop function runs over and over again forever
void loop() {
  
}

void Homing_Sensing (void* ptr)
{
  while(1)
  {
    X_Switch.loop(); // MUST call the loop() function first
    int X_state = X_Switch.getState();
    // int Y_state = Y_Switch.getState();
    // int Z_state = Z_Switch.getState();
    if (X_Switch.isPressed())
    {
      digitalWrite(X_Step,LOW);
      delayMicroseconds(500);
      digitalWrite(X_Step,LOW); 
      delayMicroseconds(500);
    }
    else if(X_state == HIGH)
    {
      xSemaphoreGive(X_Actuate);
    }
    // if (Y_Switch.isPressed())
    // {
    //   digitalWrite(Y_Step,LOW);
    //   delayMicroseconds(500);
    //   digitalWrite(Y_Step,LOW); 
    //   delayMicroseconds(500);
    // }
    // else if(Y_state == HIGH)
    // {
    //   xSemaphoreGive(Y_Actuate);
    // }
    // if (Z_Switch.isPressed())
    // {
    //   digitalWrite(Z_Step,LOW);
    //   delayMicroseconds(500);
    //   digitalWrite(Z_Step,LOW); 
    //   delayMicroseconds(500);
    // }
    // else if(Z_state == HIGH)
    // {
    //   xSemaphoreGive(Z_Actuate);
    // }    
  }
}

//  void Y_Sensing (void* ptr)
//  {
//   while(1)
//   {
//     Y_Switch.loop(); // MUST call the loop() function first
//     int Ystate = Y_Switch.getState();
//     Serial.println(Ystate);
//     if (Y_Switch.isPressed())
//     {
//       xSemaphoreGive(Z_Actuate);
//     }
//     else if(Ystate == HIGH)
//     {
//       xSemaphoreGive(Y_Actuate);
//     }
//   }
// }

// void Z_Sensing (void* ptr)
// {
//   while(1)
//   {
//     Z_Switch.loop(); // MUST call the loop() function first
//     int Zstate = Z_Switch.getState();
//     Serial.println(Zstate);
//     if (Z_Switch.isPressed())
//     {
//       xSemaphoreGive(LCD_Print_Home);
//     }
//     else if(Zstate == HIGH)
//     {
//       xSemaphoreGive(Z_Actuate);
//     }
//   }
// }

void X_Actuation (void* ptr)
{
  while(1)
  {
    xSemaphoreTake(X_Actuate,portMAX_DELAY);
    digitalWrite(X_Dir, HIGH); // set direction, HIGH for clockwise, LOW for anticlockwise
    for(int x = 0; x<100; x++) {
    digitalWrite(X_Step,HIGH);
    delayMicroseconds(500);
    digitalWrite(X_Step,LOW); 
    delayMicroseconds(500);
  }
  }  
}

// void Y_Actuation (void* ptr)
// {
//   while(1)
//   {
//     xSemaphoreTake(Y_Actuate,portMAX_DELAY);
//     digitalWrite(Y_Dir, HIGH);
//     for(int x = 0; x<100; x++){
//     digitalWrite(Y_Step,HIGH);
//     delayMicroseconds(500);
//     digitalWrite(Y_Step,LOW); 
//     delayMicroseconds(500);
//   }
//   }
// }

// void Z_Actuation (void* ptr)
// {
//   while(1)
//   {   
//     xSemaphoreTake(Z_Actuate,portMAX_DELAY);
//     digitalWrite(Z_Dir, LOW); 
//     for(int x = 0; x<100; x++){
//     digitalWrite(Z_Step,HIGH);
//     delayMicroseconds(500);
//     digitalWrite(Z_Step,LOW); 
//     delayMicroseconds(500);
//     }
//   }
// }

// void X_Stopping (void* ptr)
// {
//   while(1)
//   {
//     xSemaphoreTake(X_Stop,portMAX_DELAY);
//     digitalWrite(X_Dir, LOW); // set direction, HIGH for clockwise, LOW for anticlockwise
//     digitalWrite(X_Step,LOW);
//     delayMicroseconds(500);
//     digitalWrite(X_Step,LOW); 
//     delayMicroseconds(500);
//   }  
// }

// void Y_Stopping (void* ptr)
// {
//   while(1)
//   {
//     xSemaphoreTake(Y_Stop,portMAX_DELAY);
//     digitalWrite(Y_Dir, LOW);
//     digitalWrite(Y_Step,LOW);
//     delayMicroseconds(500);
//     digitalWrite(Y_Step,LOW); 
//     delayMicroseconds(500);
//   }
// }

// void Z_Stopping (void* ptr)
// {
//   while(1)
//   {   
//     xSemaphoreTake(Z_Stop,portMAX_DELAY);
//     digitalWrite(Z_Dir, HIGH); 
//     digitalWrite(Z_Step,LOW);
//     delayMicroseconds(500);
//     digitalWrite(Z_Step,LOW); 
//     delayMicroseconds(500);
//   }
// }