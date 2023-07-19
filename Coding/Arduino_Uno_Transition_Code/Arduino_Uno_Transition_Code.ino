/*********************************************************************************************************************************
*
* FILE-NAME : Arduino_UNO_Transition_Code
*
* AUTHOR : TEAM 2
*
* DESCRIPTION : Arduino Uno Code that targets as a bridge for the communication between the 1st MCU and the 3rd one 
*               it communicates with 1st MCU using UART protocol and with the 3rd MCU using SPI-Ethernet communication
*               In addition, according to what received from the 1st MCU this MCU displays count of products stored on an LCD
*
***********************************************************************************************************************************/

#include <EtherCard.h>
#include <Arduino_FreeRTOS.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>


/*********************************************************************************************************************************
******************************************     Pins Declaration      *************************************************************
******************************************    Hardware Definition    *************************************************************                                          
**********************************************************************************************************************************/

int small = 0;                                              // Small Products Counter
int large =0;                                               // Large Products Counter
/* Addresses */
// mac address
static byte mymac[] = { 0x1A,0x2B,0x3C,0x4D,0x5E,0x6F };
// ethernet interface ip address
static byte myip[] = { 192, 168, 1, 2 };
// gateway ip address
static byte gwip[] = { 192, 168, 1, 1 };
// subnet mask
static byte mask[] = { 255, 255, 255, 0 };
// destination ip-address
static byte dstIp[] = { 192, 168, 1, 3 };
// ports
const int dstPort PROGMEM = 1234;
const int srcPort PROGMEM = 4321;

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

char Mymessage[10]; //Initialized variable to store recieved data

/* Buffer and timer */
byte Ethernet::buffer[700];
static uint32_t timer;  // dummy incrementor that'll later be used for delays


/*********************************************************************************************************************************
***************************************     Functions Declaration      ***********************************************************                                         
**********************************************************************************************************************************/

/* Ethernet peripheral initialization function */
void initializeEthernet()
{
  // Begin Ethernet communication
  if (ether.begin(sizeof Ethernet::buffer, mymac, SS) == 0)
  {
    Serial.println("Failed to access Ethernet controller");
    return;
  }
    
  // Setup static IP address
  ether.staticSetup(myip, gwip, 0, mask);

  // Log configuration
  Serial.println(F("\n[Sender]"));
  ether.printIp("IP:  ", ether.myip);
  ether.printIp("GW:  ", ether.gwip);
}

/* FreeRTOS Ethernet Sending Task */
void TaskSendEthernet( void* pvParameters )
{
  while(1)
  {
    Serial.readBytes(Mymessage,5); //Read the serial data and store in var
    ether.sendUdp(Mymessage, sizeof(Mymessage), srcPort, dstIp, dstPort);
    if(Mymessage[0] == "5")
    {
      small++;
      lcd.setCursor(2,1);
      lcd.print(small);
    }
    else if (Mymessage[0] == "6")
    {
      large++;
      lcd.setCursor(13,1);
      lcd.print(large);      
    }
  }
}

/*********************************************************************************************************************************
*****************************************      Setup Function      ***************************************************************                                          
**********************************************************************************************************************************/

void setup () {
  // Initialize Serial
  Serial.begin(9600);

  // Initialize Ethernet
  initializeEthernet();

  lcd.begin();
	lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Welcome!!");
  lcd.setCursor(0, 1);
  lcd.print("Team 2 Project");
  delay(2000);
  lcd.clear();
  lcd.print("Small     Large");
  
  // initialize RTOS task
  xTaskCreate(
    TaskSendEthernet
    ,  "SendEthernet"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle

}

/*********************************************************************************************************************************
*****************************************      LOOP Function      ****************************************************************                                          
**********************************************************************************************************************************/

void loop () 
{
  // empty loop
}
