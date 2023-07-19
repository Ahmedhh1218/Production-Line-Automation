/*********************************************************************************************************************************
*
* FILE-NAME : Arduino_UNO_Terminal_Code
*
* AUTHOR : TEAM 2
*
* DESCRIPTION : Arduino Uno Code that targets to display the operating process on an LCD according to received data from MCU2 using 
*               SPI-Ethernet communication that was previously received from MCU1 to MCU2 using UART
*               in addition it lights different colors from a RGB LED to notify about the momentarily process
*
***********************************************************************************************************************************/

#include <EtherCard.h>
#include <IPAddress.h>
#include <LiquidCrystal.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>


/*********************************************************************************************************************************
******************************************     Pins Declaration      *************************************************************
******************************************    Hardware Definition    *************************************************************                                          
**********************************************************************************************************************************/

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd2(0x27, 16, 2);

#define Red 7
#define Green 6
#define Blue 5


/* Addresses */
// mac address
static byte mymac[] = { 0x70,0x69,0x69,0x2D,0x30,0x31 };
// ethernet interface ip address
static byte myip[] = { 192, 168, 1, 3 };
// gateway ip address
static byte gwip[] = { 192, 168, 1, 1 };
// subnet mask
static byte mask[] = { 255, 255, 255, 0 };
// ports
const int dstPort PROGMEM = 1234;

/* Buffer */
byte Ethernet::buffer[700];


/*********************************************************************************************************************************
***************************************     Functions Declaration      ***********************************************************                                         
**********************************************************************************************************************************/

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
  Serial.println(F("\n[Receiver]"));
  ether.printIp("IP:  ", ether.myip);
  ether.printIp("GW:  ", ether.gwip);
}

/* callback that prints received packets to the serial port */
void udpSerialPrint(uint16_t dest_port, uint8_t src_ip[IP_LEN], uint16_t src_port, const char *data, uint16_t len){
  IPAddress src(src_ip[0],src_ip[1],src_ip[2],src_ip[3]);

  Serial.print("dest_port: ");
  Serial.println(dest_port);
  Serial.print("src_port: ");
  Serial.println(src_port);

  Serial.print("src_port: ");
  ether.printIp(src_ip);
  Serial.println("\ndata: ");
  Serial.println(data);
  if( data[0] == '1'){
    lcd2.setCursor(0,0);
    lcd2.print("HOMING          ");
    digitalWrite(Red,HIGH);
    digitalWrite(Green,LOW);
    digitalWrite(Blue,LOW);
  }
  else if( data[0] == '2'){
    lcd2.setCursor(0,0);
    lcd2.print("GO TO PRODUCT   ");
    digitalWrite(Red,HIGH);
    digitalWrite(Green,LOW);
    digitalWrite(Blue,LOW);
  }
  else if( data[0] == '3'){
    lcd2.setCursor(0,0);
    lcd2.print("FEEDING         ");
    digitalWrite(Red,LOW);
    digitalWrite(Green,LOW);
    digitalWrite(Blue,HIGH);
  }
  else if( data[0] == '4'){
    lcd2.setCursor(0,0);
    lcd2.print("SORTING         ");
    digitalWrite(Red,HIGH);
    digitalWrite(Green,LOW);
    digitalWrite(Blue,HIGH);
  }
   else if( data[0] == '5'){
    lcd2.setCursor(0,0);
    lcd2.print("STORING         ");
    lcd2.setCursor(0,1);
    lcd2.print("SMALL PRODUCT   ");
    digitalWrite(Red,LOW);
    digitalWrite(Green,HIGH);
    digitalWrite(Blue,LOW);
  }
  else if( data[0] == '6'){
    lcd2.setCursor(0,0);
    lcd2.print("STORING         ");
    lcd2.setCursor(0,1);
    lcd2.print("LARGE PRODUCT   ");
    digitalWrite(Red,LOW);
    digitalWrite(Green,HIGH);
    digitalWrite(Blue,LOW);
  }
}

/*********************************************************************************************************************************
*****************************************      Setup Function      ***************************************************************                                          
**********************************************************************************************************************************/

void setup(){

  // Initialize Serial
  Serial.begin(9600);

  //lcd1.begin(16,2);
  //lcd1.clear();

  lcd2.begin();

	lcd2.backlight();

  pinMode(Red,OUTPUT);
  pinMode(Green,OUTPUT);
  pinMode(Blue,OUTPUT);
	
    
  // Initialize Ethernet
  initializeEthernet();
  
  //register udpSerialPrint() to destination port (callback function)
  ether.udpServerListenOnPort(&udpSerialPrint, dstPort);
  lcd2.setCursor(0, 0);
  lcd2.print("Welcome!!");
  lcd2.setCursor(0, 1);
  lcd2.print("Team 2 Project");
  delay(2000);
  lcd2.clear();
  }

/*********************************************************************************************************************************
*****************************************      LOOP Function      ****************************************************************                                          
**********************************************************************************************************************************/

void loop(){
  // receive packets, get ready for callbacks
  ether.packetLoop(ether.packetReceive());
}