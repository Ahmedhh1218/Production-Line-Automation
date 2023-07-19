#include <EtherCard.h>
#include <Arduino_FreeRTOS.h>

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

/* Data to send via Ethernet */
char textToSend[] = "payload101";

/* Buffer and timer */
byte Ethernet::buffer[700];
static uint32_t timer;  // dummy incrementor that'll later be used for delays

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
  for(;;)
  {
    if (millis() > timer) {
      timer = millis() + 100;

      ether.sendUdp(
        textToSend, 
        sizeof(textToSend), 
        srcPort, 
        dstIp, 
        dstPort );
    }
  }
}

void setup () {
  // Initialize Serial
  Serial.begin(57600);

  // Initialize Ethernet
  initializeEthernet();
  
  // initialize RTOS task
  xTaskCreate(
    TaskSendEthernet
    ,  "SendEthernet"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle
}

void loop () 
{
  // empty loop
}
