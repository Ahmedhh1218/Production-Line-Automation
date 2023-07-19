#include <EtherCard.h>
#include <Arduino_FreeRTOS.h>
const int x=0; //if x = 0 textToSend is send (Metal) if x = 1 textToSend2 is send (Non Metal)
/* Addresses */
static byte mymac[] = { 0x1A,0x2B,0x3C,0x4D,0x5E,0x6F };  // mac address
static byte myip[] = { 192, 168, 1, 2 }; // ethernet interface ip address
static byte gwip[] = { 192, 168, 1, 1 }; // gateway ip address Must be same in both Sender and reciver
static byte mask[] = { 255, 255, 255, 0 }; // subnet mask
static byte dstIp[] = { 192, 168, 1, 3 }; // destination ip-address (reciver ip address)
// ports
const int dstPort PROGMEM = 1234;
const int srcPort PROGMEM = 4321;

/* Data to send via Ethernet */
char Metal[] = "Metal"; //Metal text that will be send
char NonMetal[] = "Non_Metal";  //NonMetal text that will be send

/* Buffer and timer */
byte Ethernet::buffer[700];
static uint32_t timer;  // dummy incrementor that'll later be used for delays

/* Ethernet peripheral initialization function */
void initializeEthernet()
{
  // Begin Ethernet communication
  if (ether.begin(sizeof Ethernet::buffer, mymac, 53) == 0)  // if using Mega use value 53 if using uno use value 10
  { Serial.println("Failed to access Ethernet controller");
    return;}
    
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
      
// testing funtion if changed the global variable from the top from zero or one as to identify metal or non metal this should be your function
     if(x==0)
     {ether.sendUdp(Metal,sizeof(Metal),srcPort,dstIp,dstPort );}
     else if(x==1)
     {ether.sendUdp(NonMetal,sizeof(NonMetal),srcPort,dstIp,dstPort );}
    }
  }
}

void setup () {
  // Initialize Serial
  Serial.begin(57600);// can be 9600 normal

  // Initialize Ethernet
  initializeEthernet();
  
  // initialize RTOS task
  //Task to send ethernet
  xTaskCreate(
    TaskSendEthernet,"SendEthernet",128,NULL,2, NULL );
}

void loop () 
{
  // empty loop
}
