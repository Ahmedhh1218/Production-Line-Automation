#include <EtherCard.h>
#include <IPAddress.h>

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
}

void setup(){

  // Initialize Serial
  Serial.begin(57600);

  // Initialize Ethernet
  initializeEthernet();
  
  //register udpSerialPrint() to destination port (callback function)
  ether.udpServerListenOnPort(&udpSerialPrint, dstPort);
}

void loop(){
  // receive packets, get ready for callbacks
  ether.packetLoop(ether.packetReceive());
}