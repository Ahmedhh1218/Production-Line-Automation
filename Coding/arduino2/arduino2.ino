/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/communication-between-two-arduino
 */

// ARDUINO #2: TCP SERVER + AN LED
#include <SPI.h>
#include <Ethernet.h>

const int LED_PIN = 7;
const int serverPort = 4080;

byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};
EthernetServer TCPserver(serverPort);

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);

  Serial.println("ARDUINO #2: TCP SERVER + AN LED");

  // Initialize Ethernet Shield:
  if (Ethernet.begin(mac) == 0)
    Serial.println("Failed to configure Ethernet using DHCP");

  // Print your local IP address:
  Serial.print("TCP Server IP address: ");
  Serial.println(Ethernet.localIP());
  Serial.println("-> Please update the serverAddress in Arduino #1 code");

  // Listening for a TCP client (from Arduino #1)
  TCPserver.begin();
}

void loop() {
  // Wait for a TCP client from Arduino #1:
  EthernetClient client = TCPserver.available();

  if (client) {
    // Read the command from the TCP client:
    char command = client.read();
    Serial.print("- Received command: ");
    Serial.println(command);

    if (command == '1')
      digitalWrite(LED_PIN, HIGH); // Turn LED on
    else if (command == '0')
      digitalWrite(LED_PIN, LOW);  // Turn LED off

    Ethernet.maintain();
  }
}
