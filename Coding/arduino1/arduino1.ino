/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/communication-between-two-arduino
 */

// ARDUINO #1: TCP CLIENT + A BUTTON/SWITCH
#include <ezButton.h>
#include <SPI.h>
#include <Ethernet.h>

const int BUTTON_PIN = 7;
const int serverPort = 4080;
ezButton button(BUTTON_PIN);  // create ezButton that attach to pin 7;

byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x03};
IPAddress serverAddress(128, 0, 0, 0);
EthernetClient TCPclient;

void setup() {
  Serial.begin(9600);
  button.setDebounceTime(50); // set debounce time to 50 milliseconds

  Serial.println("ARDUINO #1: TCP CLIENT + A BUTTON/SWITCH");

  // Initialize Ethernet Shield:
  if (Ethernet.begin(mac) == 0)
    Serial.println("Failed to configure Ethernet using DHCP");

  // connect to TCP server (Arduino #2)
  if (TCPclient.connect(serverAddress, serverPort))
    Serial.println("Connected to TCP server");
  else
    Serial.println("Failed to connect to TCP server");
}

void loop() {
  button.loop(); // MUST call the loop() function first

  if (!TCPclient.connected()) {
    Serial.println("Connection is disconnected");
    TCPclient.stop();

    // reconnect to TCP server (Arduino #2)
    if (TCPclient.connect(serverAddress, serverPort))
      Serial.println("Reconnected to TCP server");
    else
      Serial.println("Failed to reconnect to TCP server");
  }

  if (button.isPressed()) {
    TCPclient.write('1');
    TCPclient.flush();
    Serial.println("- The button is pressed,  sent command: 1");
  }

  if (button.isReleased()) {
    TCPclient.write('0');
    TCPclient.flush();
    Serial.println("- The button is released, sent command: 0");
  }
}
