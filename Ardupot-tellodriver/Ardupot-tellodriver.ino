/*
* Ardupot Receiver Code for DJI Tello Driver
* Christopher Lai
* BANSHEE UAV
* Last Updated: 10/13/2022
* 
* Code that is used to control the DJI Tello.
* Uses keyboard library from U4 chip's USB2.0 compatibility
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Keyboard.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00501";
const uint8_t  channel = 123;

// Max size 32 bytes because of buffer limit
struct CMD_Packet {
  byte leftstickxL;
  byte leftstickxH;
  byte leftstickyL;
  byte leftstickyH;
  byte rightstickxL;
  byte rightstickxH;
  byte rightstickyL;
  byte rightstickyH;
  byte btnleft;
  byte btnright;
  byte potleftL;
  byte potleftH;
  byte potrightL;
  byte potrightH;
  byte modeSW;
  byte leftSW;
  byte rightSW;
};

// Simplified Packet
struct CMD_Tran{
  byte LButton;
  byte RButton;
  int LStickY;
  int LStickX;//YAW
  int RStickX;//ROLL
  int RStickY;//PITCH
  int LTrim;//THROTTLE LARGE
  int RTrim;//THROTTLE SMALL
  byte BbyMode;
  byte leftSW;
  byte rightSW;
};

CMD_Packet packet;
CMD_Tran cmd;

// Toggle Variables
bool conn = false;
bool takeoff = false;

void setup() {
  // Set Up Serial Comms - Debug
  Serial.begin(9600);

  // Set up Radio
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_LOW,0);
  radio.setChannel(channel);
  Serial.println("Starting Radio"); 

  // Check for OLED Availability
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  //to recieve
  radio.startListening();
}

void loop() {  
  if (radio.available()) {
    conn = true;
    radio.read(&packet, sizeof(packet));

    // Map values from transmitter packet to receiver packet
    map_values_RF();

    // Press keys according to data in cmd packet
    key_press();

    // Display data on OLED Display
    OLED_display_YES();
    delay(50);
    
  }
  else{
    OLED_display_NO();
    // delay(5); 
  }
}

void map_values_RF() {
  cmd.LStickX = packet.leftstickxL | ((packet.leftstickxH&0x03)<<8);
  cmd.LStickY = packet.leftstickyL | ((packet.leftstickyH&0x03)<<8);  
  cmd.RStickX = packet.rightstickxL | ((packet.rightstickxH&0x03)<<8);
  cmd.RStickY = packet.rightstickyL | ((packet.rightstickyH&0x03)<<8);
  cmd.LTrim = packet.potleftL | ((packet.potleftH&0x03)<<8);
  cmd.RTrim = packet.potrightL | ((packet.potrightH&0x03)<<8); 
  cmd.LButton = packet.btnleft;
  cmd.RButton = packet.btnright;
  cmd.BbyMode = packet.modeSW;
  cmd.leftSW = packet.leftSW;
  cmd.rightSW = packet.rightSW;
}

void key_press(){
  if (cmd.LStickX > 750){ // Yaw+
    Keyboard.press('d');
  }
  else if (cmd.LStickX < 250){ // Yaw-
    Keyboard.press('a');
  }
  else{
    Keyboard.release('d');
    Keyboard.release('a');
  }

  if (cmd.LStickY > 750){ // Throttle+
    Keyboard.press('w');
  }
  else if (cmd.LStickY < 250){  //Throttle-
    Keyboard.press('s');
  }
  else{
    Keyboard.release('w');
    Keyboard.release('s');
  }

  if (cmd.RStickX > 750){ // Roll+
    Keyboard.press(KEY_RIGHT_ARROW);
  }
  else if (cmd.RStickX < 750){  // Roll-
    Keyboard.press(KEY_LEFT_ARROW);
  }else{
    Keyboard.release(KEY_RIGHT_ARROW);
    Keyboard.release(KEY_LEFT_ARROW);
  }

  if (cmd.RStickY > 750){ // Pitch+
    Keyboard.press(KEY_UP_ARROW);
  }
  else if (cmd.RStickY < 750){  // Pitch-
    Keyboard.press(KEY_DOWN_ARROW);
  }
  else{
    Keyboard.release(KEY_UP_ARROW);
    Keyboard.release(KEY_DOWN_ARROW);
  }

  if (cmd.LTrim <= 500 && cmd.RTrim <= 500){
    if (cmd.LButton == 0){
      Keyboard.press('u');  // Front Flip
    }
    else{
      Keyboard.release('u');
    }
  }

  if (cmd.LTrim <= 500 && cmd.RTrim >= 500){
    if (cmd.LButton == 0){
      Keyboard.press('h');  // Left Flip
    }
    else{
      Keyboard.release('h');
    }
  }

  if (cmd.LTrim >= 500 && cmd.RTrim <= 500){
    if (cmd.LButton == 0){
      Keyboard.press('j');  // Back Flip
    }
    else{
      Keyboard.release('j');
    }
  }

  if (cmd.LTrim >= 500 && cmd.RTrim >= 500){
    if (cmd.LButton == 0){
      Keyboard.press('k');  // Right Flip
    }
    else{
      Keyboard.release('k');
    }
  }

  if (takeoff == false){
    if (cmd.LStickX > 750 && cmd.LStickY < 250){
      Keyboard.press('t');  // takeoff
      takeoff = true;
      delay(500);
      Keyboard.release('t');
    }
  }
  else{
    if (cmd.RButton == 0){
      Keyboard.press('l');  // land
      takeoff = false;
      delay(500);
      Keyboard.release('l');
    }
  }
}

void OLED_display_YES() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  if (conn)
  {
    display.println("RF: YES");
  }
  else
  {
    display.println("RF: NO");
  }
  
  display.setCursor(64,0);
  display.println("BTN: " + String(cmd.LButton) + " | " + String(cmd.RButton));
  display.setCursor(0,8);
  display.println("YAW: " + String(cmd.LStickX));
  display.setCursor(64,8);
  display.println("THR: " + String(cmd.LTrim));
  display.setCursor(0,16);
  display.println("ROLL: " + String(cmd.RStickX));
  display.setCursor(64,16);
  display.println("PTC: " + String(cmd.RStickY));
  display.setCursor(0,24);
  display.println("TL: " + String(cmd.LStickY));
  display.setCursor(64,24);
  display.println("TR: " + String(cmd.RTrim));
  display.display();
}

void OLED_display_NO() {
  Serial.println("NO CONNECTION");
  conn = false;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  if (conn)
  {
    display.println("RF: YES");
  }
  else
  {
    display.println("RF: NO");
  }
  display.setCursor(64,0);
  display.println("BTN: " + String(0) + " | " + String(0));
  display.setCursor(0,8);
  display.println("YAW: " + String(0));
  display.setCursor(64,8);
  display.println("THR: " + String(0));
  display.setCursor(0,16);
  display.println("ROLL: " + String(0));
  display.setCursor(64,16);
  display.println("PTC: " + String(0));
  display.setCursor(0,24);
  display.println("TL: " + String(0));
  display.setCursor(64,24);
  display.println("TR: " + String(0));
  display.display();
}

void resetCMD() {
  cmd.LButton=1;
  cmd.RButton=1;
  cmd.LStickY=500;
  cmd.LStickX=500;//YAW
  cmd.RStickX=500;//ROLL
  cmd.RStickY=500;//PITCH
  cmd.LTrim=0;//THROTTLE LARGE
  cmd.RTrim=0;//THROTTLE SMALL
}
