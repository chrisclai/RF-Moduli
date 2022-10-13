/*
* Ardupot Receiver Code for Drone PPM
* Christopher Lai
* BANSHEE UAV
* Last Updated: 10/13/2022
* 
* Code that is used to control a drone like Storm4 or Brody with PPM
* Included calibration sequence with joystick buttons
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "PPMEncoder.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00501";
const uint8_t  channel = 123;

// PPM Pin
#define SIG_PIN 10

// Default Low/High/Mid Pair
const int LOW_END = 1000;
const int HIGH_END = 2150;
const int MID_POINT = (HIGH_END + LOW_END)/2;

// Variable Low/High/Mid Pair (Trim Settings)
int high_end_yaw = HIGH_END - 158;
int mid_point_yaw = (high_end_yaw + LOW_END)/2;

int high_end_roll = HIGH_END - 212;
int mid_point_roll = (high_end_roll + LOW_END)/2;

int high_end_pitch = HIGH_END - 145;
int mid_point_pitch = (high_end_pitch + LOW_END)/2;

int offset = 0;
int OFFSET_LOW = -250;
int OFFSET_HIGH = 250;

// Arming Failsafe
int tempARM = LOW_END;

// Toggle Button Variables
bool lockThrottle = false;
int prevLButton = 1;
int prevRButton = 1;
int lockThrottleValue = 0;
bool piControl = false;
int calibrateState = 0;

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

const byte YAW = 3;
const byte PITCH = 1;
const byte ROLL = 0;
const byte THROTTLE = 2;
const byte MODE = 4;
const byte HOLD = 5;

CMD_Packet packet;
CMD_Tran cmd;

bool conn = false;

void setup() {
  // Set Up Serial Comms - Debug
  Serial.begin(9600);

  //Begin PPM Encoder
  ppmEncoder.begin(SIG_PIN);

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

    int RecLarge = packet.leftstickxL | ((packet.leftstickxH&0x03)<<8);
    cmd.LStickX = map(RecLarge,0,1023,high_end_yaw,LOW_END); //Yaw - 1992 high
    
    RecLarge= packet.leftstickyL | ((packet.leftstickyH&0x03)<<8);  
    cmd.LStickY = map(RecLarge,0,1023,LOW_END,HIGH_END); // NOT THROTTLE
    
    RecLarge= packet.rightstickxL | ((packet.rightstickxH&0x03)<<8);
    cmd.RStickX = map(RecLarge,0,1023,high_end_roll,LOW_END); // Roll - 1938 high
    
    RecLarge= packet.rightstickyL | ((packet.rightstickyH&0x03)<<8);
    cmd.RStickY = map(RecLarge,1023,0,high_end_pitch,LOW_END); // Pitch - 2005 high
    
    RecLarge= packet.potleftL | ((packet.potleftH&0x03)<<8);
    cmd.LTrim = map(RecLarge,0,1023,LOW_END,HIGH_END);
    
    RecLarge = packet.potrightL | ((packet.potrightH&0x03)<<8); 
    cmd.RTrim = map(RecLarge,0,1023,LOW_END,HIGH_END);

    cmd.LButton = packet.btnleft;
    cmd.RButton = packet.btnright;
    cmd.BbyMode = packet.modeSW;
    cmd.leftSW = packet.leftSW;
    cmd.rightSW = packet.rightSW;

    // State Machines (Button Calibration)
    
    // Left Button: Throttle Lock
    if (!cmd.LButton) // If button has been pressed
    {
      if (prevLButton != cmd.LButton) // Check to see if it was a held button (prevvalue)
      {
        lockThrottle = !lockThrottle;     // If it was different turn the lock variable on
        prevLButton = cmd.LButton;     // Set the prev value to the ON position to prevent hold
        lockThrottleValue = cmd.LTrim;
      }
    }
    else
    {
        prevLButton = cmd.LButton;     // If the button was released, set prev value back to 1
    }
    
    // Right Button: Trim Calibration
    if (!cmd.RButton) // If button has been pressed
    {
      if (prevRButton != cmd.RButton) // Check to see if it was a held button (prevvalue)
      {
        calibrateState++;
        prevRButton = cmd.RButton;     // Set the prev value to the ON position to prevent hold
      }
    }
    else
    {
      prevRButton = cmd.RButton;     // If the button was released, set prev value back to 1
    }

    // State Manager (When button pressed, add 1 to state)
    // State 0: Nothing Happens
    // State 1: Calibrate Yaw
    // State 2: Calibrate Roll
    // State 3: Calibrate Pitch
    // State 4: MOVE RTRIM BACK TO 0 POSITION (2 bit op)
    // State 5: Resets state back to 0
    if (calibrateState == 1)
    {
      offset = map(cmd.RTrim,LOW_END,HIGH_END,OFFSET_LOW,OFFSET_HIGH);
      high_end_yaw = HIGH_END - 158 + offset;
      mid_point_yaw = (high_end_yaw + LOW_END)/2;
    }
    else if (calibrateState == 2)
    {
      offset = map(cmd.RTrim,LOW_END,HIGH_END,OFFSET_LOW,OFFSET_HIGH);
      high_end_roll = HIGH_END - 212 + offset;
      mid_point_roll = (high_end_roll + LOW_END)/2;
    }
    else if (calibrateState == 3)
    {
      offset = map(cmd.RTrim,LOW_END,HIGH_END,OFFSET_LOW,OFFSET_HIGH);
      high_end_pitch = HIGH_END - 145 + offset;
      mid_point_pitch = (high_end_pitch + LOW_END)/2;
    }
    else if (calibrateState == 4)
    {
      // Do nothing (DISABLES RTRIM, RETURN RTRIM TO 0)
    }
    else
    {
      calibrateState = 0;
    }

    // Must be above assignment of BbyMode to work
    tempARM = cmd.BbyMode;

    // Map switch from 0-1 to 0-2150
    cmd.BbyMode = map(packet.modeSW,1,0,LOW_END, HIGH_END);

    if (lockThrottle)
    {
      ppmEncoder.setChannel(YAW, mid_point_yaw);
      ppmEncoder.setChannel(ROLL, mid_point_roll + 28);
      ppmEncoder.setChannel(PITCH, mid_point_pitch - 5);
      ppmEncoder.setChannel(THROTTLE, cmd.LTrim);
      if (calibrateState == 0) ppmEncoder.setChannel(MODE, cmd.RTrim);
      ppmEncoder.setChannel(HOLD, cmd.BbyMode);
    }
    else
    {
      ppmEncoder.setChannel(YAW, cmd.LStickX);
      ppmEncoder.setChannel(ROLL, cmd.RStickX);
      ppmEncoder.setChannel(PITCH, cmd.RStickY);
      ppmEncoder.setChannel(THROTTLE, cmd.LTrim);
      if (calibrateState == 0) ppmEncoder.setChannel(MODE, cmd.RTrim);
      ppmEncoder.setChannel(HOLD, cmd.BbyMode);
    }
    OLED_display_YES();
    delay(50);
    
  }
  else{
    int tempTHROTTLE = cmd.LTrim - 1;
    if(tempTHROTTLE<LOW_END)tempTHROTTLE = LOW_END;
    resetCMD();
    cmd.LTrim = tempTHROTTLE;
    ppmEncoder.setChannel(YAW, cmd.LStickX);
    ppmEncoder.setChannel(ROLL, cmd.RStickX);
    ppmEncoder.setChannel(PITCH, cmd.RStickY);
    ppmEncoder.setChannel(THROTTLE, cmd.LTrim);
    if (calibrateState == 0) ppmEncoder.setChannel(MODE, cmd.RTrim);
  
    if (tempTHROTTLE > (LOW_END + 25) && tempARM > HIGH_END - 25) {
      cmd.BbyMode = HIGH_END;
    }
    else {
      cmd.BbyMode = LOW_END;
    }
    ppmEncoder.setChannel(HOLD, cmd.BbyMode); 
    OLED_display_NO();
    // delay(5); 
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
  cmd.LStickY=MID_POINT;
  cmd.LStickX=mid_point_yaw;//YAW
  cmd.RStickX=mid_point_roll + 28;//ROLL
  cmd.RStickY=mid_point_pitch - 5;//PITCH
  cmd.LTrim=LOW_END;//THROTTLE LARGE
  cmd.RTrim=LOW_END;//THROTTLE SMALL
}
