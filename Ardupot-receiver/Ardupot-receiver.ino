/*
* Ardupot Receiver Code
* Christopher Lai
* CPP Hyperloop Edition
* Last Updated: 8/26/2022
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00501";
const uint8_t  channel = 123;

const int echoPin1 = A0;
const int trigPin1 = A1;
const int echoPin2 = A2;
const int trigPin2 = A3;

const int dir1 = 5;
const int pwm1 = 6;
const int dir2 = 9;
const int pwm2 = 10;

long duration = 0;
int distanceL = 0;
int distanceR = 0;
bool conn = false;

void setup() {
  // Set Up Serial Comms - Debug
  Serial.begin(9600);

  // Pinmode Setup
  pinMode(echoPin1, INPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin2, OUTPUT);

  // Force stop
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);

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
  
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  display.println("LOADING...");
  display.setCursor(0, 20);
  delay(2500);

  //to recieve
  radio.startListening();
}
void loop() {
  
  if (radio.available()) {
    conn = true;
    
    // Packet = [leftstickx, leftsticky, rightstickx, rightsticky, btnleft, btnright, potleft, potright, toggleleft, toggleright]
    byte packet[] = {0,0,0,0,0,0,0,0,0,0};
    radio.read(&packet, sizeof(packet));

    // Will delay code, only use if necessary
    debug(packet);

    distanceL = getDistance(trigPin1, echoPin1);
    distanceR = getDistance(trigPin2, echoPin2);  

    analogWrite(pwm1, 120); // left pot
    analogWrite(pwm2, 120);

    if (distanceL <= 15 && distanceR > 15)
    {
      motor_forward(200);
    }
    else if (distanceR <= 15 && distanceL > 15)
    {
      motor_backward(200);
    }
    else if (packet[0] <= 75)
    {
      motor_forward(200);
    }
    else if (packet[0] >= 200)
    {
      motor_backward(200);
    }
    else
    {
      motor_stop();
    }
    
    delay(50);
    
  }
  else {
    Serial.println("NO CONNECTION");
    conn = false;
    motor_stop();
  }

  OLED_display();  
}

void motor_stop()
{
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
}

void motor_forward(int speed_val)
{
  analogWrite(pwm1, speed_val);
  analogWrite(pwm2, speed_val);
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, HIGH);
}

void motor_backward(int speed_val)
{
  analogWrite(pwm1, speed_val);
  analogWrite(pwm2, speed_val);
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
}

int getDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // 0.034 constant from speed of sound
}

void OLED_display()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  if (conn)
  {
    display.println("RF Status: PASS");
  }
  else
  {
    display.println("RF Status: FAIL");
  }
  display.setCursor(0,10);
  display.println("Driver Mode: MAIN");
  display.setCursor(0,20);
  display.println("DistL: " + String(distanceL));
  display.setCursor(64,20);
  display.println("DistR: " + String(distanceR));
  display.display();
}

void debug(byte packet[]) {
      //THIS IS FOR DEBUG
      Serial.print("LX: ");
      Serial.print(packet[0]);
      Serial.print("; LY: ");
      Serial.print(packet[1]);
      Serial.print("; LB: ");
      Serial.print(packet[4]);
      Serial.print("; RX: ");
      Serial.print(packet[2]); 
      Serial.print("; RY: ");
      Serial.print(packet[3]);
      Serial.print("; RB: ");
      Serial.print(packet[5]);
      Serial.println();
      //delay(5);
}
