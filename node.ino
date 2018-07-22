#include <SoftwareSerial.h>
#include <Wire.h> 
#include <Arduino.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define sensorPin 7

int sensorState = 0;

SoftwareSerial mySerial(2, 3); // RX, TX

unsigned long t1, t2;
float len = 15; //cm
float v;

void setup() {
  // put your setup code here, to run once:
  u8g2.begin();
  
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(sensorPin, INPUT);
}

void loop() {
  t1 = millis();
  sensorState = digitalRead(sensorPin);

  while(sensorState == HIGH) {
    t2 = millis() - t1;
    //Serial.println("t2 = " + String(t2) + "ms");
    
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_torussansbold8_8u);
      u8g2.setCursor(0, 16);
      u8g2.print(t2);
      u8g2.setFont(u8g2_font_lucasfont_alternate_tf);
      u8g2.print("ms");
    } while (u8g2.nextPage());
    
    sensorState = digitalRead(sensorPin);
    if (sensorState == LOW) {
      v = (10*(len/t2))*3.6;
      //Serial.println("Velocity = " + String(v) + "km/h");
      mySerial.println(v);

      u8g2.firstPage();
      do {
        u8g2.setFont(u8g2_font_torussansbold8_8u);
        u8g2.setCursor(0, 16);
        u8g2.print(t2);
        u8g2.setFont(u8g2_font_lucasfont_alternate_tf);
        u8g2.print("ms");
      
        //u8g2.setFont(u8g2_font_lucasfont_alternate_tf);
        u8g2.setFont(u8g2_font_osb29_tn);
        //u8g2.setFont(u8g2_font_torussansbold8_8u);
        u8g2.setCursor(0, 56);
        //u8g2.print(F("0"));
        u8g2.print(v);
        u8g2.setFont(u8g2_font_lucasfont_alternate_tf);
        u8g2.print("km/h");
      } while ( u8g2.nextPage() );
    }
  }
}
