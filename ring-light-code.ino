 #include "FastLED.h"
#include <NXPMotionSense.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Bounce.h>

#define NUM_LEDS 32
#define DATA_PIN 3

uint8_t state = 8;
bool new_data;

CRGB leds[NUM_LEDS];
uint8_t whiteLed = 0;
unsigned long previous_millis;

NXPMotionSense imu;
NXPSensorFusion filter;

Bounce button1 = Bounce(22, 2);
Bounce button2 = Bounce(23, 3);

void setup() {
  delay(1000);

  FastLED.addLeds<WS2811, 11, GRB>(leds, NUM_LEDS);
  pinMode(7, OUTPUT);
  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);
  
  leds[state + 16] = CRGB::White; // n - 8 + 24
  digitalWrite(7, HIGH);
  FastLED.show();
  digitalWrite(7, LOW);
  
  Serial.begin(115200);
  imu.begin();
  filter.begin(100);
}

void loop() {
  button1.update();
  button2.update();

  bool b1 = button1.fallingEdge();
  bool b2 = button2.fallingEdge();

  if (b1 || b2) { // Either button pressed
    previous_millis = millis();
    if (state < 8) { // Coming from an animation state
      state += 8;
      for (uint8_t i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
      }
    } else { //Coming from a different transition state
      leds[state + 16] = CRGB::Black; // n - 8 + 24
    }
    if (b1) {
      state += 1;
      if (state > 15) {
        state = 8;
      }
    } else if (b2) {
      state -= 1;
      if (state < 8) {
        state = 15;
      }
    }
    leds[state + 16] = CRGB::White; // n - 8 + 24
    digitalWrite(7, HIGH);
    FastLED.show();
    digitalWrite(7, LOW);
  }

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;

  if (imu.available()) {
    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    
    Serial.print("m: ");
    Serial.print(mx);
    Serial.print(" ");
    Serial.print(my);
    Serial.print(" ");
    Serial.print(mz);
    Serial.println();

    new_data = true;
  }

  if (state < 6) {
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    if (roll > 0) {
      roll = 180 - roll;
    } else {
      roll = -180 - roll;
    }

    float r = roll / 180.0;
    float p = pitch / 180.0;
    float a = atan2f(r,p) + PI;

    if (state == 0) {
      uint8_t n = (int)(a * 3.821656050955414);
      n = 24 - n - 1;
      if (n < 0 || n > 23) {
        n = 0;
      }
      n = (n + 12) % 24;
      
      leds[n] = CRGB::White;
      digitalWrite(7, HIGH);
      FastLED.show();
      digitalWrite(7, LOW);
      leds[n] = CRGB::Black;
    } else { // state == 1 || state == 2
      float n = a * 3.821656050955414;
      uint8_t p1 = floor(n);
      uint8_t p2 = ceil(n);
      uint8_t v1 = (uint8_t)((1 - (n - p1)) * 250);
      uint8_t v2 = (uint8_t)((1 - (p2 - n)) * 250);
      if (state == 2) {
        v1 = v1 >> 2;
        v2 = v2 >> 2;
      } else if (state == 3) {
        v1 = (uint8_t)(((uint16_t)v1 * v1) >> 8);
        v2 = (uint8_t)(((uint16_t)v2 * v2) >> 8);
      }
      if ( p2 > 23 ) {
        p2 -= 24;
        if (p1 > 23) {
          p1 -= 24; 
        }
      }
      p1 = 24 - p1 - 1;
      p2 = 24 - p2 - 1;
      p1 = (p1 + 12) % 24;
      p2 = (p2 + 12) % 24;
      if (state == 5) {
        if (v1 > v2) {
          if (v1 >= 192) {
            v1 = 255;
            v2 = 0;
          } else {
            v1 = (v1 - 64) * 2;
            v2 = 255 - v1;
          }
        } else {
          if (v2 >= 192) {
            v2 = 255;
            v1 = 0;
          } else {
            v2 = (v2 - 64) * 2;
            v1 = 255 - v2;
          }
        }
      }
      if (state == 4) {
        leds[p1] = CHSV(0, 0, v1);
        leds[p2] = CHSV(0, 0, v2);
      } else {
        leds[p1] = CRGB(v1, v1, v1);
        leds[p2] = CRGB(v2, v2, v2);
      }
      digitalWrite(7, HIGH);
      FastLED.show();
      digitalWrite(7, LOW);
      leds[p1] = CRGB::Black;
      leds[p2] = CRGB::Black;
    }
  } else if (state == 6) {
    if (new_data) {
      float a = atan2f(mx, my) + PI; //Add pi so numbers are not negative
      uint8_t n = (int)(a * 3.821656050955414);
      n = 24 - n - 1;
      if (n < 0 || n > 23) {
        n = 0;
      }
      n = (n + 18) % 24;

      Serial.print(" ");
      Serial.println(n);
      
      leds[n] = CRGB::Red;
      leds[(n + 6) % 24] = CRGB(32, 32, 32);
      leds[(n + 12) % 24] = CRGB(32, 32, 32);
      leds[(n + 18) % 24] = CRGB(32, 32, 32);
      if (n == 11) {
        leds[24] = CRGB(32, 32, 32);
        leds[25] = CRGB(32, 32, 32);
        leds[30] = CRGB::Red;
        leds[31] = CRGB::Red;
      } else if (n == 23) {
        leds[24] = CRGB::Red;
        leds[25] = CRGB::Red;
        leds[30] = CRGB(32, 32, 32);
        leds[31] = CRGB(32, 32, 32);
      }
      digitalWrite(7, HIGH);
      FastLED.show();
      digitalWrite(7, LOW);
      if (n == 11 || n == 23) {
        leds[24] = CRGB::Black;
        leds[25] = CRGB::Black;
        leds[30] = CRGB::Black;
        leds[31] = CRGB::Black;
      }
      leds[n] = CRGB::Black;
      leds[(n + 6) % 24] = CRGB::Black;
      leds[(n + 12) % 24] = CRGB::Black;
      leds[(n + 18) % 24] = CRGB::Black;
    }
  } else if (state == 7) {
    if (new_data) {
      float a = atan2f(mx, my) + PI; //Add pi so numbers are not negative
      float n = a * 3.821656050955414;
      uint8_t p1 = floor(n);
      uint8_t p2 = ceil(n);
      uint8_t v1 = (uint8_t)((1 - (n - p1)) * 250);
      uint8_t v2 = (uint8_t)((1 - (p2 - n)) * 250);
      if ( p2 > 23 ) {
        p2 -= 24;
        if (p1 > 23) {
          p1 -= 24; 
        }
      }
      p1 = 24 - p1 - 1;
      p2 = 24 - p2 - 1;
      p1 = (p1 + 18) % 24;
      p2 = (p2 + 18) % 24;
      if (v1 > v2) {
        if (v1 >= 192) {
          v1 = 255;
          v2 = 0;
        } else {
          v1 = (v1 - 64) * 2;
          v2 = 255 - v1;
        }
      } else {
        if (v2 >= 192) {
          v2 = 255;
          v1 = 0;
        } else {
          v2 = (v2 - 64) * 2;
          v1 = 255 - v2;
        }
      }
      
      leds[p1] = CRGB(v1, 0, 0);
      leds[p2] = CRGB(v2, 0, 0);
      int w1 = v1 >> 3;
      int w2 = v2 >> 3;
      leds[(p1 + 6) % 24] = CRGB(w1, w1, w1);
      leds[(p1 + 12) % 24] = CRGB(w1, w1, w1);
      leds[(p1 + 18) % 24] = CRGB(w1, w1, w1);
      leds[(p2 + 6) % 24] = CRGB(w2, w2, w2);
      leds[(p2 + 12) % 24] = CRGB(w2, w2, w2);
      leds[(p2 + 18) % 24] = CRGB(w2, w2, w2);
      
      if (p1 == 11) {
        leds[24] = CRGB(w1, w1, w1);
        leds[25] = CRGB(w1, w1, w1);
        leds[30] = CRGB(v1, 0, 0);
        leds[31] = CRGB(v1, 0, 0);
      } else if (p1 == 23) {
        leds[24] = CRGB(v1, 0, 0);
        leds[25] = CRGB(v1, 0, 0);
        leds[30] = CRGB(w1, w1, w1);
        leds[31] = CRGB(w1, w1, w1);
      } else if (p2 == 11) {
        leds[24] = CRGB(w2, w2, w2);
        leds[25] = CRGB(w2, w2, w2);
        leds[30] = CRGB(v2, 0, 0);
        leds[31] = CRGB(v2, 0, 0);
      } else if (p2 == 23) {
        leds[24] = CRGB(v2, 0, 0);
        leds[25] = CRGB(v2, 0, 0);
        leds[30] = CRGB(w2, w2, w2);
        leds[31] = CRGB(w2, w2, w2);
      }
      
      digitalWrite(7, HIGH);
      FastLED.show();
      digitalWrite(7, LOW);
      
      for (uint8_t i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
      }
    }
  } else { // 8 - 15
    if (millis() - previous_millis > 1000) {
      leds[state + 16] = CRGB::Black; // n - 8 + 24
      digitalWrite(7, HIGH);
      FastLED.show();
      digitalWrite(7, LOW);
      state &= 7;
    }
  }
  
  new_data = false;
}
