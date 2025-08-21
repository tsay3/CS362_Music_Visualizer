// testcolors demo for Adafruit RGBmatrixPanel library.
// Renders 512 colors on our 16x32 RGB LED matrix:
// http://www.adafruit.com/products/420
// Library supports 4096 colors, but there aren't that many pixels!  :)

// Written by Limor Fried/Ladyada & Phil Burgess/PaintYourDragon
// for Adafruit Industries.
// BSD license, all text above must be included in any redistribution.

#include <RGBmatrixPanel.h>
#include <SoftwareSerial.h>
#include <IRremote.hpp>

// Most of the signal pins are configurable, but the CLK pin has some
// special constraints.  On 8-bit AVR boards it must be on PORTB...
// Pin 8 works on the Arduino Uno & compatibles (e.g. Adafruit Metro),
// Pin 11 works on the Arduino Mega.  On 32-bit SAMD boards it must be
// on the same PORT as the RGB data pins (D2-D7)...
// Pin 8 works on the Adafruit Metro M0 or Arduino Zero,
// Pin A4 works on the Adafruit Metro M4 (if using the Adafruit RGB
// Matrix Shield, cut trace between CLK pads and run a wire to A4).

#define CLK  8   // USE THIS ON ARDUINO UNO, ADAFRUIT METRO M0, etc.
//#define CLK A4 // USE THIS ON METRO M4 (not M0)
//#define CLK 11 // USE THIS ON ARDUINO MEGA
#define OE   9
#define LAT 10
#define A   A0
#define B   A1
#define C   A2

SoftwareSerial SlaveSerial(A4, A5);
RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, false);

int receivedData;
uint8_t newSound;
uint8_t history[32];
int tracker = 0;

int color = 0;
# define COLORS 10

uint16_t colors[] = {matrix.Color444(15, 0, 0), matrix.Color444(0, 15, 0),
matrix.Color444(0, 0, 15), matrix.Color444(15, 15, 0), matrix.Color444(15, 0, 15),
matrix.Color444(0, 15, 15), matrix.Color444(15, 4, 3), matrix.Color444(15, 15, 15)};

// number of visualizations
int mode = 0;
#define MODES 6

// represents how divided the bars are, from 1 - 6
// 1 = 1 bar
// 6 = 32 bars
int divisionLevel = 4;

int lower_limit = 0;
int upper_limit = 40;

//int lower_limit = 126;
//int upper_limit = 129;

#define INPUT_BTN1  A3
#define INPUT_BTN2  12
#define INPUT_BTN3  11
int in1_last_status = 0;
int in2_last_status = 0;
int in3_last_status = 0;
unsigned long int debounce_time = 0;

void receiveData() {
  uint8_t sounddata = 255;
  while (SlaveSerial.available()) {
    //matrix.fillScreen(matrix.Color333(0, 0, 0));
    receivedData = SlaveSerial.read();
    //Serial.print("IN: ");
    //Serial.println(receivedData);
    if(receivedData < upper_limit && receivedData > lower_limit){
      sounddata = map(receivedData, lower_limit, upper_limit, 0, 15);
    }
  }
  // if no data received within bounds, return
  if (sounddata == 255) {
    return;
  }
  history[tracker] = sounddata;
  updateHistory();
  //uint8_t sounddata = (receivedData - 120) / 2.5;
  //Serial.print(receivedData);
  //Serial.print("  ===> Sound: ");
  //Serial.println(sounddata);
  outputData();
}

void updateHistory() {
  // tracker decreases from 31 to 0, then back to 31
  tracker--;
  if (tracker < 0) {
    tracker = 31;
    //tracker = (1 << (divisionLevel - 1)) - 1;
  }
  history[tracker] = 0;
}


// Option 0: all red
// Option 1: all green
// Option 2: all blue
// Option 3: all yellow
// Option 4: all magenta
// Option 5: all cyan
// Option 5: all salmon
// Option 7: all white
// Option 8: red on bottom, yellow in middle, green on top
// Option 9: cyan on bottom, blue on top


uint16_t colorMap(int x, int y) {
	switch(color) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			return colors[color];
		case 8:
			return matrix.Color444(min(15, y*2),min(15, 30 - 2*y),0);	// 0-16 for 0 < y < 8, 16 for y > 8
		default:
			return matrix.Color444(0, 15 - y, 15);
  }
}

void outputData() {
  //history[tracker] = newSound;
  switch (mode) {
    case 0:
      classicVisualizer();
      break;

    case 1:
      centerVisualizer();
      break;

    case 2:
      histogram();
      break;

    case 3:
      classicVisualizer2();
      break;

    case 4:
      fieryVisualizer();
      break;

    default:
      heartbeatVisualizer();
      break;
  }
  //updateHistory();
}

void readButtons() {
  if (debounce_time - millis() > 50) {
    int in1_current_status = digitalRead(INPUT_BTN1);
    if (in1_current_status != in1_last_status) {
      if (in1_current_status == HIGH) {
        color = (color + 1) % COLORS;
        //Serial.print("Color changed to");
        //Serial.println(color);
      } 
      in1_last_status = in1_current_status;
      debounce_time = millis();
    }

    int in2_current_status = digitalRead(INPUT_BTN2);
    if (in2_current_status != in2_last_status) {
      if (in2_current_status == HIGH) {
        mode = (mode + 1) % MODES;
      }
      in2_last_status = in2_current_status;
      debounce_time = millis();
    }

    int in3_current_status = digitalRead(INPUT_BTN3);
    if (in3_current_status != in3_last_status) {
      if (in3_current_status == HIGH) {
        divisionLevel = (divisionLevel % 6) + 1;
      }
      in3_last_status = in3_current_status;
      debounce_time = millis();
    }
    
    outputData();
  }
}

const unsigned long BTN_VOL_UP = 0xB946FF00;
const unsigned long BTN_VOL_DOWN = 0xEA15FF00;
const unsigned long BTN_BACK = 0xBB44FF00;
const unsigned long BTN_FWD = 0xBC43FF00;
const unsigned long BTN_DOWN = 0xF807FF00;
const unsigned long BTN_UP = 0xF609FF00;
const unsigned long BTN_EQ = 0xE619FF00;

void remoteCode() {
  // Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
  if (IrReceiver.decode()) {

    switch(IrReceiver.decodedIRData.decodedRawData) {

    case BTN_VOL_UP: // increase
        color = (color + 1) % COLORS;
        break;

    case BTN_VOL_DOWN: // decrease
        color = (color - 1) % COLORS;
        break;

    case BTN_BACK: // decrease lower limit
        if (lower_limit > 0)
          lower_limit--;
        break;

    case BTN_FWD: // increase lower limit
        if (lower_limit + 2 < upper_limit)
          lower_limit++;
        break;

    case BTN_DOWN: // decrease upper limit
        if (upper_limit - 2 > lower_limit)
          upper_limit--;
        break;

    case BTN_UP: // increase lower limit
        if (upper_limit < 255)
          upper_limit++;
        break;

    case BTN_EQ:
        divisionLevel++;
        if (divisionLevel > 6) {
          divisionLevel = 1;
          //tracker = 0;
        }
        break;

    default:
      //Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
      break;
    }
    /*Serial.print("color ");
    Serial.print(color);
    Serial.print(", bars: ");
    Serial.print(1 << (divisionLevel - 1));
    Serial.print(", ");
    Serial.print(lower_limit);
    Serial.print(" - ");
    Serial.println(upper_limit);*/
    IrReceiver.resume();
  }
}

void setup() {
  for (uint8_t i = 0; i < 32; i++) {
    history[i] = 0;
  }
  IrReceiver.begin(13, ENABLE_LED_FEEDBACK);
  Serial.begin(38400);
  SlaveSerial.begin(38400);
  matrix.begin();
  pinMode(INPUT_BTN1, INPUT);
  pinMode(INPUT_BTN2, INPUT);
  pinMode(INPUT_BTN3, INPUT);
}

void loop() {
  readButtons();
  remoteCode();
  receiveData();
  delay(150);
}

void cleanVLine(uint8_t x, uint8_t y, uint8_t h) {
  if ((y + h) > 16) h = 16 - y;
  for(uint8_t i = 0; i < h; i++) {
    matrix.drawPixel(x, y + i, colorMap(x,y));
  }
  for(uint8_t i = 0; i < y; i++) {
    matrix.drawPixel(x, i, matrix.Color444(0, 0, 0));
  }
  for(uint8_t i = y + h; i < 16; i++) {
    matrix.drawPixel(x, i, matrix.Color444(0, 0, 0));
  }
}

void clearColumn(uint8_t x) {
  for(uint8_t i = 0; i < 16; i++) {
    matrix.drawPixel(x, i, matrix.Color444(0, 0, 0));
  }
}

// The visualizers

// different bars for each part of the buffer
void classicVisualizer() {
  uint16_t j = 0;
  for(uint8_t i = 0; i < 32; i++) {
    if (i >= (32 >> (divisionLevel - 1)) * j) {
      j++;
    }
    int outputIndex = (tracker + j) % 32;
    cleanVLine(i, 0, history[outputIndex]);
  }
}

void classicVisualizer2() {
  int j = 0;
  //int barsPer = 1 << (divisionLevel - 1);
  int nthBar = 32 >> (divisionLevel - 1);
  int randSeed = 0;
  for(uint8_t i = 0; i < 32; i++ ){ 
    if (i >= (32 >> (divisionLevel - 1)) * (j + 1)) {
      j++;
    }
    if(i % nthBar == 0) {
        randSeed = random(-2, 3);
    }
    int outputIndex = (tracker + j) % 32;
    cleanVLine(i, 0, history[outputIndex] + randSeed);
  }
} 

void fieryVisualizer() {
  int j = 0;
  int overflow = 1 << (divisionLevel - 1);
  for(uint8_t i = 0; i < 32; i++) {
    if (i >= (32 >> (divisionLevel - 1)) * (j + 1)) {
      j++;
    }
    int outputIndex = (tracker + j) % 32;
    //if (outputIndex < 0) outputIndex += overflow;
    uint8_t height = map(history[outputIndex], 0, 15, 2, 13) + random(-2, 3);
    cleanVLine(i, 0, height);
  }
}

void centerVisualizer() {
  int rando;
  int j = 0;
  for(uint8_t i = 0; i < 32; i++) {
    if (i >= (32 >> (divisionLevel - 1)) * (j + 1)) {
      j++;
    }
    int outputIndex = (tracker + j) % 32;
    rando = random(-1, 2);
    uint8_t top = map((history[outputIndex] / 2), 0, 8, 1, 7) + rando + 8; // 
    uint8_t bottom = 16 - top;
    cleanVLine(i, bottom, top - bottom);
  }
}

void histogram() {
  for(uint8_t i = 0; i < 32; i++) {
    int y = history[(tracker + i) % 32];
    for (uint8_t j = 0; j < 16; j++) {
      if (j == y) matrix.drawPixel(i, j, colorMap(i,y));
      else matrix.drawPixel(i, j, matrix.Color444(0, 0, 0));
    }
  }
}

void heartbeatVisualizer() {
  int j = -1;
  int limits = divisionLevel;
  if (divisionLevel > 4) limits = 4;
  // max width of each beat
  int width = 32 >> (limits - 1);
  for (uint8_t k = 0; k < 32; k++) {
    clearColumn(k);
  }
  uint8_t quarter = 32 >> (limits + 1);
  for(uint8_t i = 0; i < (32 >> (6 - limits)); i++) {
    int outputIndex = (tracker + i) % 32;
    //if (outputIndex < 0) outputIndex += width;
    uint8_t nextY = history[outputIndex] / 2 + 8;
    directLine(width * i, 7, width * i + quarter, nextY);
    directLine(width * i + quarter, nextY, width * i + 3 * quarter, 16 - nextY);
    directLine(width * i + 3 * quarter, 16 - nextY, width * i + 4 * quarter, 7);
  }
}

// Bresenham's line algorithm
void directLine(uint8_t start_x, uint8_t start_y, uint8_t end_x, uint8_t end_y) {
    int dx = abs(end_x - start_x);
    int sx = (start_x < end_x) ? 1 : -1;
    int dy = -1 * abs(end_y - start_y);
    int sy = (start_y < end_y) ? 1 : -1;
    int error = dx + dy;
    
    while(1) {
        matrix.drawPixel(start_x, start_y, colorMap(start_x,start_y));
        if ((start_x == end_x) && (start_y == end_y)) break;
        int e2 = 2 * error;
        if (e2 >= dy) {
            if (start_x == end_x) break;
            error += dy;
            start_x += sx;
        }
        if (e2 <= dx) {
            if (start_y == end_y) break;
            error += dx;
            start_y += sy;
        }
    }
}