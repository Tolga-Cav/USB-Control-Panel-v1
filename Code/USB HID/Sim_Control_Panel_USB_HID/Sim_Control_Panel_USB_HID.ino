#include "Wire.h"    // imports the wire library for talking over I2C
#include "I2Cdev.h"
#include "TCA6424A.h"
#include "Adafruit_TLC5947.h"

// Define the TI TCA6424A 24-bit I/O extender
TCA6424A tca(TCA6424A_ADDRESS_ADDR_LOW);
void buttonRead(){
  uint8_t ibytes[3]; // byte components of the full input
  tca.readAll(ibytes);
  Serial.print(ibytes[0], BIN);
  Serial.print(".");
  Serial.print(ibytes[1], BIN);
  Serial.print(".");
  Serial.println(ibytes[2], BIN);
}

// How many boards do you have chained?
#define NUM_TLC5947 1

#define data   19
#define clock   18
#define latch   16
#define oe  -1  // set to -1 to not use the enable pin (it
Adafruit_TLC5947 tlc = Adafruit_TLC5947(NUM_TLC5947, clock, data, latch);


#define ROTA 3  // GPIO1 - Encoder A
#define ROTB 2  // GPIO0 - Encoder B

volatile int encoder_val = 0;
volatile uint8_t encoder_state = 0;
static int last_val = 0;
volatile bool flag = false;

void readEncoder() {
  flag = true;
  encoder_state = (encoder_state << 4) | (digitalRead(ROTB) << 1) | digitalRead(ROTA);

  switch (encoder_state) {
    case 0x23: encoder_val++; break;  // Clockwise
    case 0x32: encoder_val--; break;  // Counter-clockwise
    default: break;
  }
}

void setup() {
  //initialize the serial output
  Serial.begin(9600);
  delay(3000);
  Serial.println("Analog Test");
  Wire.begin(); //Join I2C bus
  Serial.println("Setup Complete");
  analogReadResolution(12);

  //
  //  TCS6424A
  //

  // initialize device
  Serial.println("Initializing TCA6424A device...");
  tca.initialize();

  // verify connection
  Serial.println("Testing device connection...");
  if (tca.testConnection()) {
    Serial.println("Connection successful!...");
  }
  else {
    Serial.println("Connection failed!...");
    digitalWrite(LED_BUILTIN, HIGH);
    while (1);
  }

  // set all of the pins as input
  tca.setAllDirection(0xFF,0xFF,0xFF);

  tlc.begin();
  if (oe >= 0) {
    pinMode(oe, OUTPUT);
    digitalWrite(oe, LOW);
  }

  pinMode(ROTA, INPUT);
  pinMode(ROTB, INPUT);

  attachInterrupt(digitalPinToInterrupt(ROTA), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTB), readEncoder, CHANGE);
  
}


void loop() {
  // buttonRead();
  // Serial.println("Measure");
  // Serial.println(analogRead(A0));
  // Serial.println(analogRead(A1));
  // Serial.println(analogRead(A2));
  // delay(2000);

  // colorWipe(4095, 0, 0, 100); // "Red" (depending on your LED wiring)
  // delay(200);
  // colorWipe(0, 4095, 0, 100); // "Green" (depending on your LED wiring)
  // delay(200);
  // colorWipe(0, 0, 4095, 100); // "Blue" (depending on your LED wiring)
  // delay(200);
  Serial.println(encoder_val);
  if(flag){
    Serial.print(".");
    Serial.println(flag);
    flag = false;
  }
  delay(20);
}



// Fill the dots one after the other with a color
void colorWipe(uint16_t r, uint16_t g, uint16_t b, uint8_t wait) {
  for(uint16_t i=0; i<8*NUM_TLC5947; i++) {
      tlc.setLED(i, r, g, b);
      tlc.write();
      delay(wait);
  }
}
