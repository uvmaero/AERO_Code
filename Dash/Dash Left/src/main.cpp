/**
 * Copyright 2020, George Spearing, UVM AERO
 * Dash Left CS5
 * Temperature LEDs
 * Battery SOC LED
 * LCD Screen
 */

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <mcp_can.h>

// CAN Setup
#define PIN_SPI_CAN_CS 5 // CAN SPI Chip
#define CAN_INT 2 // interrupt pin
MCP_CAN CAN(PIN_SPI_CAN_CS); // set CS Pin
#define DAQ_CAN_INTERVAL 100 // time in ms
uint16_t lastSendDaqMessage;

// CAN Address
#define ID_BASE 0x76
#define ID_DASH_SELF_TEST 0x72 // SAME AS DASH RIGHT

// Initialize LCD Screen
#define PIN_LCD_BTN  8 // selector pin for lcd screen
#define RS A0
#define EN A1
#define D4 A2
#define D5 A3
#define D6 A4
#define D7 A5
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7); // start lcd screen
const int numRows = 2; // LCD number of rows
const int numCols = 16; // LCD number of columns

// Initialize LED Chip
#define PIN_LED_OE 3
#define PIN_LED_LE 6
#define PIN_LED_CLK 4
#define PIN_LED_SDI 7
#define STP16_DELAY 1 // time in us, delay needed for bit-bang operation
uint16_t ext_leds = 0; // register value

// LED PIN Values fo STP16 chip

// CAN Address Inputs
// Battery Temp
// Motor Temp
// RineHart Temp
// Battery Voltage

void selfTest();
void filterCAN(unsigned long canID, unsigned char buf[8]);
void setLED(int ledNum, bool state);
void printToLCD(uint8_t menuOption);

void filterCAN(unsigned long canID, unsigned char buf[8]){
  switch(canID){
    case ID_DASH_SELF_TEST:
    selfTest();
    break;
    // motor temp, rinehart temp, battery temp, SOC
  
  }
}

// print value to lcd screen
void print_to_lcd(uint8_t menuOption){
  switch(menuOption){
    case 0: // welcome
      lcd.home();
      lcd.print("Welcome");
      lcd.setCursor(2,1);
      lcd.print("AERO");
      break;
    
    case 1: // battery 
      lcd.home();
      lcd.print("Battery Status  ");
      lcd.setCursor(0,1);
      lcd.print(uint16_t(batteryVoltage));
      lcd.print("V     ");
      lcd.setCursor(10,1);
      lcd.print(batterySOC);
      lcd.print("%");
      break;
    
    case 2: // temps
      lcd.home();
      lcd.print("Temeratures     ");
      lcd.setCursor(0,1);
      lcd.print("B:     ");
      lcd.setCursor(5,1);
      lcd.print("M:     ");
      lcd.setCursor(10,1);
      lcd.print("C:     ");
      break;
    
    case 3: // self Test
      lcd.home();
      lcd.print("     TESTING    ");
      lcd.setCursor(0,1);
      lcd.print("     TESTING    ");
      break;
  }
}

void selfTest(){
  // turn off interrupts
  cli();

  // Turn off all LEDs

  // Clear Display

  // Turn on LEDs one by one

  // show message on screen

  // Test Button Push

  // reenable interrupts
  sei();

}

void setLED(int ledNum, bool state){

  // setup data to send
  ext_leds = (ext_leds & (~(1<<ledNum))) | (state << ledNum); 

  // send data bits
  for(int i=15; i>=0; i--){
    digitalWrite(PIN_LED_CLK, LOW); // turn off chip clock
    delayMicroseconds(STP16_DELAY);
    digitalWrite(PIN_LED_SDI, ((ext_leds >> i) & 1)); // write next bit
    delayMicroseconds(STP16_DELAY);
    digitalWrite(PIN_LED_CLK, HIGH); // turn clock back on
    delayMicroseconds(STP16_DELAY);
  }

  // clear latched data
  digitalWrite(PIN_LED_LE, HIGH);
  delayMicroseconds(STP16_DELAY);
  digitalWrite(PIN_LED_LE, LOW);

  // enable output
  digitalWrite(PIN_LED_OE, LOW);

}


// Temperature Sampling

// Battery SOC Calculation

// LCD Screen output

void setup() {
  
  // Pin setup
  pinMode(PIN_LED_CLK, OUTPUT);
  pinMode(PIN_LED_LE, OUTPUT);
  pinMode(PIN_LED_OE, OUTPUT);
  pinMode(PIN_LED_SDI, OUTPUT);

  pinMode(PIN_LCD_BTN, INPUT);
  pinMode(8, OUTPUT);

  // STP16 pins, diable latch and output
  digitalWrite(PIN_LED_LE, LOW);
  digitalWrite(PIN_LED_OE, HIGH);

  // Initialize LCD Screen
  lcd.begin(16, 2);
  lcd.clear();
  lcd.noCursor();
  lcd.setCursor(2,0);
  lcd.print("Welcome");
  lcd.setCursor(2,1);
  lcd.print("AERO");

  // Initialize CAN
  // IF USING CAN INTERRUPT PIN, UNCOMMENT THIS:
  // pinMode(CAN_INT, INPUT);
  CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
  CAN.setMode(MCP_NORMAL);


}


void loop() {
  
  // initialize CAN buffers
  unsigned long id; 
  unsigned char len = 0;
  unsigned char buf[8];

  // read CANbus messages
  if(CAN_MSGAVAIL == CAN.checkError()){
    CAN.readMsgBuf(&id, &len, buf);
    filterCAN(id, buf);
  }

}
