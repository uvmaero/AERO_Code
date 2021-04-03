/**
 * Copyright 2020, George Spearing, UVM AERO
 * Data Acquisition Board CS5
 * (Rear DAQ w/ fan and brake control)
 */


#include <Arduino.h>
#include <mcp_can.h>

// CAN Settings
#define PIN_SPI_CAN_CS 5 // CAN SPI Chip
#define CAN_INT 2 // interrupt pin
MCP_CAN CAN(PIN_SPI_CAN_CS); // Set CS Pin
#define DAQ_CAN_INTERVAL 100 // time in ms
uint16_t lastSendDaqMessage = millis();

// CAN Address
#define ID_BASE 0x01
#define ID_REAR_DAQ_DATA ID_BASE + 1 // send data on this address
#define ID_DASH_RIGHT_DATA 0x74 // receive data from dash
#define ID_FRONT_PEDALBOARD 0x37 // receive data from pedal

// Input Pins
#define PIN_WHEEL_L A0
#define PIN_WHEEL_R A1
#define PIN_DAMPER_L A2
#define PIN_DAMPER_R A3
#define PIN_SENSOR_5 A4 // cooling temperature sensor
#define PIN_SENSOR_6 A5
#define PIN_SENSOR_1 6
#define PIN_SENSOR_2 7

// Output Pins
#define PIN_FAN_OUT 3
#define PIN_BRAKE_OUT 4

// intialize output values
bool brakeSig = false;
int fanSig;
bool autoTemp = false;


// wheel speed sampling

// damper position sampling

// brake light trigger

// fan trigger
// includes auto speed sensing and pwm signal 

void sendDaqData(){
  // send wheel and damper positions

  // turn off interrupts
  cli();

  // sample wheels and damper and temperature
  sampleSensors();
  // build daq message

  // send message

  // reenable interrupts
  sei();

}

void sampleSensors(){
  // sample damper positions
  // sample wheel sensors
  // sample temperature
}

void TempControl(){
    // if auto temp is 'true' enable temperature controlled fan interrupt
  // calculate temperature and base fan PWM speed off this.
}

void filterCAN(unsigned long canID, unsigned char buf[8]){
  switch(canID){
    case ID_DASH_RIGHT_DATA:
      autoTemp = buf[2];
      // digitalWrite(PIN_FAN_OUT, autoTemp) // write fan output 'high' or 'low'
      break;
    case ID_FRONT_PEDALBOARD:
      brakeSig = buf[5];
      // digitalWrite(PIN_BRAKE_OUT, brakeSig) // write brake output 'high' or 'low'
      break;
  }
}

void setup() {
  // setup PinMode
  pinMode(PIN_WHEEL_L, INPUT);
  pinMode(PIN_WHEEL_R, INPUT);
  pinMode(PIN_DAMPER_L, INPUT);
  pinMode(PIN_DAMPER_R, INPUT);
  pinMode(PIN_SENSOR_1, INPUT);
  pinMode(PIN_SENSOR_2, INPUT);
  pinMode(PIN_SENSOR_5, INPUT);
  pinMode(PIN_SENSOR_6, INPUT);

  pinMode(PIN_FAN_OUT, OUTPUT);
  pinMode(PIN_BRAKE_OUT, OUTPUT);

  // IF USING INTERRUPT PIN UNCOMMENT THIS CODE:
  // pinMode(CAN_INT, INPUT)

  // Initialize CANbus Interface
  CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
  CAN.setMode(MCP_NORMAL);
}

void loop() {

  // Initialize can buffers
  unsigned long id;
  unsigned char len = 0;
  unsigned char buf[8];

  // read the CANbus for data, filter as needed
  if (CAN_MSGAVAIL == CAN.checkReceive()){
    CAN.readMsgBuf(&id, &len, buf);
    filterCAN(id, buf);
  }

// interrupts

// brake light 10Hz


  

}