/** 
 * Copyright 2020, George Spearing, UVM AERO
 * Fault Latcher CS5
 * Get faults from TMS 1 & 2, BMS, IMD
 * Clear with push buttons, send values over CAN
 */

#include <Arduino.h>
#include <mcp_can.h>

// Define CAN Pins
#define PIN_SPI_CAN_CS 5
#define PIN_CAN_INT 2
#define ID_FAULTLATCHER_FAULTS 0X80 // "SEND" CAN Bus address
MCP_CAN CAN(PIN_SPI_CAN_CS); // set CS Pin for CAN

// Fault indicator pins
#define IMD_IND 3
#define TMS2_IND 4
#define TMS1_IND 6
#define BMS_IND 7

// default fault states
// "1" is faulted, "0" is clear
uint16_t BMS_FAULT = 1; // BMS
uint16_t TMS1_FAULT = 1; // Temp for Pack 1
uint16_t TMS2_FAULT = 1; // Temp for Pack 2
uint16_t TMS_Fault = TMS1_FAULT | TMS2_FAULT; // TMS fault for either pack 1/2
uint16_t IMD_FAULT = 1; // IMD

uint16_t lastSendDaqMessage = millis();
uint16_t DAQ_CAN_INTERVAL = 100; // time in ms


void sendDaqData(){
  // turn off interrupts (interrupts not used, but here for consistancy)
  cli();

  uint8_t bufToSend[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  bufToSend[0] = BMS_FAULT;
  bufToSend[1] = TMS1_FAULT && TMS2_FAULT;
  bufToSend[2] = IMD_FAULT;

  // send message
  CAN.sendMsgBuf(ID_FAULTLATCHER_FAULTS, 0, 8, bufToSend);

  // update last send time stamp
  lastSendDaqMessage = millis();

  // reenable interrupts (interrupts not used, but here for consistency)
  sei();

}

void setup() {
  // define pin modes
  pinMode(BMS_IND, INPUT);
  pinMode(TMS1_IND, INPUT);
  pinMode(TMS2_IND, INPUT);
  pinMode(IMD_IND, INPUT);

  // start CAN interface
  CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
  CAN.setMode(MCP_NORMAL);

}

void loop() {
  // if time to send data, sample pins
  if(millis() > (lastSendDaqMessage + DAQ_CAN_INTERVAL)){
    BMS_FAULT = digitalRead(BMS_IND);
    TMS1_FAULT = digitalRead(TMS1_IND);
    TMS2_FAULT = digitalRead(TMS2_IND);
    IMD_FAULT = digitalRead(IMD_IND);

    // send message
    sendDaqData();
  }

}