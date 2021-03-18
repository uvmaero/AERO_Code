/**
 * Copyright 2020, George Spearing, UVM AERO
 * PedalBoard CS5
 * Brake Pressure Sensor, Accelerator Position 
 */

// TODO: Add Wheel Speed Sensors
// TODO: Finish Brake Pressure Sensors

#include <Arduino.h>
#include <EEPROM.h>
#include <mcp_can.h>
#include <SPI.h>

// CAN initilization
#define PIN_SPI_CAN_CS 5
#define PIN_CAN_INT 2
MCP_CAN CAN(PIN_SPI_CAN_CS); // set CS Pin

#define PIN_SPI_ADC_CS 9

// EEPROM address values values
#define EEPROM_BASE         0 // BASE VALUE
#define TORQUE_EEPROM       EEPROM_BASE + sizeof(bool) // max torque stored at base
#define PEDAL0_MIN_EEPROM   TORQUE_EEPROM + sizeof(uint16_t)
#define PEDAL0_MAX_EEPROM   PEDAL0_MIN_EEPROM + sizeof(uint16_t)
#define PEDAL1_MIN_EEPROM   PEDAL0_MAX_EEPROM + sizeof(uint16_t)
#define PEDAL1_MAX_EEPROM   PEDAL1_MIN_EEPROM + sizeof(uint16_t)

// CAN Address
#define ID_PEDAL_BASE         0x30
#define ID_PEDAL_DATA         ID_PEDAL_BASE + 1 // 0X32
#define ID_PEDAL_SET_EEPROM   ID_PEDAL_BASE + 2 // 0X33

#define ID_RINEHART_COMMAND 0xC0 // send for torque commands

#define ID_DASH_RIGHT_DATA 0x74 // Dash vehicle data

// Input Pins
#define PIN_STEER   A0
#define PIN_BRAKE1  A1
#define PIN_BRAKE0  A2
#define PIN_ACC1    A3
#define PIN_ACC0    A4

// default settings
#define PEDAL_DEADBAND 10 // lower limit for '0' value
#define ACC_MAX_SKEW 10 // acc pedal difference limit
// following vlaues based on testing. Can be updated using ID_PEDAL_SET_EEPROM id
uint16_t pedal0_min = 90; // analog read min
uint16_t pedal0_max = 430; // analog read max
uint16_t pedal1_min = 190; // analog read min
uint16_t pedal1_max = 870; // analog read max
uint16_t max_torque = 50; // hard coded default, can be updated over can

#define BRAKE_THRESHOLD 500 // amount of pressure change for brake
uint16_t brake_max_pressure = 3000; // TODO: check if this is correct for sensor / system
bool brakeTrip = false; // for brake light signal
int16_t commanded_torque = 0; // value to send to rinehart. SIGNED!! (x10 factor expected)
int16_t brake_torque = 0; // value for regen brake

// acc raw ADC values from sensors
uint16_t pedal0 = 0, pedal1 = 0, brake0 = 0, brake1 = 0, steer = 0;
// mapped values
uint8_t pedal0_mapped, pedal1_mapped, brake0_mapped, brake1_mapped, steer_mapped,
        pedal_avg, brake_avg;

#define DAQ_INTERVAL 100 // time in ms
uint16_t lastSendDaqMessage;
uint16_t currentMillis;

// default values (updated from DASH)
uint8_t ready_to_drive = 0; // from DASH precharge / start value
uint8_t last_RTD = 0; // placeholder, see if state changed (If )
uint8_t direction = 0; // from DASH direction switch (CS5 runs in "reverse")
uint8_t lastDirection = 0; // placeholder, see if direction changed
uint8_t coast_regen_torque = 10; // from DASH pot
uint8_t brake_regen_torque = 10; // from DASH pot

// values for wheelspeed
uint16_t wheel_right, wheel_left, damper_left, damper_right;

// function declarations
void sampleACC();
void setEERPOM();
void filterCAN(unsigned long canID, unsigned char buf[8]);
void sendRinehartCommand();
void sendDaqData();

void setup() {
  // get current eeprom values
  // set eeprom values to default if none exist
  bool setValue = false; // defaults have been set
  if(!(EEPROM.get(EEPROM_BASE, setValue))){
    EEPROM.update(TORQUE_EEPROM, max_torque);
    EEPROM.update(PEDAL0_MIN_EEPROM, pedal0_min);
    EEPROM.update(PEDAL0_MAX_EEPROM, pedal0_max);
    EEPROM.update(PEDAL1_MIN_EEPROM, pedal1_max);
    EEPROM.update(PEDAL1_MAX_EEPROM, pedal1_max);
    EEPROM.update(EEPROM_BASE, true);
  }else{
    EEPROM.get(TORQUE_EEPROM, max_torque);
    EEPROM.get(PEDAL0_MIN_EEPROM, pedal0_min);
    EEPROM.get(PEDAL0_MAX_EEPROM, pedal0_max);
    EEPROM.get(PEDAL1_MIN_EEPROM, pedal1_min);
    EEPROM.get(PEDAL1_MAX_EEPROM, pedal1_max);
  }

  // set pin modes
  pinMode(PIN_STEER, INPUT);
  pinMode(PIN_ACC0, INPUT);
  pinMode(PIN_ACC1, INPUT);
  pinMode(PIN_BRAKE0, INPUT);
  pinMode(PIN_BRAKE1, INPUT);

  cli();

  //set timer1 interrupt at 100Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 10hz increments
  OCR1A = 156.25;// = (16*10^6) / (1*10240) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 10240 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();

}

void loop() {

  currentMillis = millis();

  // initialize CAN buffers
  unsigned long id; 
  unsigned char len = 0; 
  unsigned char buf[8];
  
  // send Daq
  if(currentMillis - lastSendDaqMessage > DAQ_INTERVAL){
    sendDaqData();
  }

  // read CANbus for incomming messages
    if(CAN_MSGAVAIL == CAN.checkReceive()){
      CAN.readMsgBuf(&id, &len, buf);
      filterCAN(id, buf);
    }
  
}

// create and send Data CAN Message
void sendDaqData(){


  sampleACC();
  sampleBrake();

  // build DAQ Message
  unsigned char bufToSend[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  bufToSend[0] = wheel_left;
  bufToSend[1] = wheel_right;
  bufToSend[2] = damper_left;
  bufToSend[3] = damper_right;
  bufToSend[4] = steer_mapped;
  bufToSend[5] = brake_avg;
  bufToSend[6] = pedal_avg;

  // turn off interrupts
  cli();
  // send message
  CAN.sendMsgBuf(ID_DASH_RIGHT_DATA, 0, 8, bufToSend);

  // update last send time
  lastSendDaqMessage = millis();

  // reenable interrupts
  sei();

}

void sampleBrake(){
  brake0 = analogRead(PIN_BRAKE0);
  brake1 = analogRead(PIN_BRAKE1);

  // map break0 to pressures

  // map break1 to pressures 

  // get average value

  // setup bool variable for break light?

}



// accerlator pedal sampling and comapring
void sampleACC(){
  pedal0 = analogRead(PIN_ACC0);
  pedal1 = analogRead(PIN_ACC1);

  pedal0_mapped = map(pedal0, pedal0_min, pedal0_max, 0, 255);
  pedal1_mapped = map(pedal1, pedal1_min, pedal1_max, 0, 255);

  // check skew value, map to 0 if something is wrong
  if((pedal0_mapped/2+pedal1_mapped/2) >
    (pedal0+ACC_MAX_SKEW || pedal1+ACC_MAX_SKEW)){
    pedal0_mapped = 0;
    pedal1_mapped = 0;
  }

  // average pedal values
  pedal_avg = (pedal0_mapped/2)+(pedal1_mapped/2);

}


// filter CAN messages
void filterCAN(unsigned long canID, unsigned char buf[8]){
  switch(canID){
    case ID_DASH_RIGHT_DATA: // values for rinehart command message
        coast_regen_torque = buf[0];
        brake_regen_torque = buf[1];
        ready_to_drive = buf[3];
        direction = buf[4];
      break;
  
    case ID_PEDAL_SET_EEPROM: // set pedal values and torque limit over can
      max_torque = buf[0];
      pedal0_min = buf[1];
      pedal0_max = buf[2];
      pedal1_min = buf[3];
      pedal1_max = buf[4];
      setEERPOM(); // change occured, update EEPROM values
      break;
  }
}

// set eeprom values from can message
void setEERPOM(){

  // update eeprom values from CAN message
  EEPROM.update(TORQUE_EEPROM, max_torque);
  EEPROM.update(PEDAL0_MIN_EEPROM, pedal0_min);
  EEPROM.update(PEDAL0_MAX_EEPROM, pedal0_max);
  EEPROM.update(PEDAL1_MIN_EEPROM, pedal1_min);
  EEPROM.update(PEDAL1_MAX_EEPROM, pedal1_max);
  EEPROM.update(EEPROM_BASE, true);

}

// create and send Data CAN Message
void sendRinehartCommand(){

  // get current pedal value
  sampleACC();

  // compute commanded torque (N-m * 10 [10x factor is how Rinehart expects it])
  // TODO: should this be a linear map?
  commanded_torque = map(pedal_avg, PEDAL_DEADBAND, 255, 0, 10*max_torque);

  if (pedal_avg < PEDAL_DEADBAND) {
      // enable coasting regen if pedal is below deadband
      commanded_torque = -10*coast_regen_torque;

      // also apply brake regen proportionally to brake pressure
      brake_torque = brake_avg * brake_regen_torque;
      commanded_torque -= brake_torque;
  }

  // if not ready to drive, no torque please.
  if (!ready_to_drive){
    commanded_torque = 0;
  }

  // build DAQ Message
  unsigned char bufToSend[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  bufToSend[0] = commanded_torque && 0xFF; // LSB
  bufToSend[1] = commanded_torque >> 8; // MSB
  bufToSend[2] = 0; // speed command not used
  bufToSend[3] = 0; // speed command not used
  bufToSend[4] = direction; // "0" for reverse, "1" for forward, CS5 is normally "reverse"
  bufToSend[5] = ready_to_drive; // inverter enable 
  bufToSend[6] = max_torque*10 && 0xFF; // LSB
  bufToSend[7] = max_torque*10 >> 8; // MSB

  // send message
  CAN.sendMsgBuf(ID_RINEHART_COMMAND, 0, 8, bufToSend);

}


//timer1 interrupt 100Hz
// send rinehart command for torque
ISR(TIMER1_COMPA_vect){

  cli();
  // switch directions, disable inverter
  if (direction != lastDirection){
    lastDirection = direction; // save new direction
    ready_to_drive = false; // disable interter
    direction = ~direction; // set to old direction 
    sendRinehartCommand(); // send data (dsiable inverter w/ old direction)
    ready_to_drive = true; // turn inverter back on
    direction = lastDirection; // update to current direction
  }

  sendRinehartCommand(); // send command values to rinehart

  sei();

}

// // Consistent interrupt for sampling wheel speed sensors
// ISR(TIMER0_COMPA_vect) {
//     // increment sample counters
//     wsl_samples_since_last++;
//     wsr_samples_since_last++;
//     wsl_debounce_samples++;
//     wsr_debounce_samples++;

//     // LEFT WHEEL
//     if (bit_is_set(PINC, PIN_WHEEL_SPEED_LEFT) && !wsl_detected  // pulse begins
//         && wsl_debounce_samples > WHEEL_SPEED_DEBOUNCE) {
//         wsl_detected = true;
//         uint32_t speed =  // uint32 used because some of the numbers in the calculation get very large
//                 (WHEEL_SPEED_DIST_PER_PULSE / wsl_samples_since_last)   // wheel speed in mils/sample
//             *    WHEEL_SPEED_SAMPLE_RATE                                // converts to mils/second
//             /    MILS_PER_SECOND_TO_MILES_PER_HOUR;                     // converts to mph
//         wheel_speed_left = speed;  // the final result stored in `speed` should fit into a uint8
//         wsl_samples_since_last = 0;
//         wsl_debounce_samples = 0;
//     } else if (bit_is_clear(PINC, PIN_WHEEL_SPEED_LEFT) && wsl_detected  // pulse ends
//                && wsl_debounce_samples > WHEEL_SPEED_DEBOUNCE) {
//         wsl_detected = false;
//         wsl_debounce_samples = 0;
//     }

//     // RIGHT WHEEL
//     if (bit_is_set(PINC, PIN_WHEEL_SPEED_RIGHT) && !wsr_detected  // pulse begins
//         && wsr_debounce_samples > WHEEL_SPEED_DEBOUNCE) {
//         wsr_detected = true;
//         uint32_t speed =  // uint32 used because some of the numbers in the calculation get very large
//                 (WHEEL_SPEED_DIST_PER_PULSE / wsr_samples_since_last)   // wheel speed in mils/sample
//             *    WHEEL_SPEED_SAMPLE_RATE                                // converts to mils/second
//             /    MILS_PER_SECOND_TO_MILES_PER_HOUR;                     // converts to mph
//         wheel_speed_right = speed;  // the final result stored in `speed` should fit into a uint8
//         wsr_samples_since_last = 0;
//         wsr_debounce_samples = 0;
//     } else if (bit_is_clear(PINC, PIN_WHEEL_SPEED_LEFT) && wsr_detected  // pulse ends
//                && wsr_debounce_samples > WHEEL_SPEED_DEBOUNCE) {
//         wsr_detected = false;
//         wsr_debounce_samples = 0;
//     }
// }