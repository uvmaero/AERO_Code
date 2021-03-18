/**
 * Copyright 2020, George Spearing, UVM AERO
 * Dash Right CS5
 * Fault LEDs, Start button, cooling switch, direction
 * coast / regen pots for rinehart
 * Precharge
 */

// TODO: Finish Precharge Function

#include <Arduino.h>
#include <mcp_can.h>

// CAN Setup
#define PIN_SPI_CAN_CS 5 // CAN SPI Chip
#define CAN_INT 2 // interrupt pin
MCP_CAN CAN(PIN_SPI_CAN_CS); // set CS Pin
#define DAQ_CAN_INTERVAL 100 // time in ms
uint16_t lastSendDaqMessage = millis();

// CAN Addresses
#define ID_BASE             0x71
#define ID_DASH_SELF_TEST   ID_BASE +1
#define ID_DASH_STATUS      ID_BASE +2
#define ID_DASH_RIGHT_DATA  ID_BASE +3

#define ID_FAULTLATCHER 0x80 // receive faults

// Input Pins
#define PIN_COAST A0 // coast regen value
#define PIN_BRAKE A1 // brake regen value
#define PIN_DIRECTION A2 // motor direction switch
#define PIN_COOLING A3 // cooling switch value
#define PIN_RTD_BTN 3 // input "start button"
#define PIN_SENSOR_1 8 // extra digital sensor

// output pins
#define PIN_BMS_LED 4 // bms fault indicator
#define PIN_IMD_LED 6 // imd fault indicator
#define PIN_TMS_LED 7 // tms fault indicator
#define PIN_RTD_LED 9 // 'start button' LED
#define PIN_RTD_IND 10 // buzzer (low side mosfet)
// create array for fault pins
uint8_t faultLED[] = {PIN_BMS_LED, PIN_IMD_LED, PIN_TMS_LED,
                  PIN_RTD_LED, PIN_RTD_IND};
// RTDS, buzzer, souding period (ms)
#define RTDS_PERIOD 2000
// RTDS on time (ms)
uint16_t time_since_rtds_start = 0;
bool rtds_on = false;
bool rtdLED_on = false;

// dash Self Test Variables
#define SELFTEST_DELAY 1000 // time in ms
#define SELFTESET_POT_WORKING_THRESHOLD 50 // threshold value for pot change
#define SELFTEST_TIMEOUT 20000 // time in ms

// initialize input values
uint16_t coastRegen, brakeRegen;
bool coolEnable, rtdButton;
bool direction; // CS5 drives in 'reverse (0)'
uint8_t coast_mapped;
uint8_t brake_mapped;

// Precharge values
bool ready_to_drive = false; // ready to drive, precharge done, buzzer done
bool rtds_on = false; // buzzer sound
bool precharge_state_enter = false; // state change

// precharge State
enum precharge_state_enum {
  PRECHARGE_OFF,
  PRECHARGE_ON,
  PRECHARGE_DONE,
  PRECHARGE_ERROR
};
precharge_state_enum precharge_state = PRECHARGE_OFF;

// precharge coefficient -- rinehart voltage must be more than emus voltage * coefficient
// in order to finish precharge
#define PRECHARGE_COEFFICIENT 0.9

// Maximum number of missed messages before canceling precharge
#define MAX_MISSED_EMUS_MESSAGES 10
#define MAX_MISSED_RINEHART_MESSAGES 100

// Emus and Rinhart message timeout values
uint8_t cycles_since_last_emus_message = 0;
// assume Rinehart connection hasn't been made yet
uint8_t cycles_since_last_rinehart_message = MAX_MISSED_RINEHART_MESSAGES;  

// Rinehart Pins
#define RINEHART_PIN_TSMS 4
#define RINEHART_PIN_PRECHARGE 0
#define RINEHART_PIN_MAIN_CONT 1
#define RINEHART_PIN_RTDS 2

// voltages
// hardcoded voltage, should be taken from CAN data
uint16_t emus_voltage = 265.0;  
uint16_t rinehart_voltage = 0;

// function declarations
void selfTest(); 
void filterCAN(unsigned long canID, unsigned char buf[8]); 
void sendDaqData(); 
void buttonChange();

void setup() {
  pinMode(PIN_COAST, INPUT);
  pinMode(PIN_BRAKE, INPUT);
  pinMode(PIN_DIRECTION, INPUT);
  pinMode(PIN_COOLING, INPUT);
  pinMode(PIN_RTD_BTN, INPUT);
  pinMode(PIN_SENSOR_1, INPUT);

  pinMode(PIN_BMS_LED, OUTPUT);
  pinMode(PIN_IMD_LED, OUTPUT);
  pinMode(PIN_TMS_LED, OUTPUT);
  pinMode(PIN_RTD_LED, OUTPUT);
  pinMode(PIN_RTD_IND, OUTPUT);

  // button interrupt
  attachInterrupt(digitalPinToInterrupt(3), buttonChange, RISING);


  cli();
  //set timer1 interrupt at 10Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1562.5;// = (16*10^6) / (1*10240) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 10240 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();

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

  // read CANbus for incomming messages
  if(CAN_MSGAVAIL == CAN.checkReceive()){
    CAN.readMsgBuf(&id, &len, buf);
    filterCAN(id, buf);
  }

  // // LED indicators
  // digitalWrite(PIN_RTD_LED, rtdLED_on); // start button light
  
}


// filter CAN messages
void filterCAN(unsigned long canID, unsigned char buf[8]){
  switch(canID){
    case ID_DASH_SELF_TEST:
      selfTest();
      break;
    case ID_FAULTLATCHER:
      digitalWrite(PIN_BMS_LED, buf[0]);
      digitalWrite(PIN_TMS_LED, buf[1]);
      digitalWrite(PIN_IMD_LED, buf[2]);
      break;
  }
}

// create and send Data CAN Message
void sendDaqData(){
  // turn off interrupts
  cli();

  // sample inputs
  coastRegen = analogRead(PIN_COAST);
  brakeRegen = analogRead(PIN_BRAKE); 
  coolEnable = digitalRead(PIN_COOLING);
  direction = digitalRead(PIN_DIRECTION);
  rtdButton = digitalRead(PIN_RTD_BTN);

  coast_mapped = map(coastRegen, 0, 1024, 0, 255);
  brake_mapped = map(brakeRegen, 0, 1024, 0, 255);

  // build DAQ Message
  unsigned char bufToSend[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  bufToSend[0] = coast_mapped;
  bufToSend[1] = brake_mapped;
  bufToSend[2] = coolEnable;
  bufToSend[3] = ready_to_drive;
  bufToSend[4] = direction;
  bufToSend[5] = rtdButton;

  // send message
  CAN.sendMsgBuf(ID_DASH_RIGHT_DATA, 0, 8, bufToSend);

  // update last send time
  lastSendDaqMessage = millis();

  // reenable interrupts
  sei();


}

// Test Right Side Dash Hardware, user input required
void selfTest(){
  // turn off interrupts
  cli();

  uint8_t i=0; // used for all for loops

  // Turn off LEDs
  for(i=0; i<sizeof(faultLED); i++){
    digitalWrite(i, LOW); // turn off
  }

  // turn on one-by-one
  for(i=0; i<sizeof(faultLED); i++){
    digitalWrite(i, HIGH); // turn on
    delay(SELFTEST_DELAY);
    digitalWrite(i, LOW); // turn off
  }

  // test buttons / inputs
  bool coastKnobWorking = false, brakeKnobWorking = false, 
  coolingSwitchWorking = false, directionSwitchWorking = false,
  rtdBtnWorking = false;
  
  // only continue once all switches / buttons / knobs have been tested

  uint16_t coastKnobInitialValue = analogRead(PIN_COAST);
  uint16_t brakeKnobInitialValue = analogRead(PIN_BRAKE);
  bool coolingSwitchInitialValue = digitalRead(PIN_COOLING);
  bool directionSwitchInitialValue = digitalRead(PIN_DIRECTION);
  bool rtdButtonInitialValue = digitalRead(PIN_RTD_BTN);

  // Turn on leds for switch / pot test
  for(i=0; i<sizeof(faultLED); i++){
    digitalWrite(i, HIGH);
  }

  while(!coastKnobWorking || !brakeKnobWorking || 
  !coolingSwitchWorking || !directionSwitchWorking || !rtdBtnWorking){

    // check rtd button
    if(rtdButtonInitialValue != digitalRead(PIN_RTD_BTN)){
      rtdBtnWorking = true;
      digitalWrite(PIN_RTD_IND, LOW);
    }

    // check cooling
    if(coolingSwitchInitialValue != digitalRead(PIN_COOLING)){
      coolingSwitchWorking = true;
      digitalWrite(PIN_BMS_LED, LOW);
    }

    // check direction
    if(directionSwitchInitialValue != digitalRead(PIN_DIRECTION)){
      directionSwitchWorking = true;
      digitalWrite(PIN_IMD_LED, LOW);
    }

    // check pots, both have to change for led to turn off. 
    if(abs(coastKnobInitialValue-analogRead(PIN_COAST))> SELFTESET_POT_WORKING_THRESHOLD &&
      abs(brakeKnobInitialValue-analogRead(PIN_BRAKE)) > SELFTESET_POT_WORKING_THRESHOLD){
      coastKnobWorking = true;
      brakeKnobWorking = true;
      digitalWrite(PIN_TMS_LED, LOW);
    }

    // delay in while loop
    delay(5); // check every 5 miliseconds

  }

  // reenable interrupts
  sei();

}



// interrupts

// precharge interrupt

void set_precharge_state(enum precharge_state_enum state){
  precharge_state = state;
  precharge_state_enter = true;
}

// control state of precharge
void control_precharge(){

  // missed too many rinehart messages = ERROR
  if(cycles_since_last_rinehart_message >= MAX_MISSED_RINEHART_MESSAGES){
    precharge_state = PRECHARGE_ERROR;
  }

  // missed too many emus messages = ERROR
  if(cycles_since_last_emus_message >= MAX_MISSED_EMUS_MESSAGES){
    precharge_state = PRECHARGE_ERROR;
  }

  // switch precharge state
  switch(precharge_state){

    // PRECHARGE OFF 
    case(PRECHARGE_OFF):

      // turn off rinehart outputs

      // vehicle not ready to drive
      ready_to_drive = false;

      // if TMS is still ready, try PRECHARGE Again 

    
    break;

    // PRECHARGE ON
    case(PRECHARGE_ON):

      // try to precharge rinehart

      // if TMS is not ready, go to PRECHARGE OFF

      // vechile not ready to drive (yet)
      ready_to_drive = false;

      // check voltages, if above threashold, then precharge is done
      
        // turn on Start button led
        rtdLED_on = true;
        // switch to PRECHARGE DONE
        set_precharge_state(PRECHARGE_DONE);


    // PRECHARGE DONE

      // If just entered, turn off outputs

      // if TMS is not on, switch to PRECHARGE OFF


    // PRECHARGE_ERROR

      // Turn off rinehart outputs

      // vehicle not ready to drive

      // If both devices have been heard from, return to PRECHARGE OFF

  } // end switch

  // Increment emus missed messages

  // Increment rinehart missed messages

  // send precharge message

} // end control_precharge

// button interrupt, change state and activate buzzer
void buttonChange(){
  if(precharge_state == PRECHARGE_DONE && rtdLED_on){
    rtdLED_on = false; // turn start button led off
    rtds_on = true; // turn buzzer on
    time_since_rtds_start = 0; // reset buzzer time
  }
}

// 10Hz Timer Interrupt -- Precharge Control
ISR(TIMER1_COMPA_vect){ // check if it's timer one or whatnot

  // control precharge status
  control_precharge();

  // if precharge done, light up start button
  if(!ready_to_drive && precharge_state == PRECHARGE_DONE){
    digitalWrite(PIN_RTD_LED, rtdLED_on);
  }

  // if start button is then pressed, sound buzzer, increment buzzer time
  if(rtds_on && time_since_rtds_start <= RTDS_PERIOD && precharge_state == PRECHARGE_DONE){
    time_since_rtds_start += 100;
  }
  // if precharge done, and buzzer done, turn on HV (ready to drive)
  if(rtds_on && precharge_state==PRECHARGE_DONE && time_since_rtds_start > RTDS_PERIOD){
    rtds_on = false; // trun off buzzer
    ready_to_drive = true; // now we're ready to drive (for pedal board)
  }

}
