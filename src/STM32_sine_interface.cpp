//------------------------------------------------------------------------------
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2011 Philip Court <philip@greenstage.co.nz>
//
//   This file is part of TumanakoVC.
//
//   TumanakoVC is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Lesser General Public License as published
//   by the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   TumanakoVC is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public License
//   along with TumanakoVC.  If not, see <http://www.gnu.org/licenses/>.
//------------------------------------------------------------------------------

/*******************************************************************************
 * File Name          : STM32_interface.cpp
 * Author             : Philip Court
 * Date First Issued  : 2/Jan/2011
 * Description        : Interfce between Tumanako Vehicle control and Sine Motor
 *                      Control.
 *******************************************************************************
 * History:
 * 02/Jan/2011 v1.0 - PCC: First Cut Sine Motor Control interface
 * 13/Mar/2011 v1.0 - PCC: Dash IO and basic sine motor control working (no contactors)
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "STM32_interface.hpp"
#include "tumanako_global.hpp"

#ifdef TUMANAKO_USE_FILTER
#include "filter/filter.hpp"
#endif

extern "C" {
#include "stm32_sine.h" //interface to sine motor control
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/scb.h>
#include <libopencm3/stm32/systick.h>
#include <libopencm3/cm3/common.h>  //u8 etc

  //manage global timer interupt
  static volatile u32 TimeBase_;
  void sys_tick_handler() {
    //See also sysTickInit()

    //With a timer configured for 500usec updates this will cycle a u32 once ~ every 24 days
    //(hence using a single u32 for mulitple timers is fine)
    TimeBase_++;
  }

}

//DIGITAL INPUTS
#define TK_IGN_PIN          GPIO7  //Triggers engagement of contactors (Ignition key on)
#define TK_IGN_PORT         GPIOE
#define TK_START_PIN        GPIO10  //Triggers transition to run mode (Start/Run button pushed)
#define TK_START_PORT       GPIOD
#define TK_CRAWL_PIN        GPIO7  //Triggers motor speed to be held slow
#define TK_CRAWL_PORT       GPIOA
#define TK_FWD_PIN          GPIO11  //Triggers motor to rotate in the forward direction
#define TK_FWD_PORT         GPIOD
#define TK_REV_PIN          GPIO8  //Triggers motor to rotate in reverse
#define TK_REV_PORT         GPIOA

//These pin and port settings clash with the blue led on the board!
#define TK_CONTACTORS_PIN   GPIO7  //Used to read the state of the contactors (TODO not implemented at HW level)
#define TK_CONTACTORS_PORT  GPIOC

#define TK_EMERGENCY_PIN    GPIO8  //Triggers motor speed to be held slo (TODO not implemented at HW level)
#define TK_EMERGENCY_PORT   GPIOC

//DIGITAL OUTPUTS
#define TK_ERROR_LED_PIN    GPIO12  //This output controls the Error LED (Red) on the dashboard
#define TK_ERROR_LED_PORT   GPIOA
#define TK_RUN_LED_PIN      GPIO11  //This output controls the Run LED (Green) on the dashboard
#define TK_RUN_LED_PORT     GPIOA

//#ifdef TUMANAKO_KIWIAC
//Contactor IO (smart card)
//Outputs used to control contactor
#define TK_K_OUT_PORT GPIOB
#define TK_K1_OUT_PIN GPIO8 //precharge (red)
#define TK_K2_OUT_PIN GPIO9 //+ve 750V (yellow)
#define TK_K3_OUT_PIN GPIO10 //-ve 750V (blue)
//brown = ground

//Contactor state feedback
#define TK_K_IN_PORT GPIOC
#define TK_K1_IN_PIN GPIO10 //precharge
#define TK_K2_IN_PIN GPIO11 //+ve 750V
#define TK_K3_IN_PIN GPIO12 //-ve 750V  

//CONSTANTS
#define TK_FILTER_SIZE 20 //increase to make digital signal software more tolerant of noise

STM32Interface::STM32Interface()
#ifdef TUMANAKO_USE_FILTER
    : myFilter_(TK_FILTER_SIZE) //used to filter out noise (to prevent false starting)
#endif
{ }

void STM32Interface::sysTickInit() {
  TimeBase_ = 0;

  // 72MHz / 8 = 9,000,000 counts per second
  //systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);
  // 9000000/9000 = 1000 overflows per second - every 1ms one interrupt
  //systick_set_reload(9000);

  //72MHz = 72,000,000 counts per second
  systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB); //Processor clock, AHB clock(HCLK)
  //Interrupt every 0.5ms
  systick_set_reload(36000);

  systick_counter_enable();

  //PRI 2, SUB_PRI 0
  SCB_SHPR3 &= (u32)0xFF0000; //clear PRI_15 (SysTcik)
  SCB_SHPR3 |= 0x08; //PRI = 2 and SUB PRI =0

  //start counting
  systick_interrupt_enable();
}

//TODO - remove these
unsigned long STM32Interface::getMillisecTimer() {
  return TimeBase_;
}

//TODO - remove these
void STM32Interface::resetMillisecTimer() {
  TimeBase_ = 0;
}

//helper class that allows multiple timers to be created from one TimeBase
MyTimer::MyTimer():myTimeBase_(TimeBase_) {}

void MyTimer::reset() {
  myTimeBase_ = TimeBase_;
}

u32 MyTimer::getElapsed() {
  if (TimeBase_ >= myTimeBase_)
    return TimeBase_ - myTimeBase_;
  else  //TimeBase_ has wrapped around (this is unlikely, it takes 24 days with a 500usec timer and a u32 counter)
    return UINT32_MAX - (myTimeBase_- TimeBase_);
  //This section of logic will fail if TimeBase_ does a full cycle (although this would take in excess of 20 days),     //hence the need for the TUMANAKO_MAX_RUNTIME shutdown timer to ensure logical integrity
}

//returns the scaled bus voltage direct from a single digital conversion
unsigned short STM32Interface::getRawScaledBusVolt() {
  //PCC
  //SKAI 1200 0V in = 0V DC Bus, 9V in = 900V DC Bus (10V = 1000V)
  //SKAI 600 0V in = 0V DC Bus, 9V in = 450V DC Bus

  //KiwiAC max = 10 * 3.3/10.1 (99% of full 3.3V scale)
  //(10 * 3.3/10.1) * 32768 = 32444

  //TODO create SI unit object.  All data exposed via these objects
  // si.eu (scaled enginerring units - fixed point and float versions)
  // si.digital (raw digital value)

  return ((u16)(( (u32)getRawBusVolt() * 1000)/32444));
}

//returns the digital bus voltage A/D conversion without SI unit scaling
unsigned short STM32Interface::getRawBusVolt() {
  return (ADC2_JDR2 >> 3);  //right allignment of injected data register 2
}


// ==== Provide access to various motor control variables ====

unsigned short STM32Interface::getPhaseAOffset() {
  //TODO - part of IFOC
  return 1024;
}
unsigned short STM32Interface::getPhaseBOffset() {
  //TODO - part of IFOC
  return 1024;
}
short STM32Interface::getPhase1() {
  //TODO - part of IFOC
  return 1024;
}
short STM32Interface::getPhase2() {
  //TODO - part of IFOC
  return 1024;
}
short STM32Interface::getPhase3() {
  //TODO - part of IFOC
  return 1024;
}
unsigned long STM32Interface::getRotorTimeConstant() {
  //TODO - part of IFOC
  return 1024;
}
void STM32Interface::setRotorTimeConstant(unsigned long) {
  //TODO - part of IFOC
}
long STM32Interface::getSlipFreq() {
  //TODO - part of IFOC
  return 1024;
}
short STM32Interface::getFluxAngle() {
  //TODO - part of IFOC
  return 1024;
}
short STM32Interface::getElectricalAngle() {
  //TODO - part of IFOC
  return 1024;
}

// ==== IO from the vehcile loom ====

// ==== Physical Inputs ====

bool STM32Interface::getIGN(void) { //Ignition switch on or off?
  return (gpio_get(TK_IGN_PORT, TK_IGN_PIN) == 0);
}

bool STM32Interface::getRAWStart(void) { //the raw digital Start button signal
  return (gpio_get(TK_START_PORT, TK_START_PIN) == 0);
}

bool STM32Interface::getStart(void) { //Start button engaged?
#ifdef TUMANAKO_USE_FILTER
  myFilter_.store(getRAWStart());
  return myFilter_.result();
#else
  return getRAWStart();  //return raw value
#endif
}

bool STM32Interface::getCrawl(void) { //Crawl switch on or off?
  return (gpio_get(TK_CRAWL_PORT, TK_CRAWL_PIN) == 0);
}

bool STM32Interface::getFWD(void) { //Forward selected?
  return (gpio_get(TK_FWD_PORT, TK_FWD_PIN) == 0);
}

bool STM32Interface::getREV(void) { //Reverse selected?
  return (gpio_get(TK_REV_PORT, TK_REV_PIN) == 0);
}

bool STM32Interface::getContactorsEngaged(void) { //Are the contactors engaged (Physical test)?
  return (gpio_get(TK_CONTACTORS_PORT, TK_CONTACTORS_PIN) == 0);
}

bool STM32Interface::getEmergencyStop(void) { //Has the emergency stop been activated?
  return (gpio_get(TK_EMERGENCY_PORT, TK_EMERGENCY_PIN) == 0);
}

// ==== Physical Outputs ====

void STM32Interface::setErrorLED(bool value) { //Show red error light
  if (value == true) gpio_set(TK_ERROR_LED_PORT, TK_ERROR_LED_PIN);
  else gpio_clear(TK_ERROR_LED_PORT, TK_ERROR_LED_PIN);
}

void STM32Interface::setRunLED(bool value) { //Show green run light
  if (value == true) gpio_set(TK_RUN_LED_PORT, TK_RUN_LED_PIN);
  else gpio_clear(TK_RUN_LED_PORT, TK_RUN_LED_PIN);
}

// Reads the value of the specified ADC channel
unsigned short STM32Interface::readADC(unsigned char channel) { //:TODO: needs to be encapsulated into +ve torque and -ve torque etc (channel param not being used!)
  adc_on(ADC1); // If the ADC_CR2_ON bit is already set -> setting it another time starts the conversion
  while (!(ADC_SR(ADC1) & ADC_SR_EOC)) {};  // Waiting for end of conversion 
  return (ADC_DR(ADC1) & 0xFFF);   /* read adc data */
}

void gpioInit() {
#ifdef TUMANAKO_KIWIAC
  AFIO_MAPR |= AFIO_MAPR_TIM1_REMAP_FULL_REMAP;
#endif

  // Set dashboard Run and Error LEDs 'output push-pull'
  gpio_set_mode(TK_ERROR_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, TK_ERROR_LED_PIN);
  gpio_set_mode(TK_RUN_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, TK_RUN_LED_PIN);

  //Contactor output
  gpio_set_mode(TK_K_OUT_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, TK_K1_OUT_PIN | TK_K2_OUT_PIN | TK_K3_OUT_PIN);

  //Contactor feedback
  gpio_set_mode(TK_K_IN_PORT, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, TK_K1_IN_PIN | TK_K2_IN_PIN | TK_K3_IN_PIN);
  gpio_set(TK_K_IN_PORT, TK_K1_IN_PIN | TK_K2_IN_PIN | TK_K3_IN_PIN);  //make it pull up (reset to make pull down)

  //Contactor feedback (TODO not physically implemented yet)
//  gpio_set_mode(TK_CONTACTORS_PORT, GPIO_MODE_INPUT,
//          GPIO_CNF_INPUT_PULL_UPDOWN, TK_CONTACTORS_PIN);
//  gpio_set(TK_CONTACTORS_PORT, TK_CONTACTORS_PIN);  //make it pull up (reset to make pull down)

  //FWD
  gpio_set_mode(TK_FWD_PORT, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, TK_FWD_PIN);
  gpio_set(TK_FWD_PORT, TK_FWD_PIN);  //make it pull up (reset to make pull down)

  //REV
  gpio_set_mode(TK_REV_PORT, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, TK_REV_PIN);
  gpio_set(TK_REV_PORT, TK_REV_PIN);  //make it pull up (reset to make pull down)

  //Crawl
  gpio_set_mode(TK_CRAWL_PORT, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, TK_CRAWL_PIN);
  gpio_set(TK_CRAWL_PORT, TK_CRAWL_PIN);  //make it pull up (reset to make pull down)

  //Start
  gpio_set_mode(TK_START_PORT, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, TK_START_PIN);
  gpio_set(TK_START_PORT, TK_START_PIN);  //make it pull up (reset to make pull down)

}

void STM32Interface::adc_setup(void) {
  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);

  /* make sure it didnt run during config */
  adc_off(ADC1);

  /* we configure everything for one single conversion */
  adc_disable_scan_mode(ADC1);
  adc_set_single_conversion_mode(ADC1);
  adc_enable_discontinous_mode_regular(ADC1);
  adc_disable_external_trigger_regular(ADC1);
  adc_set_right_aligned(ADC1);
  /* we want read out the temperature sensor so we have to enable it */
  adc_enable_temperature_sensor(ADC1);
  adc_set_conversion_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

  adc_on(ADC1);
  /* wait for adc starting up*/
  for (volatile int i = 0; i < 80000; i++) {}; // wait (volitile removes need for -O0 CFLAGS). */

  adc_reset_calibration(ADC1);
  adc_calibration(ADC1);
}

//TODO this has been copied and pasted from sine code (tidy up!)
u8 STM32Interface::adcchfromport(int command_port, int command_bit) {
  /*
   PA0 ADC12_IN0
   PA1 ADC12_IN1
   PA2 ADC12_IN2
   PA3 ADC12_IN3
   PA4 ADC12_IN4
   PA5 ADC12_IN5
   PA6 ADC12_IN6
   PA7 ADC12_IN7
   PB0 ADC12_IN8
   PB1 ADC12_IN9
   PC0 ADC12_IN10
   PC1 ADC12_IN11
   PC2 ADC12_IN12
   PC3 ADC12_IN13
   PC4 ADC12_IN14
   PC5 ADC12_IN15
   temp ADC12_IN16
   */
  switch (command_port) {
  case 0: /* port A */
    if (command_bit<8) return command_bit;
    break;
  case 1: /* port B */
    if (command_bit<2) return command_bit+8;
    break;
  case 2: /* port C */
    if (command_bit<6) return command_bit+10;
    break;
  }
  return 16;
}

/** Used to init the Motor Control libraries*/
int STM32Interface::init(void) {
  //TODO need to consolidate all hardware setup into a single place so Tumanako modules do not conflict
  STM32_SINE_Init();  //can cause conflicts

  sysTickInit();
  gpioInit();

  //analogue init
  static u8 channel_array[16] = {16,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};

  //clock_setup(); this is done in the main
  adc_setup();    /* todo: check setup of analog peripheral*/

  gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO4);
  channel_array[0] = adcchfromport(2,4);
  adc_set_regular_sequence(ADC1, 1, channel_array);

  return 0;
}

// Getter/Setters for the current torque setting (Iq)
signed short STM32Interface::getTorque(void) { //read current value of Torque
  //WARNING this is temp code (motor control for stm32_sine is in tim1_up_isr)
  return STM32_SINE_SineSpeed;  //TODO - add some speed to torque type logic here
}

void STM32Interface::setTorque(signed short torque) { //Set the target Torque value (used in torque control algorithm)
  //WARNING this is temp code (motor control for stm32_sine is in tim1_up_isr)
  STM32_SINE_SineSpeed =  10*torque;  //TODO - add some torque to speed type logic here
}

// Getter/Setters for the current torque setting (Iq)
signed short STM32Interface::getSpeed(void) { //read current value of Torque
  //TODO
  return 1024;
}
void STM32Interface::setSpeed(signed short) { //Set the target Torque value (used in torque control algorithm)
  //TODO
}

// Getter/Setters for the current flux setting (Id)
signed short STM32Interface::getFlux(void) { //read current value of rotor Flux
  //TODO
  return 1024;
}
void STM32Interface::setFlux(signed short) { //Set the target rotor Flux value (used in torque control algorithm)
  //TODO
}

// Getter/Setters for the current speed (RPM) setting
signed short STM32Interface::getRPM(void) { //read current value of motor RPM
  //TODO
  return 1024;
}
void STM32Interface::setRPM(signed short) { //Set the target RPM (speed control loop only)
  //TODO
}

//Is the bus voltage OK? (TODO - more work needed here.  Must expose limits etc...)
bool STM32Interface::busVoltageOK(void) {
  unsigned short busVolt = getRawScaledBusVolt();

  //  check bus over voltage
  if ((busVolt >= TUMANAKO_PRECHARGE_V) && (busVolt <= TUMANAKO_MAX_BUX_V ))
    return true;
  else
    return true;
}

//Get current bus voltage (via a historical 16 reading average)
short STM32Interface::busVoltage(void) {
  //TODO:PCC
  return 1024;
}

//Get current temperature from power stage
short STM32Interface::powerStageTemperature(void) {
  //TODO
  return 1024;
}

short STM32Interface::motorTemperature(void) {
  //TODO
  return 1024;
}


//returns 0 if no issues
short STM32Interface::testVariousMotorParam(void) {
  unsigned short busVolt = getRawScaledBusVolt();

  //TODO make enum

  //  check over temperature of power stage
  if (false) { //TODO
    return 1; //OVERHEAT;
  }

  //  check bus over voltage
  if (busVolt > TUMANAKO_MAX_BUX_V ) {
    return 2; //OVER_VOLTAGE
  }

#ifdef TUMANAKO_UNDERVOLTAGE_TEST
  //  check bus under voltage
  if (busVolt < TUMANAKO_PRECHARGE_V) //should never go below precharge min voltage
#else
  if (false)  //dont test undervoltage
#endif
  {
    return 3; //UNDER_VOLTAGE
  }
  return 0; //All OK
}

//Contactor controls (See diagram here: http://liionbms.com/php/precharge.php)
bool STM32Interface::getK1() {
  return (gpio_get(TK_K_IN_PORT, TK_K1_IN_PIN) == 0);
}

bool STM32Interface::getK2() {
  return (gpio_get(TK_K_IN_PORT, TK_K2_IN_PIN) == 0);
}

bool STM32Interface::getK3() {
  return (gpio_get(TK_K_IN_PORT, TK_K3_IN_PIN) == 0);
}

void STM32Interface::setK1(bool status) {
  if (status == true) gpio_set(TK_K_OUT_PORT, TK_K1_OUT_PIN);
  else gpio_clear(TK_K_OUT_PORT, TK_K1_OUT_PIN);
}

void STM32Interface::setK2(bool status) {
  if (status == true) gpio_set(TK_K_OUT_PORT, TK_K2_OUT_PIN);
  else gpio_clear(TK_K_OUT_PORT, TK_K2_OUT_PIN);
}

void STM32Interface::setK3(bool status) {
  if (status == true) gpio_set(TK_K_OUT_PORT, TK_K3_OUT_PIN);
  else gpio_clear(TK_K_OUT_PORT, TK_K3_OUT_PIN);
}

//Blocking Wait. Wait specified number of milliseconds (ms)
void STM32Interface::wait(unsigned short time) {
  MyTimer timer;
  u32 waitTime = time;

  while (timer.getElapsed() < waitTime) { //wait NOP
    asm(
      "mov     r0, r0\n\t"
      "mov     r0, r0\n\t"
      "mov     r0, r0\n\t"
      "mov     r0, r0"
    );
  }
}

//Sanity checks (TODO paramatise this)
void STM32Interface::checkPowerStageLimits() {
  //TODO
}

//Prep motor for start (TODO document details)
void STM32Interface::motorInit() {
  //TODO
}

//Start motor
void STM32Interface::motorStart() {
  TIM1_BDTR |= TIM_BDTR_MOE; //enable TIM1 main outputs

  //Check motor starts and rotation occurs when reasonable torque is applied
#ifdef DONT_INCLUDE
  if (speed != 0) {
    // turn LED off to show startup time has succeeded?
  }
  //shutdown power if torque figure is large enough to be concerned at lack of rotation (after suitable wait)
  else if ((torqueRef > STM_STALL_TORQUE_LIMIT) || (torqueRef < -STM_STALL_TORQUE_LIMIT)) {
    // turn LED on to show startup time has expired and torque figure > than threshold
    if (startTimeElapsed()) {
      //Go to fault, reason is speed feedback
    }
  } else //Reset the startup timer (because the torque applied is low)
#endif
  }

//test to be executed in the main loop when motor is running (TODO document details)
void STM32Interface::motorTestForSpeedError() {
  //TODO see motorStart logic
}

//Shutdown motor control and power stage
void STM32Interface::shutdownPower() {
  //Disable the PWM main outputs
  TIM1_BDTR &= (uint16_t)~(TIM_BDTR_MOE); //main output enable OFF

  //Explicitly disconnect contactors
  setK1(false);
  setK2(false);
  setK3(false);
}


