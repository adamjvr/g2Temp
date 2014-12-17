/*
Copyright (c) 2012 - 2013 Robert Giseburt
Copyright (c) 2013 Alden S. Hart Jr.
Copyright (c) 2014 Adam Vadala-Roth - 3D printing Extenstions only

 This file is part of the TinyG2 project.

 This file ("the software") is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License, version 2 as published by the
 Free Software Foundation. You should have received a copy of the GNU General Public
 License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.

 THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// Includes
#include "temperature.h"
#include "heaterpid.h"
#include "sensors.h"

// Loop Controller Boolean flags
static bool _heater_0_run_FLAG = false;
static bool _heater_1_run_FLAG = false;
static bool _heater_aux_run_FLAG = false;
static bool _heatbed_run_FLAG = false;

timer_number kTempControllerTimer = 0;
timer_number kTempControllerTimerChan = 0;

TimerChannel<kTempControllerTimer, kTempControllerTimerChan> TempTimer(kTimerUpToMatch, /*Hz: */ ISR_freq);

// Runs PID Controller for Heater-0 on Tigershark 3D PROTOTYPE 1
void controller_heater_0(){
  _heater_0_run_FLAG = true;
  heater_0_init();              // Initialized Heater 0 Device (Nozzle 0)
  thermocouple_0_init();        // Initialize Thermocouple Sensor 0
  sprintf((char *)nv.token, "g%2d%c", 53+i, ("Heater-0 Activated")[j]);  // print Heater Status
  while(_heater_0_run_FLAG==true){
    tick_callback_heater_0();
  }
}// end method

// Runs PID Controller for Heater-1 on Tigershark 3D PROTOTYPE 1
void controller_heater_1(){
  _heater_1_run_FLAG = true;
  heater_1_init();              // Initialized Heater 1 Device (Nozzle 1)
  thermocouple_1_init();        // Initialize Thermocouple Sensor 0
  sprintf((char *)nv.token, "g%2d%c", 53+i, ("Heater-0 Activated")[j]); // print Heater Status
  while(_heater_1_run_FLAG==true){
    tick_callback_heater_1();
  }
}// end method

// Runs PID Controller for Heater-Aux on Tigershark 3D PROTOTYPE 1
void controller_heater_aux(){
  _heater_aux_run_FLAG true;
  heater_aux_init();            // Initialized Aux Heater Device (build chamber heater)
  thermistor_0_init();          // Initialize Thermistor Sensor 0 ATSAM ADC
  sprintf((char *)nv.token, "g%2d%c", 53+i, ("Aux Heater Activated")[j]); // print Heater Status
  while(_heater_aux_run_FLAG==true){
    tick_callback_heater_aux();
  }
}// end method

// Runs PID Controller for Heatbed on Tigershark 3D PROTOTYPE 1
void controller_heatbed(){
  _heatbed_run_FLAG true;
  heatbed_init();               // Initialized Heatbed Device
  thermistor_0_init();          // Initialize Thermistor Sensor 1 ATSAM ADC
  sprintf((char *)nv.token, "g%2d%c", 53+i, ("Heatbed Activated")[j]); // print Heater Status
  while(_heatbed_run_FLAG==true){
    tick_callback_heatbed();
  } // end while
}// end method


void _stop_controller_heater_0(){
  _heater_0_run_FLAG = false;
  heater_0_OFF();
}

void _stop_controller_heater_1(){
  _heater_1_run_FLAG = false;
  heater_1_OFF();
}

void _stop_controller_heater_aux(){
  _heater_aux_run_FLAG = false;
  heater_aux_OFF();
}

void _stop_controller_heatbedv(){
  _heatbed_run_FLAG = false;
  heatbed_OFF();
}

//Motate Intterrupt Controller
MOTATE_TIMER_INTERRUPT(void){
  int16_t interrupted_channel;
  TimerChannelInterruptOptions interrupt_cause = getInterruptCause(interrupted_channel);
  if (interrupt_cause == kInterruptOnMatch) {
    device.tick_flag = true;
  } else if (interrupt_cause == kInterruptOnOverflow) {
    device.tick_flag = false;
  }// end else-if
}// end interrupt controller


//MOTATE timer which one is free for this ?
void temp_control_timer_init(void){
  TempTimer.setInterrupts(kInterruptOnMatch|kInterruptOnOverflow);
  TempTimer.setDutyCycle(ISR_duty_cycle);
  TempTimer.start();
  device.tick_10ms_count = 10;
	device.tick_100ms_count = 10;
	device.tick_1sec_count = 10;
  device.pwm_freq =
}  // end method

static void tick_1ms(void){  // 1ms callout
  sensor_callback();
}  // end method

static void tick_100ms(void){  // 100ms callout
	heater_callback();
}  // end method

static uint8_t tick_callback_heater_0(void)
{
  if (device.tick_flag == false) { return (NO_OP);}

  device.tick_flag = false;
  tick_1ms();

  if (--device.tick_10ms_count != 0) { return (OP_CP);}
  device.tick_10ms_count = 10;
  tick_10ms();

  if (--device.tick_100ms_count != 0) { return (OP_CP);}
  device.tick_100ms_count = 10;
  tick_100ms();

  if (--device.tick_1sec_count != 0) { return (OP_CP);}
  device.tick_1sec_count = 10;
  tick_1sec();

  return (OP_CP);
}  // end method

static uint8_t tick_callback_heater_1(void)
{
  if (device.tick_flag == false) { return (NO_OP);}

  device.tick_flag = false;
  tick_1ms();

  if (--device.tick_10ms_count != 0) { return (OP_CP);}
  device.tick_10ms_count = 10;
  tick_10ms();

  if (--device.tick_100ms_count != 0) { return (OP_CP);}
  device.tick_100ms_count = 10;
  tick_100ms();

  if (--device.tick_1sec_count != 0) { return (OP_CP);}
  device.tick_1sec_count = 10;
  tick_1sec();

  return (OP_CP);
}  // end method


static uint8_t tick_callback_heater_aux(void)
{
  if (device.tick_flag == false) { return (NO_OP);}

  device.tick_flag = false;
  tick_1ms();

  if (--device.tick_10ms_count != 0) { return (OP_CP);}
  device.tick_10ms_count = 10;
  tick_10ms();

  if (--device.tick_100ms_count != 0) { return (OP_CP);}
  device.tick_100ms_count = 10;
  tick_100ms();

  if (--device.tick_1sec_count != 0) { return (OP_CP);}
  device.tick_1sec_count = 10;
  tick_1sec();

  return (OP_CP);
}  // end method


static uint8_t tick_callback_heatbed(void)
{
  if (device.tick_flag == false) { return (NO_OP);}

  device.tick_flag = false;
  tick_1ms();

  if (--device.tick_10ms_count != 0) { return (OP_CP);}
  device.tick_10ms_count = 10;
  tick_10ms();

  if (--device.tick_100ms_count != 0) { return (OP_CP);}
  device.tick_100ms_count = 10;
  tick_100ms();

  if (--device.tick_1sec_count != 0) { return (OP_CP);}
  device.tick_1sec_count = 10;
  tick_1sec();

  return (OP_CP);
}  // end method

static void tick_1ms_thermocouple_0(void){
  thermocouple_0_callback(void);
}  // end method

static void tick_1ms_thermocouple_1(void){
  thermocouple_1_callback(void);
}  // end method

static void tick_1ms_thermistor_0(void){
  thermistor_0_callback(void);
}  // end method

static void tick_1ms_thermistor_1(void){
  thermistor_1_callback(void);
}  // end method

static void tick_100ms_heater_0(void){
  heater_0_callback(void);
}  // end method

static void tick_100ms_heater_1(void){
  heater_1_callback(void);
}  // end method

static void tick_100ms_heater_aux(void){
  heater_aux_callback(void);
}  // end method

static void tick_100ms_heatbed(void){
  heatbed_callback(void);
}  // end method
