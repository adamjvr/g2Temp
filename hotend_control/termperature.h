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

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

// Heater MOSFET Configuration
// 0 = OFF! / 1 = ON!
#define HEATER_0_ENABLE 0
#define HEATER_1_ENABLE 0
#define HEATER_AUX_ENABLE 0
#define HEATBED_ENABLE 0

//Controller Flags
#define NO_OP 0    // No Function Operation
#define OP_CP 1    // Function operation successfully complete


//Stucts
struct device_t {	         // hardware devices that are part of the chip
	uint8_t tick_flag;			  // true = the timer interrupt fired
	uint8_t tick_10ms_count;	// 10ms down counter
	uint8_t tick_100ms_count; // 100ms down counter
	uint8_t tick_1sec_count;	// 1 second down counter
	double pwm_freq;			    // save it for stopping and starting PWM
};
device_t device;				    // Device is always a singleton (there is only one device)

static void temp_control_timer_init(void);
static uint8_t tick_callback_heater_0(void);
static uint8_t tick_callback_heater_1(void);
static uint8_t tick_callback_heater_aux(void);
static uint8_t tick_callback_heatbed(void);
static void tick_1ms_thermocouple_0(void);
static void tick_1ms_thermocouple_1(void);
static void tick_1ms_thermistor(void);
static void tick_1ms_thermistor(void);
//static void tick_10ms(void);
static void tick_100ms_heater_0(void);
static void tick_100ms_heater_1(void);
static void tick_100ms_heater_aux(void);
static void tick_100ms_heatbed(void);


// Function Prototypes
// Runs PID Controller for Heater-0 on Tigershark 3D PROTOTYPE 1
void controller_heater_0();
// Runs PID Controller for Heater-1 on Tigershark 3D PROTOTYPE 1
void controller_heater_1();
// Runs PID Controller for Heater-Aux on Tigershark 3D PROTOTYPE 1
void controller_heater_aux();
// Runs PID Controller for HeatBed on Tigershark 3D PROTOTYPE 1
void controller_heatbed();


#endif
