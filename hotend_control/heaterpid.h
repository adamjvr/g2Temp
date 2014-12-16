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

#ifndef HEATERPID_H_ONCE
#define HEATERPID_H_ONCE

/******************************************************************************
 * PARAMETERS AND SETTINGS
 ******************************************************************************/

/**** Heater default parameters ***/

#define HEATER_0_TICK_SECONDS 		0.1		// 100 ms
#define HEATER_0_HYSTERESIS 			10		// number of successive readings before declaring heater at-temp or out of regulation
#define HEATER_0_AMBIENT_TEMPERATURE	40		// detect heater not heating if readings stay below this temp
#define HEATER_0_OVERHEAT_TEMPERATURE 300		// heater is above max temperature if over this temp. Should shut down
#define HEATER_0_AMBIENT_TIMEOUT 		90		// time to allow heater to heat above ambinet temperature (seconds)
#define HEATER_0_REGULATION_RANGE 	3		// +/- degrees to consider heater in regulation
#define HEATER_0_REGULATION_TIMEOUT 	300		// time to allow heater to come to temp (seconds)
#define HEATER_0_BAD_READING_MAX 		5		// maximum successive bad readings before shutting down

/**** Heater default parameters ***/

#define HEATER_1_TICK_SECONDS 		0.1		// 100 ms
#define HEATER_1_HYSTERESIS 			10		// number of successive readings before declaring heater at-temp or out of regulation
#define HEATER_1_AMBIENT_TEMPERATURE	40		// detect heater not heating if readings stay below this temp
#define HEATER_1_OVERHEAT_TEMPERATURE 300		// heater is above max temperature if over this temp. Should shut down
#define HEATER_1_AMBIENT_TIMEOUT 		90		// time to allow heater to heat above ambinet temperature (seconds)
#define HEATER_1_REGULATION_RANGE 	3		// +/- degrees to consider heater in regulation
#define HEATER_1_REGULATION_TIMEOUT 	300		// time to allow heater to come to temp (seconds)
#define HEATER_1_BAD_READING_MAX 		5		// maximum successive bad readings before shutting down

/**** Heater default parameters ***/

#define HEATER_AUX_TICK_SECONDS 		0.1		// 100 ms
#define HEATER_AUX_HYSTERESIS 			10		// number of successive readings before declaring heater at-temp or out of regulation
#define HEATER_AUX_AMBIENT_TEMPERATURE	40		// detect heater not heating if readings stay below this temp
#define HEATER_AUX_OVERHEAT_TEMPERATURE 300		// heater is above max temperature if over this temp. Should shut down
#define HEATER_AUX_AMBIENT_TIMEOUT 		90		// time to allow heater to heat above ambinet temperature (seconds)
#define HEATER_AUX_REGULATION_RANGE 	3		// +/- degrees to consider heater in regulation
#define HEATER_AUX_REGULATION_TIMEOUT 	300		// time to allow heater to come to temp (seconds)
#define HEATER_AUX_BAD_READING_MAX 		5		// maximum successive bad readings before shutting down

/**** Heater default parameters ***/

#define HEATBED_TICK_SECONDS 		0.1		// 100 ms
#define HEATBED_HYSTERESIS 			10		// number of successive readings before declaring heater at-temp or out of regulation
#define HEATBED_AMBIENT_TEMPERATURE	40		// detect heater not heating if readings stay below this temp
#define HEATBED_OVERHEAT_TEMPERATURE 300		// heater is above max temperature if over this temp. Should shut down
#define HEATBED_AMBIENT_TIMEOUT 		90		// time to allow heater to heat above ambinet temperature (seconds)
#define HEATBED_REGULATION_RANGE 	3		// +/- degrees to consider heater in regulation
#define HEATBED_REGULATION_TIMEOUT 	300		// time to allow heater to come to temp (seconds)
#define HEATBED_BAD_READING_MAX 		5		// maximum successive bad readings before shutting down

enum tcHeaterState {						// heater state machine
	HEATER_OFF = 0,							// heater turned OFF or never turned on - transitions to HEATING
	HEATER_SHUTDOWN,						// heater has been shut down - transitions to HEATING
	HEATER_HEATING,							// heating up from OFF or SHUTDOWN - transitions to REGULATED or SHUTDOWN
	HEATER_REGULATED						// heater is at setpoint and in regulation - transitions to OFF or SHUTDOWN
};

enum tcHeaterCode {
	HEATER_OK = 0,							// heater is OK - no errors reported
	HEATER_AMBIENT_TIMED_OUT,				// heater failed to get past ambient temperature
	HEATER_REGULATION_TIMED_OUT,			// heater heated but failed to achieve regulation before timeout
	HEATER_OVERHEATED,						// heater exceeded maximum temperature cutoff value
	HEATER_SENSOR_ERROR						// heater encountered a fatal sensor error
};

/**** PID default parameters ***/

#define PID_DT 				HEATER_TICK_SECONDS	// time constant for PID computation
#define PID_EPSILON 		0.1				// error term precision
#define PID_MAX_OUTPUT 		100				// saturation filter max PWM percent
#define PID_MIN_OUTPUT 		0				// saturation filter min PWM percent

#define PID_Kp 				5.00			// proportional gain term
#define PID_Ki 				0.1 			// integral gain term
#define PID_Kd 				0.5				// derivative gain term
#define PID_INITIAL_INTEGRAL 200			// initial integral value to speed things along

// some starting values from the example code
//#define PID_Kp 0.1						// proportional gain term
//#define PID_Ki 0.005						// integral gain term
//#define PID_Kd 0.01						// derivative gain term

enum tcPIDState {							// PID state machine
	PID_OFF = 0,							// PID is off
	PID_ON
};

// Data Structures


// Heater Struct
struct heater_t {
	uint8_t state;				// heater state
	uint8_t code;				// heater code (more information about heater state)
	uint8_t	toggle;
	int8_t hysteresis;			// number of successive readings in or out or regulation before changing state
	uint8_t bad_reading_max;	// sets number of successive bad readings before declaring an error
	uint8_t bad_reading_count;	// count of successive bad readings
	double temperature;			// current heater temperature
	double setpoint;			// set point for regulation
	double regulation_range;	// +/- range to consider heater in regulation
	double regulation_timer;	// time taken so far in a HEATING cycle
	double ambient_timeout;		// timeout beyond which regulation has failed (seconds)
	double regulation_timeout;	// timeout beyond which regulation has failed (seconds)
	double ambient_temperature;	// temperature below which it's ambient temperature (heater failed)
	double overheat_temperature;// overheat temperature (cutoff temperature)
};

// PID Struct
struct pid_struct {
  uint8_t state;
  uint8_t code;
  double output;
  double output_max;
  double output_min;
  double error;
  double prev_error;
  double integral;
  double derivative;
  //
  double Kp;
  double Ki;
  double Kd;
};


// allocations
static heater_t heater_0;      // allocate one heater...
static heater_t heater_1;     // Allocate second heater
static heater_t heater_aux;         // Allocate aux heater
static heater_t heatbed;
static pid_struct pid_heater_0;						// allocate one PID channel...
static pid_strudt pid_heater_1;
static pid_struct pid_heater_aux;
static pid_struct pid_heatbed;

/******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

void heater_0_init(void);
void heater_1_init(void);
void heater_aux_init(void);
void heatbed_init(void);

static void heater_0_ON(double setpoint);
static void heater_1_ON(double setpoint);
static void heater_aux_ON(double setpoint);
static void heatbed_ON(double setpoint);

static void heater_0_OFF(uint8_t state, uint8_t code);
static void heater_1_OFF(uint8_t state, uint8_t code);
static void heater_aux_OFF(uint8_t state, uint8_t code);
static void heatbed_OFF(uint8_t state, uint8_t code);

extern void heater_0_callback(void);
extern void heater_1_callback(void);
extern void heater_aux_callback(void);
extern void heatbed_callback(void);

static void set_pwm(double pwm_duty_cycle,uint8_t chan);



static void pid_heater0_init();
static void pid_heater1_init();
static void pid_heaterAux_init();
static void pid_heatbed_init();

static void pid_heater0_reset();
static void pid_heater1_reset();
static void pid_heaterAux_reset();
static void pid_heatbed_reset();

static double pid_heater0_calculate(double setpoint,double temperature);
static double pid_heater1_calculate(double setpoint,double temperature);
static double pid_heaterAux_calulate(double setpoint,double temperature);
static double pid_heatbed_calculae(double setpoint,double temperature);


#endif	// End of include
