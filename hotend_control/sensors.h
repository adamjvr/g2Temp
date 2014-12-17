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

#ifndef SENSORS_H
#define SENSORS_H


/******************************************************************************
 * PARAMETERS AND SETTINGS
 ******************************************************************************/

/**** Sensor default parameters ***/

// Thermocouple Sensor Parameters

// THERMOCOUPLE_0
#define THERMOCOUPLE_0_SAMPLES 					20		// number of sensor samples to take for each reading period
#define THERMOUCOUPLE_0_SAMPLE_VARIANCE_MAX 		1.1		// number of standard deviations from mean to reject a sample
#define THERMOCOUPLE_0_READING_VARIANCE_MAX 	20		// reject entire reading if std_dev exceeds this amount
#define THERMOCOUPLE_0_NO_POWER_TEMPERATURE 	-2		// detect thermocouple amplifier disconnected if readings stay below this temp
#define THERMOCOUPLE_0_DISCONNECTED_TEMPERATURE 400		// sensor is DISCONNECTED if over this temp (works w/ both 5v and 3v refs)
#define THERMOCOUPLE_0_TICK_SECONDS 			0.01	// 10 ms

// THERMOCOUPLE_1
#define THERMOCOUPLE_1_SAMPLES 					20		// number of sensor samples to take for each reading period
#define THERMOUCOUPLE_1_SAMPLE_VARIANCE_MAX 		1.1		// number of standard deviations from mean to reject a sample
#define THERMOCOUPLE_1_READING_VARIANCE_MAX 	20		// reject entire reading if std_dev exceeds this amount
#define THERMOCOUPLE_1_NO_POWER_TEMPERATURE 	-2		// detect thermocouple amplifier disconnected if readings stay below this temp
#define THERMOCOUPLE_1_DISCONNECTED_TEMPERATURE 400		// sensor is DISCONNECTED if over this temp (works w/ both 5v and 3v refs)
#define THERMOCOUPLE_1_TICK_SECONDS 			0.01	// 10 ms



// Slope for MAX31588K set for type K
//#define SENSOR_SLOPE 		0.489616568		// derived from AD597 chart between 80 deg-C and 300 deg-C
//
#define THERMOUCOUPLE_0_OFFSET 		-0.10506	// derived from MAX31588 Type K Cold Ref Thermocuple ADC
#define THERMOUCOUPLE_1_OFFSET 		-0.10506	// derived from MAX31588 Type K Cold Ref Thermocuple ADC
                                            // 10.506 uV/C is the MAX31588K offset
#define SURFACE_OF_THE_SUN 	5505			// termperature at the surface of the sun in Celcius
#define HOTTER_THAN_THE_SUN 10000			// a temperature that is hotter than the surface of the sun
#define ABSOLUTE_ZERO 		-273.15			// Celcius
#define LESS_THAN_ZERO 		-274			// Celcius - a value the thermocouple sensor cannot output

enum tcSensorState {						// main state machine
	SENSOR_OFF = 0,							// sensor is off or uninitialized
	SENSOR_NO_DATA,							// interim state before first reading is complete
	SENSOR_ERROR,							// a sensor error occurred. Don't use the data
	SENSOR_HAS_DATA							// sensor has valid data
};

enum tcSensorCode {							// success and failure codes
	SENSOR_IDLE = 0,						// sensor is idling
	SENSOR_TAKING_READING,					// sensor is taking samples for a reading
	SENSOR_ERROR_BAD_READINGS,				// ERROR: too many number of bad readings
	SENSOR_ERROR_DISCONNECTED,				// ERROR: thermocouple detected as disconnected
	SENSOR_ERROR_NO_POWER					// ERROR: detected lack of power to thermocouple amplifier
};

// Structs
// thermocouple sensor struct
struct sensor_t {
  uint8_t state;				// sensor state
	uint8_t code;				// sensor return code (more information about state)
	uint8_t sample_idx;			// index into sample array
	uint8_t samples;			// number of samples in final average
	double temperature;			// high confidence temperature reading
	double std_dev;				// standard deviation of sample array
	double sample_variance_max;	// sample deviation above which to reject a sample
	double reading_variance_max;// standard deviation to reject the entire reading
	double disconnect_temperature;	// bogus temperature indicates thermocouple is disconnected
	double no_power_temperature;	// bogus temperature indicates no power to thermocouple amplifier
	double sample[SENSOR_SAMPLES];	// array of sensor samples in a reading
	double test;
} // End Struct def


//Allocate Sensors
sensor_t themroucouple_0;
sensor_t themroucouple_1;
sensor_t thermistor_0;
sensor_t thermistor_1;


// Function Prototypes
// Sensor initialization
void thermocouple_0_init(void);
void thermocouple_1_init(void);
void thermistor_0_init(void);
void thermistor_1_init(void);

// Sensor On/off
void thermocouple_0_ON(void);
void thermocouple_1_ON(void);
void thermistor_0_ON(void);
void thermistor_1_ON(void);
void thermocouple_0_OFF(void);
void thermocouple_1_OFF(void);
void thermistor_0_OFF(void);
void thermistor_1_OFF(void);

//Sensor start reading
void thermocouple_0_read(void);
void thermocouple_1_read(void);
void thermistor_0_read(void);
void thermistor_1_read(void);

//Sensor Hardware Control
// SPI/CS for MAX31588 Type K Cold Reference Thermocouple ADCs
// ADC Channel's for Thermistors
uint16_t thermocouple_0_adc_read();
uint16_t thermocouple_1_adc_read();
uint16_t thermistor_0_adc_read();
uint16_t thermistor_1_adc_read();

// Getters
// Get sensor state
uint8_t thermocouple_0_get_state(void);
uint8_t thermocouple_1_get_state(void);
uint8_t thermistor_0_get_state(void);
uint8_t thermistor_1_get_state(void);
// Get Sensor Code
uint8_t thermocouple_0_get_code(void);
uint8_t thermocouple_1_get_code(void);
uint8_t thermistor_0_get_code(void);
uint8_t thermistor_1_get_code(void);
// Get Temperature Celscius
double thermocouple_0_get_temp(void);
double thermocouple_1_get_temp(void);
double thermistor_0_get_temp(void);
double thermistor_1_get_temp(void);
// Sensor Callback
extern void thermocouple_0_callback(void);
extern void thermocouple_1_callback(void);
extern void thermistor_0_callback(void);
extern void thermistor_1_callback(void);

#endif
