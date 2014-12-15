/*

# Copyright (c) 2012 - 2013 Robert Giseburt
# Copyright (c) 2013 Alden S. Hart Jr.
# Copyright (c) 2014 Adam Vadala-Roth - 3D printing Extenstions only
#
# This file is part of the TinyG2 project.
#
# This file ("the software") is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License, version 2 as published by the
# Free Software Foundation. You should have received a copy of the GNU General Public
# License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
#
# THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
# WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
# SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
# OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "util.h"
#include "sensors.h"
#include "thermistor_tables.h"

// Thermocouple 0 (MAX31588 Type K) Sensor Sample
static inline double _thermocouple_0_sample(uint8_t adc_channel);
// Thermocouple 1 (MAX31588 Type K) Sensor Sample
static inline double _thermocouple_1_sample(uint8_t adc_channel);
// Thermistor 0 Sensor Sample
static inline double _thermistor_0_sample(uint8_t adc_channel);
// Thermistor 1 Sensor Sample
static inline double _thermistor_1_sample(uint8_t adc_channel);



// Sensor initialization
void thermocouple_0_init(void){
  memset(&thermoucouple_0, 0, sizeof(sensor_t));
	thermocouple_0.temperature = ABSOLUTE_ZERO;
	thermocouple_0.sample_variance_max = SENSOR_SAMPLE_VARIANCE_MAX;
	thermocouple_0.reading_variance_max = SENSOR_READING_VARIANCE_MAX;
	thermocouple_0.disconnect_temperature = SENSOR_DISCONNECTED_TEMPERATURE;
	thermocouple_0.no_power_temperature = SENSOR_NO_POWER_TEMPERATURE;
	// note: there are no bits to set to outputs in this initialization
}


void thermocouple_1_init(void){
  memset(&thermoucouple_1, 0, sizeof(sensor_t));
  thermocouple_1.temperature = ABSOLUTE_ZERO;
  thermocouple_1.sample_variance_max = SENSOR_SAMPLE_VARIANCE_MAX;
  thermocouple_1.reading_variance_max = SENSOR_READING_VARIANCE_MAX;
  thermocouple_1.disconnect_temperature = SENSOR_DISCONNECTED_TEMPERATURE;
  thermocouple_1.no_power_temperature = SENSOR_NO_POWER_TEMPERATURE;
  // note: there are no bits to set to outputs in this initialization
}


void thermistor_0_init(void){
  memset(&thermistor_0, 0, sizeof(sensor_t));
  thermistor_0.temperature = ABSOLUTE_ZERO;
  thermistor_0.sample_variance_max = SENSOR_SAMPLE_VARIANCE_MAX;
  thermistor_0.reading_variance_max = SENSOR_READING_VARIANCE_MAX;
  thermistor_0.disconnect_temperature = SENSOR_DISCONNECTED_TEMPERATURE;
  thermistor_0.no_power_temperature = SENSOR_NO_POWER_TEMPERATURE;
  // note: there are no bits to set to outputs in this initialization
}


void thermistor_1_init(void){
  memset(&thermistor_1, 0, sizeof(sensor_t));
  thermistor_1.temperature = ABSOLUTE_ZERO;
  thermistor_1.sample_variance_max = SENSOR_SAMPLE_VARIANCE_MAX;
  thermistor_1.reading_variance_max = SENSOR_READING_VARIANCE_MAX;
  thermistor_1.disconnect_temperature = SENSOR_DISCONNECTED_TEMPERATURE;
  thermistor_1.no_power_temperature = SENSOR_NO_POWER_TEMPERATURE;
  // note: there are no bits to set to outputs in this initialization
}


// Sensor On/off
void thermocouple_0_ON(void){
  thermoucouple_0.state = SENSOR_NO_DATA;
}


void thermocouple_1_ON(void){
  thermocouple_1.state = SENSOR_NO_DATA;
}


void thermistor_0_ON(void){
  thermistor_0.state = SENSOR_NO_DATA;
}


void thermistor_1_ON(void){
  thermistor_1.state = SENSOR_NO_DATA;
}


void thermocouple_0_OFF(void){
  thermocuple_0.state = SENSOR_OFF;
}


void thermocouple_1_OFF(void){
  thermocouple_1.state = SENSOR_OFF;
}


void thermistor_0_OFF(void){
  thermistor_0.state = SENSOR_OFF;
}

void thermistor_1_OFF(void){
  thermistor_1.state = SENSOR_OFF;
}

//Sensor start reading
void thermocouple_0_read(void){
  thermocouple_0.sample_idx = 0;
	thermocouple_0.code = SENSOR_TAKING_READING;
}

void thermocouple_1_read(void){
  thermocouple_1.sample_idx = 0;
	thermocouple_1.code = SENSOR_TAKING_READING;
}


void thermistor_0_read(void){
	thermistor_0.sample_idx = 0;
	thermistor_0.code = SENSOR_TAKING_READING;
}

void thermistor_1_read(void){
	thermistor_1.sample_idx = 0;
	thermistor_1.code = SENSOR_TAKING_READING;
}


uint8_t thermocouple_0_get_state() { return (thermocouple_0.state);}
uint8_t thermocouple_0_get_code() { return (thermocouple_0.code);}

uint8_t thermocouple_1_get_state() { return (thermocouple_1.state);}
uint8_t thermocouple_1_code() { return (thermocouple_1.code);}

uint8_t thermistor_0_get_state() { return (thermistor_0.state);}
uint8_t thermistor_0_get_code() { return (thermistor_0.code);}

uint8_t thermistor_1_get_state() { return (thermistor_1.state);}
uint8_t thermistor_1_get_code() { return (thermistor_1.code);}


void thermocouple_0_callback(void){
  // cases where you don't execute the callback:
	if ((thermocouple_0.state == SENSOR_OFF) || (thermocouple_0.code != SENSOR_TAKING_READING)) {
		return;
	}

	// get a sample and return if still in the reading period
	thermocouple_0.sample[thermocouple_0.sample_idx] = _sensor_sample(ADC_CHANNEL);
	if ((++thermocouple_0.sample_idx) < SENSOR_SAMPLES) { return; }

	// process the array to clean up samples
	double mean;
	thermocouple_0.std_dev = std_dev(thermocouple_0.sample, SENSOR_SAMPLES, &mean);
	if (thermocouple_0.std_dev > thermocouple_0.reading_variance_max) {
		thermocouple_0.state = SENSOR_ERROR;
		thermocouple_0.code = SENSOR_ERROR_BAD_READINGS;
		return;
	}

	// reject the outlier samples and re-compute the average
	thermocouple_0.samples = 0;
	thermocouple_0.temperature = 0;
	for (uint8_t i=0; i<SENSOR_SAMPLES; i++) {
		if (fabs(thermocouple_0.sample[i] - mean) < (thermocouple_0.sample_variance_max * thermocouple_0.std_dev)) {
			thermocouple_0.temperature += thermocouple_0.sample[i];
			thermocouple_0.samples++;
		}
	}
	thermocouple_0.temperature /= sensor.samples;// calculate mean temp w/o the outliers
	thermocouple_0.state = SENSOR_HAS_DATA;
	thermocouple_0.code = SENSOR_IDLE;			// we are done. Flip it back to idle

	// process the exception cases
	if (thermocouple_0.temperature > SENSOR_DISCONNECTED_TEMPERATURE) {
		thermocouple_0.state = SENSOR_ERROR;
		thermocouple_0.code = SENSOR_ERROR_DISCONNECTED;
	} else if (thermocouple_0.temperature < SENSOR_NO_POWER_TEMPERATURE) {
		thermocouple_0.state = SENSOR_ERROR;
		thermocouple_0.code = SENSOR_ERROR_NO_POWER;
	}
}

void thermocouple_1_callback(void){
  // cases where you don't execute the callback:
  if ((thermocouple_1.state == SENSOR_OFF) || (thermocouple_1.code != SENSOR_TAKING_READING)) {
    return;
  }

  // get a sample and return if still in the reading period
  thermocouple_1.sample[thermocouple_1.sample_idx] = _sensor_sample(ADC_CHANNEL);
  if ((++thermocouple_1.sample_idx) < SENSOR_SAMPLES) { return; }

  // process the array to clean up samples
  double mean;
  thermocouple_1.std_dev = std_dev(thermocouple_1.sample, SENSOR_SAMPLES, &mean);
  if (thermocouple_1.std_dev > thermocouple_1.reading_variance_max) {
    thermocouple_1.state = SENSOR_ERROR;
    thermocouple_1.code = SENSOR_ERROR_BAD_READINGS;
    return;
  }

  // reject the outlier samples and re-compute the average
  thermocouple_1.samples = 0;
  thermocouple_1.temperature = 0;
  for (uint8_t i=0; i<SENSOR_SAMPLES; i++) {
    if (fabs(thermocouple_1.sample[i] - mean) < (thermocouple_1.sample_variance_max * thermocouple_1.std_dev)) {
      thermocouple_1.temperature += thermocouple_1.sample[i];
      thermocouple_1.samples++;
    }
  }
  thermocouple_1.temperature /= sensor.samples;// calculate mean temp w/o the outliers
  thermocouple_1.state = SENSOR_HAS_DATA;
  thermocouple_1.code = SENSOR_IDLE;			// we are done. Flip it back to idle

  // process the exception cases
  if (thermocouple_1.temperature > SENSOR_DISCONNECTED_TEMPERATURE) {
    thermocouple_1.state = SENSOR_ERROR;
    thermocouple_1.code = SENSOR_ERROR_DISCONNECTED;
  } else if (thermocouple_1.temperature < SENSOR_NO_POWER_TEMPERATURE) {
    thermocouple_1.state = SENSOR_ERROR;
    thermocouple_1.code = SENSOR_ERROR_NO_POWER;
  }
}


void thermistor_0_callback(void){

}
void thermistor_1_callback(void){

}



static inline double _thermocouple_0_sample(uint8_t adc_channel)
{
#ifdef __TEST
	double random_gain = 5;
	double random_variation = ((double)(rand() - RAND_MAX/2) / RAND_MAX/2) * random_gain;
	double reading = 60 + random_variation;
	return (((double)reading * SENSOR_SLOPE) + SENSOR_OFFSET);	// useful for testing the math
#else
	return (((double)adc_read() * SENSOR_SLOPE) + SENSOR_OFFSET);
#endif
}


static inline double _thermocouple_1_sample(uint8_t adc_channel)
{
#ifdef __TEST
	double random_gain = 5;
	double random_variation = ((double)(rand() - RAND_MAX/2) / RAND_MAX/2) * random_gain;
	double reading = 60 + random_variation;
	return (((double)reading * SENSOR_SLOPE) + SENSOR_OFFSET);	// useful for testing the math
#else
	return (((double)adc_read() * SENSOR_SLOPE) + SENSOR_OFFSET);
#endif
}
