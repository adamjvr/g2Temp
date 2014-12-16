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

#include "heaterpid.h"
#include "util.h"
#include <string.h>
#include <math.h>

/**** Heater Functions ****/
/*
 * heater_init() - initialize heater with default values
 * heater_on()	 - turn heater on
 * heater_off()	 - turn heater off
 * heater_callback() - 100ms timed loop for heater control
 *
 *	heater_init() sets default values that may be overwritten via Kinen communications.
 *	heater_on() sets initial values used regardless of any changes made to settings.
 */
void heater_0_init()
{
	// initialize heater, start PID and PWM
	// note: PWM and ADC are initialized as part of the device init
	memset(&heater_0, 0, sizeof(heater_t));
	heater_0.regulation_range = HEATER_REGULATION_RANGE;
	heater_0.ambient_timeout = HEATER_AMBIENT_TIMEOUT;
	heater_0.regulation_timeout = HEATER_REGULATION_TIMEOUT;
	heater_0.ambient_temperature = HEATER_AMBIENT_TEMPERATURE;
	heater_0.overheat_temperature = HEATER_OVERHEAT_TEMPERATURE;
	heater_0.bad_reading_max = HEATER_BAD_READING_MAX;
	thermocouple_0_init();
	pid_heater0_init();
}

void heater_1_init()
{
  // initialize heater, start PID and PWM
  // note: PWM and ADC are initialized as part of the device init
  memset(&heater_1, 0, sizeof(heater_t));
  heater_1.regulation_range = HEATER_REGULATION_RANGE;
  heater_1.ambient_timeout = HEATER_AMBIENT_TIMEOUT;
  heater_1.regulation_timeout = HEATER_REGULATION_TIMEOUT;
  heater_1.ambient_temperature = HEATER_AMBIENT_TEMPERATURE;
  heater_1.overheat_temperature = HEATER_OVERHEAT_TEMPERATURE;
  heater_1.bad_reading_max = HEATER_BAD_READING_MAX;
  thermocouple_1_init();
  pid_heater1_init();
}

void heater_aux_init()
{
  // initialize heater, start PID and PWM
  // note: PWM and ADC are initialized as part of the device init
  memset(&heater_aux, 0, sizeof(heater_t));
  heater_aux.regulation_range = HEATER_REGULATION_RANGE;
  heater_aux.ambient_timeout = HEATER_AMBIENT_TIMEOUT;
  heater_aux.regulation_timeout = HEATER_REGULATION_TIMEOUT;
  heater_aux.ambient_temperature = HEATER_AMBIENT_TEMPERATURE;
  heater_aux.overheat_temperature = HEATER_OVERHEAT_TEMPERATURE;
  heater_aux.bad_reading_max = HEATER_BAD_READING_MAX;
  thermistor_0_init();
  pid_heaterAux_init();
}

void heatbed_init()
{
  // initialize heater, start PID and PWM
  // note: PWM and ADC are initialized as part of the device init
  memset(&heatbed, 0, sizeof(heater_t));
  heatbed.regulation_range = HEATER_REGULATION_RANGE;
  heatbed.ambient_timeout = HEATER_AMBIENT_TIMEOUT;
  heatbed.regulation_timeout = HEATER_REGULATION_TIMEOUT;
  heatbed.ambient_temperature = HEATER_AMBIENT_TEMPERATURE;
  heatbed.overheat_temperature = HEATER_OVERHEAT_TEMPERATURE;
  heatbed.bad_reading_max = HEATER_BAD_READING_MAX;
  thermistor_1_init();
  pid_heatbed_init();
}

void heater_0_ON(double setpoint)
{
	// no action if heater is already on
	if ((heater_0.state == HEATER_HEATING) || (heater_0.state == HEATER_REGULATED)) {
		return;
	}
	// turn on lower level functions
	thermocouple_0_ON();						// enable the sensor
	thermocouple_0_read();				// now start a reading
	pid_heater0_reset();

  // Replace with ARM equivalent
  ///pwm_on(PWM_FREQUENCY, 0);			// duty cycle will be set by PID loop

	// initialize values for a heater cycle
	heater_0.setpoint = setpoint;
	heater_0.hysteresis = 0;
	heater_0.bad_reading_count = 0;
	heater_0.regulation_timer = 0;		// reset timeouts
	heater_0.state = HEATER_HEATING;
}

void heater_1_ON(double setpoint)
{
  // no action if heater is already on
  if ((heater_1.state == HEATER_HEATING) || (heater_1.state == HEATER_REGULATED)) {
    return;
  }
  // turn on lower level functions
  thermocouple_1_ON();						// enable the sensor
  thermocouple_1_read();				// now start a reading
  pid_heater1_reset();

  // Replace with ARM equivalent
  ///pwm_on(PWM_FREQUENCY, 0);			// duty cycle will be set by PID loop

  // initialize values for a heater cycle
  heater_1.setpoint = setpoint;
  heater_1.hysteresis = 0;
  heater_1.bad_reading_count = 0;
  heater_1.regulation_timer = 0;		// reset timeouts
  heater_1.state = HEATER_HEATING;
}

void heater_aux_ON(double setpoint)
{
  // no action if heater is already on
  if ((heater_aux.state == HEATER_HEATING) || (heater_aux.state == HEATER_REGULATED)) {
    return;
  }
  // turn on lower level functions
  thermistor_0_ON();						// enable the sensor
  thermistor_0_read();				// now start a reading
  pid_heaterAux_reset();

  // Replace with ARM equivalent
  ///pwm_on(PWM_FREQUENCY, 0);			// duty cycle will be set by PID loop

  // initialize values for a heater cycle
  heater_aux.setpoint = setpoint;
  heater_aux.hysteresis = 0;
  heater_aux.bad_reading_count = 0;
  heater_aux.regulation_timer = 0;		// reset timeouts
  heater_aux.state = HEATER_HEATING;
}

void heatbed_ON(double setpoint)
{
  // no action if heater is already on
  if ((heatbed.state == HEATER_HEATING) || (heatbed.state == HEATER_REGULATED)) {
    return;
  }
  // turn on lower level functions
  thermistor_1_ON();						// enable the sensor
  thermistor_1_read();				// now start a reading
  pid_heatbed_reset();
  // Replace with ARM equivalent
  ///pwm_on(PWM_FREQUENCY, 0);			// duty cycle will be set by PID loop

  // initialize values for a heater cycle
  heatbed.setpoint = setpoint;
  heatbed.hysteresis = 0;
  heatbed.bad_reading_count = 0;
  heatbed.regulation_timer = 0;		// reset timeouts
  heatbed.state = HEATER_HEATING;
}



void heater_0_off(uint8_t state, uint8_t code)
{
	// Replace with function that shuts off PWM on PC22
  set_pwm(0.0,0);					 // stop sending current to the heater
  thermocouple_0_OFF();		// stop taking readings
	heater_0.state = state;
	heater_0.code = code;

}

void heater_1_off(uint8_t state, uint8_t code)
{
  // Replace with function that shuts off PWM on PC21
  set_pwm(0.0,1);							// stop sending current to the heater
	thermocouple_1_OFF();   // stop taking readings
  heater_1.state = state;
  heater_1.code = code;

}

void heater_aux_off(uint8_t state, uint8_t code)
{
  // Replace with function that shuts off PWM on PC20
  set_pwm(0.0,2);							// stop sending current to the heater
  thermistor_0_OFF();						// stop taking readings
  heater_aux.state = state;
  heater_aux.code = code;
}


void heatbed_off(uint8_t state, uint8_t code)
{
  // Replace with function that shuts off PWM on PB25
  set_pwm(0.0,3);							// stop sending current to the heater
  thermistor_1_OFF();		 // stop taking readings
  heatbed.state = state;
  heatbed.code = code;
}

void heater_0_callback()
{
	// catch the no-op cases
	if ((heater_0.state == HEATER_OFF) || (heater_0.state == HEATER_SHUTDOWN)) { return;}

  //rpt_readout(); Replace with TinyG2 compliant data printout code

	// get current temperature from the sensor
	heater_0.temperature = thermocouple_0_get_temp();

	// trap overheat condition
	if (heater_0.temperature > heater_0.overheat_temperature) {
		heater_0_OFF(HEATER_SHUTDOWN, HEATER_OVERHEATED);
		return;
	}

	thermocouple_0_read();				// start reading for the next interval

	// handle bad readings from the sensor
	if (heater_0.temperature < ABSOLUTE_ZERO) {
		if (++heater_0.bad_reading_count > heater_0.bad_reading_max) {
			heater_0_OFF(HEATER_SHUTDOWN, HEATER_SENSOR_ERROR);
		//	printf_P(PSTR("Heater Sensor Error Shutdown\n"));
		}
		return;
	}
	heater_0.bad_reading_count = 0;		// reset the bad reading counter

	double duty_cycle = pid_heater0_calculate(heater_0.setpoint, heater_0.temperature);
	set_pwm(duty_cycle,0);

	// handle HEATER exceptions
	if (heater_0.state == HEATER_HEATING) {
		heater_0.regulation_timer += HEATER_TICK_SECONDS;

		if ((heater_0.temperature < heater_0.ambient_temperature) &&
			(heater_0.regulation_timer > heater_0.ambient_timeout)) {
			heater_0_OFF(HEATER_SHUTDOWN, HEATER_AMBIENT_TIMED_OUT);
		//	printf_P(PSTR("Heater Ambient Error Shutdown\n"));
			return;
		}
		if ((heater_0.temperature < heater_0.setpoint) &&
			(heater_0.regulation_timer > heater_0.regulation_timeout)) {
			heater_0_OFF(HEATER_SHUTDOWN, HEATER_REGULATION_TIMED_OUT);
		//	printf_P(PSTR("Heater Timeout Error Shutdown\n"));
			return;
		}
	}

	// Manage regulation state and LED indicator
	// Heater.regulation_count is a hysteresis register that increments if the
	// heater is at temp, decrements if not. It pegs at max and min values.
	// The LED flashes if the heater is not in regulation and goes solid if it is.

  // check on C++ vs C differences for fabs math.h call
	if (fabs(pid_heater_0.error) <= heater_0.regulation_range) {
		if (++heater_0.hysteresis > HEATER_HYSTERESIS) {
			heater_0.hysteresis = HEATER_HYSTERESIS;
			heater_0.state = HEATER_REGULATED;
		}
	} else {
		if (--heater_0.hysteresis <= 0) {
			heater_0.hysteresis = 0;
			heater_0.regulation_timer = 0;			// reset timeouts
			heater_0.state = HEATER_HEATING;
		}
	}


	if (heater_0.state == HEATER_REGULATED) {
		//led_on()
	} else {
		if (++heater_0.toggle > 3) {
			heater_0.toggle = 0;
			//led_toggle();
		}
	}
}// end function

void heater_1_callback()
{
  // catch the no-op cases
  if ((heater_1.state == HEATER_OFF) || (heater_1.state == HEATER_SHUTDOWN)) { return;}

  //rpt_readout(); Replace with TinyG2 compliant data printout code

  // get current temperature from the sensor
  heater_1.temperature = thermocouple_1_get_temp();

  // trap overheat condition
  if (heater_1.temperature > heater_1.overheat_temperature) {
    heater_1_OFF(HEATER_SHUTDOWN, HEATER_OVERHEATED);
    return;
  }

  thermocouple_1_read();				// start reading for the next interval

  // handle bad readings from the sensor
  if (heater_1.temperature < ABSOLUTE_ZERO) {
    if (++heater_1.bad_reading_count > heater_1.bad_reading_max) {
      heater_1_OFF(HEATER_SHUTDOWN, HEATER_SENSOR_ERROR);
    //	printf_P(PSTR("Heater Sensor Error Shutdown\n"));
    }
    return;
  }
  heater_1.bad_reading_count = 0;		// reset the bad reading counter

  double duty_cycle = pid_heater1_calculate(heater_1.setpoint, heater_1.temperature);
  set_pwm(duty_cycle,1);

  // handle HEATER exceptions
  if (heater_1.state == HEATER_HEATING) {
    heater_1.regulation_timer += HEATER_TICK_SECONDS;

    if ((heater_1.temperature < heater_1.ambient_temperature) &&
      (heater_1.regulation_timer > heater_1.ambient_timeout)) {
      heater_1_OFF(HEATER_SHUTDOWN, HEATER_AMBIENT_TIMED_OUT);
    //	printf_P(PSTR("Heater Ambient Error Shutdown\n"));
      return;
    }
    if ((heater_1.temperature < heater_1.setpoint) &&
      (heater_1.regulation_timer > heater_1.regulation_timeout)) {
      heater_1_OFF(HEATER_SHUTDOWN, HEATER_REGULATION_TIMED_OUT);
    //	printf_P(PSTR("Heater Timeout Error Shutdown\n"));
      return;
    }
  }

  // Manage regulation state and LED indicator
  // Heater.regulation_count is a hysteresis register that increments if the
  // heater is at temp, decrements if not. It pegs at max and min values.
  // The LED flashes if the heater is not in regulation and goes solid if it is.

  // check on C++ vs C differences for fabs math.h call
  if (fabs(pid_heater_1.error) <= heater_1.regulation_range) {
    if (++heater_1.hysteresis > HEATER_HYSTERESIS) {
      heater_1.hysteresis = HEATER_HYSTERESIS;
      heater_1.state = HEATER_REGULATED;
    }
  } else {
    if (--heater_1.hysteresis <= 0) {
      heater_1.hysteresis = 0;
      heater_1.regulation_timer = 0;			// reset timeouts
      heater_1.state = HEATER_HEATING;
    }
  }


  if (heater_1.state == HEATER_REGULATED) {
    //led_on()
  } else {
    if (++heater_1.toggle > 3) {
      heater_1.toggle = 0;
      //led_toggle();
    }
  }
}// end function

void heater_aux_callback()
{
  // catch the no-op cases
  if ((heater_aux.state == HEATER_OFF) || (heater_aux.state == HEATER_SHUTDOWN)) { return;}

  //rpt_readout(); Replace with TinyG2 compliant data printout code

  // get current temperature from the sensor
  heater_aux.temperature = thermistor_0_get_temp();

  // trap overheat condition
  if (heater_aux.temperature > heater_aux.overheat_temperature) {
    heater_aux_OFF(HEATER_SHUTDOWN, HEATER_OVERHEATED);
    return;
  }

  thermistor_0_read();				// start reading for the next interval

  // handle bad readings from the sensor
  if (heater_aux.temperature < ABSOLUTE_ZERO) {
    if (++heater_aux.bad_reading_count > heater_aux.bad_reading_max) {
      heater_aux_OFF(HEATER_SHUTDOWN, HEATER_SENSOR_ERROR);
    //	printf_P(PSTR("Heater Sensor Error Shutdown\n"));
    }
    return;
  }
  heater_aux.bad_reading_count = 0;		// reset the bad reading counter

  double duty_cycle = pid_heaterAux_calculate(heater_aux.setpoint, heater_aux.temperature);
  set_pwm(duty_cycle,2);

  // handle HEATER exceptions
  if (heater_aux.state == HEATER_HEATING) {
    heater_aux.regulation_timer += HEATER_TICK_SECONDS;

    if ((heater_aux.temperature < heater_aux.ambient_temperature) &&
      (heater_aux.regulation_timer > heater_aux.ambient_timeout)) {
      heater_aux_OFF(HEATER_SHUTDOWN, HEATER_AMBIENT_TIMED_OUT);
    //	printf_P(PSTR("Heater Ambient Error Shutdown\n"));
      return;
    }
    if ((heater_aux.temperature < heater_aux.setpoint) &&
      (heater_aux.regulation_timer > heater_aux.regulation_timeout)) {
      heater_aux_OFF(HEATER_SHUTDOWN, HEATER_REGULATION_TIMED_OUT);
    //	printf_P(PSTR("Heater Timeout Error Shutdown\n"));
      return;
    }
  }

  // Manage regulation state and LED indicator
  // Heater.regulation_count is a hysteresis register that increments if the
  // heater is at temp, decrements if not. It pegs at max and min values.
  // The LED flashes if the heater is not in regulation and goes solid if it is.

  // check on C++ vs C differences for fabs math.h call
  if (fabs(pid_heater_aux.error) <= heater_aux.regulation_range) {
    if (++heater_aux.hysteresis > HEATER_HYSTERESIS) {
      heater_aux.hysteresis = HEATER_HYSTERESIS;
      heater_aux.state = HEATER_REGULATED;
    }
  } else {
    if (--heater_aux.hysteresis <= 0) {
      heater_aux.hysteresis = 0;
      heater_aux.regulation_timer = 0;			// reset timeouts
      heater_aux.state = HEATER_HEATING;
    }
  }


  if (heater_aux.state == HEATER_REGULATED) {
    //led_on()
  } else {
    if (++heater_aux.toggle > 3) {
      heater_aux.toggle = 0;
      //led_toggle();
    }
  }
}// end function

void heatbed_callback()
{
  // catch the no-op cases
  if ((heatbed.state == HEATER_OFF) || (heatbed.state == HEATER_SHUTDOWN)) { return;}

  //rpt_readout(); Replace with TinyG2 compliant data printout code

  // get current temperature from the sensor
  heatbed.temperature = thermistor_1_get_temp();

  // trap overheat condition
  if (heatbed.temperature > heatbed.overheat_temperature) {
    heatbed_OFF(HEATER_SHUTDOWN, HEATER_OVERHEATED);
    return;
  }

  thermistor_1_read();				// start reading for the next interval

  // handle bad readings from the sensor
  if (heatbed.temperature < ABSOLUTE_ZERO) {
    if (++heatbed.bad_reading_count > heatbed.bad_reading_max) {
      heatbed_OFF(HEATER_SHUTDOWN, HEATER_SENSOR_ERROR);
    //	printf_P(PSTR("Heater Sensor Error Shutdown\n"));
    }
    return;
  }
  heatbed.bad_reading_count = 0;		// reset the bad reading counter

  double duty_cycle = pid_heatbed_calculate(heatbed.setpoint, heatbed.temperature);
  set_pwm(duty_cycle,3);

  // handle HEATER exceptions
  if (heatbed.state == HEATER_HEATING) {
    heatbed.regulation_timer += HEATER_TICK_SECONDS;

    if ((heatbed.temperature < heatbed.ambient_temperature) &&
      (heatbed.regulation_timer > heatbed.ambient_timeout)) {
      heatbed_OFF(HEATER_SHUTDOWN, HEATER_AMBIENT_TIMED_OUT);
    //	printf_P(PSTR("Heater Ambient Error Shutdown\n"));
      return;
    }
    if ((heatbed.temperature < heatbed.setpoint) &&
      (heatbed.regulation_timer > heatbed.regulation_timeout)) {
      heatbed_OFF(HEATER_SHUTDOWN, HEATER_REGULATION_TIMED_OUT);
    //	printf_P(PSTR("Heater Timeout Error Shutdown\n"));
      return;
    }
  }

  // Manage regulation state and LED indicator
  // Heater.regulation_count is a hysteresis register that increments if the
  // heater is at temp, decrements if not. It pegs at max and min values.
  // The LED flashes if the heater is not in regulation and goes solid if it is.

  // check on C++ vs C differences for fabs math.h call
  if (fabs(pid_heatbed.error) <= heatbed.regulation_range) {
    if (++heatbed.hysteresis > HEATER_HYSTERESIS) {
      heatbed.hysteresis = HEATER_HYSTERESIS;
      heatbed.state = HEATER_REGULATED;
    }
  } else {
    if (--heatbed.hysteresis <= 0) {
      heatbed.hysteresis = 0;
      heatbed.regulation_timer = 0;			// reset timeouts
      heatbed.state = HEATER_HEATING;
    }
  }


  if (heatbed.state == HEATER_REGULATED) {
    //led_on()
  } else {
    if (++heatbed.toggle > 3) {
      heatbed.toggle = 0;
      //led_toggle();
    }
  }
}// end function


// Heater and Fan PWM signal channel control
void set_pwm(double pwm_duty_cycle,uint8_t chan){
  switch(chan){
    case 0:        // Heater-0 PWM Control Signal
    break;
    case 1:        // Heater-1 PWM Control Signal
    break;
    case 2:        // Heater-Aux PWM Control Signal
    break;
    case 3:        // Heatbed PWM Control Signal
    break;
    case 4:        // Fan-0 PWM Control Signal
    break;
    case 5:        // Fan-1 PWM Control Signal
    break;
  }

}


/**** Heater PID Functions ****/
/*
 * pid_init() - initialize PID with default values
 * pid_reset() - reset PID values to cold start
 * pid_calc() - derived from: http://www.embeddedheaven.com/pid-control-algorithm-c-language.htm
 */
void pid_heater0_init()
{
	memset(&pid, 0, sizeof(struct pid_struct));
	pid_heater_0.Kp = PID_Kp;
	pid_heater_0.Ki = PID_Ki;
	pid_heater_0.Kd = PID_Kd;
	pid_heater_0.output_max = PID_MAX_OUTPUT;		// saturation filter max value
	pid_heater_0.output_min = PID_MIN_OUTPUT;		// saturation filter min value
	pid_heater_0.state = PID_ON;
}

void pid_heater1_init()
{
  memset(&pid, 0, sizeof(struct pid_struct));
  pid_heater_1.Kp = PID_Kp;
  pid_heater_1.Ki = PID_Ki;
  pid_heater_1.Kd = PID_Kd;
  pid_heater_1.output_max = PID_MAX_OUTPUT;		// saturation filter max value
  pid_heater_1.output_min = PID_MIN_OUTPUT;		// saturation filter min value
  pid_heater_1.state = PID_ON;
}

void pid_heaterAux_init()
{
  memset(&pid, 0, sizeof(struct pid_struct));
  pid_heater_aux.Kp = PID_Kp;
  pid_heater_aux.Ki = PID_Ki;
  pid_heater_aux.Kd = PID_Kd;
  pid_heater_aux.output_max = PID_MAX_OUTPUT;		// saturation filter max value
  pid_heater_aux.output_min = PID_MIN_OUTPUT;		// saturation filter min value
  pid_heater_aux.state = PID_ON;
}

void pid_heatbed_init()
{
  memset(&pid, 0, sizeof(struct pid_struct));
  pid_heatbed.Kp = PID_Kp;
  pid_heatbed.Ki = PID_Ki;
  pid_heatbed.Kd = PID_Kd;
  pid_heatbed.output_max = PID_MAX_OUTPUT;		// saturation filter max value
  pid_heatbed.output_min = PID_MIN_OUTPUT;		// saturation filter min value
  pid_heatbed.state = PID_ON;
}


void pid_heater0_reset()
{
	pid_heater_0.output = 0;
	pid_heater_0.integral = PID_INITIAL_INTEGRAL;
	pid_heater_0.prev_error = 0;
}

void pid_heater1_reset()
{
  pid_heater_1.output = 0;
  pid_heater_1.integral = PID_INITIAL_INTEGRAL;
  pid_heater_1.prev_error = 0;
}

void pid_heaterAux_reset()
{
  pid_heater_aux.output = 0;
  pid_heater_aux.integral = PID_INITIAL_INTEGRAL;
  pid_heater_aux.prev_error = 0;
}

void pid_heatbed_reset()
{
  pid_heatbed.output = 0;
  pid_heatbed.integral = PID_INITIAL_INTEGRAL;
  pid_heatbed.prev_error = 0;
}

double pid_heater0_calculate(double setpoint,double temperature)
{
	if (pid_heater_0.state == PID_OFF) { return (pid_heater_0.output_min);}

	pid_heater_0.error = setpoint - temperature;		// current error term

	// perform integration only if error is GT epsilon, and with anti-windup
	if ((fabs(pid_heater_0.error) > PID_EPSILON) && (pid_heater_0.output < pid_heater_0.output_max)) {
		pid_heater_0.integral += (pid_heater_0.error * PID_DT);
	}
	// compute derivative and output
	pid_heater_0.derivative = (pid_heater_0.error - pid_heater_0.prev_error) / PID_DT;
	pid_heater_0.output = pid_heater_0.Kp * pid_heater_0.error + pid_heater_0.Ki * pid_heater_0.integral + pid_heater_0.Kd * pid_heater_0.derivative;

	// fix min amd max outputs (saturation filter)
	if(pid_heater_0.output > pid_heater_0.output_max) { pid_heater_0.output = pid_heater_0.output_max; } else
	if(pid_heater_0.output < pid_heater_0.output_min) { pid_heater_0.output = pid_heater_0.output_min; }
	pid_heater_0.prev_error = pid_heater_0.error;

	return pid_heater_0.output;
}// end fucntion


double pid_heater_calculate(double setpoint,double temperature)
{
  if (pid_heater_1.state == PID_OFF) { return (pid_heater_1.output_min);}

  pid_heater_1.error = setpoint - temperature;		// current error term

  // perform integration only if error is GT epsilon, and with anti-windup
  if ((fabs(pid_heater_1.error) > PID_EPSILON) && (pid_heater_1.output < pid_heater_1.output_max)) {
    pid_heater_1.integral += (pid_heater_1.error * PID_DT);
  }
  // compute derivative and output
  pid_heater_1.derivative = (pid_heater_1.error - pid_heater_1.prev_error) / PID_DT;
  pid_heater_1.output = pid_heater_1.Kp * pid_heater_1.error + pid_heater_1.Ki * pid_heater_1.integral + pid_heater_1.Kd * pid_heater_1.derivative;

  // fix min amd max outputs (saturation filter)
  if(pid_heater_1.output > pid_heater_1.output_max) { pid_heater_1.output = pid_heater_1.output_max; } else
  if(pid_heater_1.output < pid_heater_1.output_min) { pid_heater_1.output = pid_heater_1.output_min; }
  pid_heater_1.prev_error = pid_heater_1.error;

  return pid_heater_1.output;
}// end fucntion

double pid_heater_calculate(double setpoint,double temperature)
{
  if (pid_heater_aux.state == PID_OFF) { return (pid_heater_aux.output_min);}

  pid_heater_aux.error = setpoint - temperature;		// current error term

  // perform integration only if error is GT epsilon, and with anti-windup
  if ((fabs(pid_heater_aux.error) > PID_EPSILON) && (pid_heater_aux.output < pid_heater_aux.output_max)) {
    pid_heater_aux.integral += (pid_heater_aux.error * PID_DT);
  }
  // compute derivative and output
  pid_heater_aux.derivative = (pid_heater_aux.error - pid_heater_aux.prev_error) / PID_DT;
  pid_heater_aux.output = pid_heater_aux.Kp * pid_heater_aux.error + pid_heater_aux.Ki * pid_heater_aux.integral + pid_heater_aux.Kd * pid_heater_aux.derivative;

  // fix min amd max outputs (saturation filter)
  if(pid_heater_aux.output > pid_heater_aux.output_max) { pid_heater_aux.output = pid_heater_aux.output_max; } else
  if(pid_heater_aux.output < pid_heater_aux.output_min) { pid_heater_aux.output = pid_heater_aux.output_min; }
  pid_heater_aux.prev_error = pid_heater_aux.error;

  return pid_heater_aux.output;
}// end fucntion


double pid_heater_calculate(double setpoint,double temperature)
{
  if (pid_heatbed.state == PID_OFF) { return (pid_heatbed.output_min);}

  pid_heatbed.error = setpoint - temperature;		// current error term

  // perform integration only if error is GT epsilon, and with anti-windup
  if ((fabs(pid_heatbed.error) > PID_EPSILON) && (pid_heatbed.output < pid_heatbed.output_max)) {
    pid_heatbed.integral += (pid_heatbed.error * PID_DT);
  }
  // compute derivative and output
  pid_heatbed.derivative = (pid_heatbed.error - pid_heatbed.prev_error) / PID_DT;
  pid_heatbed.output = pid_heatbed.Kp * pid_heatbed.error + pid_heatbed.Ki * pid_heatbed.integral + pid_heatbed.Kd * pid_heatbed.derivative;

  // fix min amd max outputs (saturation filter)
  if(pid_heatbed.output > pid_heatbed.output_max) { pid_heatbed.output = pid_heatbed.output_max; } else
  if(pid_heatbed.output < pid_heatbed.output_min) { pid_heatbed.output = pid_heatbed.output_min; }
  pid_heatbed.prev_error = pid_heatbed.error;

  return pid_heatbed.output;
}// end fucntion
