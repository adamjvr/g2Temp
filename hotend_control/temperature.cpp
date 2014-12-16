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
#include "sensors.h"


// Heater MOSFET Configuration
// 0 = OFF! / 1 = ON!
#define HEATER_0_ENABLE 0
#define HEATER_1_ENABLE 0
#define HEATER_AUX_ENABLE 0
#define HEATBED_ENABLE 0


// Runs PID Controller for Heater-0 on Tigershark 3D PROTOTYPE 1
void controller_heater_0(){
  heater_0_init();
  thermocouple_0_init();
}
// Runs PID Controller for Heater-1 on Tigershark 3D PROTOTYPE 1
void controller_heater_1(){
  heater_1_init();
  thermocouple_1_init();
}
// Runs PID Controller for Heater-Aux on Tigershark 3D PROTOTYPE 1
void controller_heater_aux(){
  heater_aux_init();
  thermistor_0_init();
}
// Runs PID Controller for Heatbed on Tigershark 3D PROTOTYPE 1
void controller_heatbed(){
  heatbed_init();
  thermistor_0_init();
}
