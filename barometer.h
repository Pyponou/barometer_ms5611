/*
 * barometer.h
 *
 *  Created on: 5 oct. 2018
 *      Author: alex
 */

#ifndef BAROMETER_H_
#define BAROMETER_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Init the Barometer with default parameters
 */
extern void Barometer_init();

/**
 * @brief Return the temperature with a 2 digits precision in celcius
 * Example : 2000 -> 20,00°C
 *
 * @param calculate true if you want to update the value
 * 		  It will update the threes values
 * @pre call Barometer_init
 *
 * @return the temperature
 */
extern int32_t Barometer_getTemp(bool calculate);

/**
 * @brief Return the pressure in mbar with a 2 digits precision
 * Example : 100000 -> 1000,00 mbar
 *
 * @param calculate true if you want to update the value
 * 		  It will update the threes values
 * @pre call Barometer_init
 *
 * @return the pressure
 */
extern int32_t Barometer_getPressure(bool calculate);

/**
 * @brief Return the altitude in meters
 *
 * @param calculate true if you want to update the value
 * 		  It will update the threes values
 * @pre call Barometer_init
 *
 * @return the altitude
 */
extern float Barometer_getAltitude(bool calculate);

/**
 * @brief calculate/update the altitude/pressure/temperature
 * 		  using the barometer
 */
extern void Barometer_calculate();

#endif /* BAROMETER_H_ */
