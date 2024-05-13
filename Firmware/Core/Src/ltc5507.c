/**
  ******************************************************************************
  * @file           : ltc5507.c
  * @brief          : LTC5507 RF Power Detector related code
  ******************************************************************************
*/

#include <stdint.h>
#include <math.h>

#include <ltc5507.h>

/**
 * @brief Translate LTC5507 mV Vout to dBm RF power
 * 
 * @param vout_mv: Vout voltage in mV
 * @return float: RF power in dBm
 *          Value of -26.1 mean the result is <-26.0dBm min. measurable value.
 *          Value of 14.1 mean the result is >14.0dBm max. measurable value.
 * 
 * Polynomial regression equation of LTC5507 power to voltage response 
 * f(x) = 1.128E-08 x³ − 5.754E-05 x² + 1.004E-01 x − 5.058E+01
 */
// 
float ltc5507_vout_mv2dbm(uint16_t vout_mv) {
    if(vout_mv <= MIN_VOUT) return -26.1 ;
    if(vout_mv >= MAX_VOUT) return 14.1 ;

    float vout = (float) vout_mv ;
    float rf_power = 0.0 ;

    rf_power += 1.128e-8 * powf(vout, 3) ;
    rf_power -= 5.754e-5 * powf(vout, 2) ;
    rf_power += 0.1004 * vout ;
    rf_power -= 50.58 ;

    return rf_power ;
}