/**
  ******************************************************************************
  * @file           : ltc5507.h
  * @brief          : Header for latching_relays.c file.
  ******************************************************************************
*/

#ifndef __LTC5507__H__
#define __LTC5507__H__

#ifdef __cplusplus
 extern "C" {
#endif

#define MAX_RF_POWER    14.0    // dBm
#define MIN_RF_POWER    -34.0   // dBm

#define MIN_VOUT        280     // mV
#define MAX_VOUT       2400     // mV

float ltc5507_vout_mv2dbm(uint16_t vout_mv) ;

#ifdef __cplusplus
}
#endif

#endif /* __LTC5507__H__ */