/**
  ******************************************************************************
  * @file           : latching_relays.h
  * @brief          : Header for latching_relays.c file.
  ******************************************************************************
*/

#ifndef __LATCHING_RELAYS__H__
#define __LATCHING_RELAYS__H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>

typedef struct {
    GPIO_TypeDef * Relay_Port ;
    GPIO_TypeDef * Relay_Power_Port ;

    uint16_t set_pin ;
    uint16_t reset_pin ;
    uint16_t power_pin ;
    
    bool current_state ;
} relay_t ;

#define RELAY_DELAY 110     // ms delay for switching relay

void set_relay(relay_t * relay) ;
void reset_relay(relay_t * relay) ;

#ifdef __cplusplus
}
#endif

#endif /* __LATCHING_RELAYS__H__ */
