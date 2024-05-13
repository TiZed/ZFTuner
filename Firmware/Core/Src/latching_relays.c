/**
  ******************************************************************************
  * @file           : latching_relays.c
  * @brief          : Latching relays control code
  ******************************************************************************
*/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "stm32g4xx_hal.h"

#include "latching_relays.h"

/**
 * @brief Set a relay 
 * 
 * @param relay: Relay to set
 */
void set_relay(relay_t * relay) {
    // Enable power for relay
    HAL_GPIO_WritePin(relay->Relay_Power_Port, relay->power_pin, GPIO_PIN_SET) ;
    HAL_Delay(RELAY_DELAY) ;

    // Enable set pin
    HAL_GPIO_WritePin(relay->Relay_Port, relay->set_pin, GPIO_PIN_SET) ;
    HAL_Delay(RELAY_DELAY) ;

    // Disable set pin and power
    HAL_GPIO_WritePin(relay->Relay_Port, relay->set_pin, GPIO_PIN_RESET) ;
    HAL_GPIO_WritePin(relay->Relay_Power_Port, relay->power_pin, GPIO_PIN_RESET) ;

    relay->current_state = true ;
}

/**
 * @brief Reset a relay
 * 
 * @param relay: Relay to reset
 */
void reset_relay(relay_t * relay) {
    // Enable power for relay
    HAL_GPIO_WritePin(relay->Relay_Power_Port, relay->power_pin, GPIO_PIN_SET) ;
    HAL_Delay(RELAY_DELAY) ;

    // Enable reset pin
    HAL_GPIO_WritePin(relay->Relay_Port, relay->reset_pin, GPIO_PIN_SET) ;
    HAL_Delay(RELAY_DELAY) ;

    // Disable reset pin and power
    HAL_GPIO_WritePin(relay->Relay_Port, relay->reset_pin, GPIO_PIN_RESET) ;
    HAL_GPIO_WritePin(relay->Relay_Power_Port, relay->power_pin, GPIO_PIN_RESET) ;

    relay->current_state = false ;
}