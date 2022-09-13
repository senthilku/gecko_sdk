/***************************************************************************//**
 * @file
 * @brief Simple LED Driver
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "sl_simple_led.h"

#include "em_cmu.h"

sl_status_t sl_simple_led_init(void *context)
{
  sl_simple_led_context_t *led = context;

#ifndef CCP_SI917_BRINGUP        
  #if !defined(_SILICON_LABS_32B_SERIES_2)
  CMU_ClockEnable(cmuClock_HFPER, true);
  #endif //!defined(_SILICON_LABS_32B_SERIES_2)

  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(led->port,
                  led->pin,
                  gpioModePushPull,
                  !led->polarity);
#endif /* CCP_SI917_BRINGUP */
  return SL_STATUS_OK;
}

void sl_simple_led_turn_on(void *context)
{
#ifndef CCP_SI917_BRINGUP      
  sl_simple_led_context_t *led = context;
  if (led->polarity == SL_SIMPLE_LED_POLARITY_ACTIVE_LOW) {
    GPIO_PinOutClear(led->port, led->pin);
  } else {
    GPIO_PinOutSet(led->port, led->pin);
  }
#endif /* CCP_SI917_BRINGUP */  
}

void sl_simple_led_turn_off(void *context)
{
#ifndef CCP_SI917_BRINGUP      
  sl_simple_led_context_t *led = context;
  if (led->polarity == SL_SIMPLE_LED_POLARITY_ACTIVE_LOW) {
    GPIO_PinOutSet(led->port, led->pin);
  } else {
    GPIO_PinOutClear(led->port, led->pin);
  }
#endif /* CCP_SI917_BRINGUP */  
}

void sl_simple_led_toggle(void *context)
{
#ifndef CCP_SI917_BRINGUP      
  sl_simple_led_context_t *led = context;
  GPIO_PinOutToggle(led->port, led->pin);
#endif /* CCP_SI917_BRINGUP */  
}

sl_led_state_t sl_simple_led_get_state(void *context)
{
#ifndef CCP_SI917_BRINGUP      
  sl_simple_led_context_t *led = context;
  sl_led_state_t value = (sl_led_state_t)GPIO_PinOutGet(led->port, led->pin);

  if (led->polarity == SL_SIMPLE_LED_POLARITY_ACTIVE_LOW) {
    return !value;
  } else {
    return value;
  }
#endif /* CCP_SI917_BRINGUP */  
}
