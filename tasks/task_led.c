#include "task_led.h"
#include "task_environmental.h"
#include "ruuvi_driver_error.h"
#include "ruuvi_interface_gpio.h"
#include <stddef.h>

ruuvi_driver_status_t task_led_init(void)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;

  if(!ruuvi_interface_gpio_is_init())
  {
    err_code |= ruuvi_interface_gpio_init();
  }

  ruuvi_interface_gpio_id_t leds[] = RUUVI_BOARD_LEDS_LIST;

  for(size_t ii = 0; ii < RUUVI_BOARD_LEDS_NUMBER; ii++)
  {
    ruuvi_interface_gpio_configure(leds[ii], RUUVI_INTERFACE_GPIO_MODE_OUTPUT_HIGHDRIVE);
    ruuvi_interface_gpio_write(leds[ii], !RUUVI_BOARD_LEDS_ACTIVE_STATE);
  }

  return err_code;
}

ruuvi_driver_status_t task_led_write(const uint16_t led, const task_led_state_t state)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_interface_gpio_id_t pin = {.pin = led };
  err_code |= ruuvi_interface_gpio_write(pin, state);
  return err_code;
}

ruuvi_driver_status_t task_led_cycle(void)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  static uint8_t phase = 0;
  uint8_t leds[] = RUUVI_BOARD_LEDS_LIST;
  ruuvi_interface_gpio_id_t pin = {.pin = leds[phase++]};
  err_code |= ruuvi_interface_gpio_toggle(pin);

  if(RUUVI_BOARD_LEDS_NUMBER <= phase) { phase = 0; }

  return err_code;
}