#include "application_config.h"
#include "analysis/ruuvi_library_peak2peak.h"
#include "analysis/ruuvi_library_rms.h"
#include "analysis/ruuvi_library_variance.h"
#include "ruuvi_boards.h"
#include "ruuvi_driver_error.h"
#include "ruuvi_driver_sensor.h"
#include "ruuvi_interface_acceleration.h"
#include "ruuvi_interface_communication.h"
#include "ruuvi_interface_gpio.h"
#include "ruuvi_interface_gpio_interrupt.h"
#include "ruuvi_interface_lis2dh12.h"
#include "ruuvi_interface_log.h"
#include "ruuvi_interface_rtc.h"
#include "ruuvi_interface_scheduler.h"
#include "ruuvi_interface_timer.h"
#include "ruuvi_interface_yield.h"

#include "task_acceleration.h"
#include "task_gatt.h"
#include "task_led.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

RUUVI_PLATFORM_TIMER_ID_DEF(acceleration_timer);
static ruuvi_driver_sensor_t acceleration_sensor = {0};
static uint8_t m_nbr_movements;
static float m_p2p[3];
static float m_rms[3];
static float m_var[3];


//handler for scheduled accelerometer event
static void task_acceleration_scheduler_task(void *p_event_data, uint16_t event_size)
{
  // No action necessary
}

// Timer callback, schedule accelerometer event here.
static void task_acceleration_timer_cb(void* p_context)
{
  ruuvi_platform_scheduler_event_put(NULL, 0, task_acceleration_scheduler_task);
}

static inline int8_t f2i(float value)
{
  if(value > 125) return 125;
  if(value < -125) return -125;
  return (int8_t) lroundf(value);
}

static void task_acceleration_fifo_full_task(void *p_event_data, uint16_t event_size)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_interface_acceleration_data_t data[32];
  size_t data_len = sizeof(data)/sizeof(ruuvi_interface_acceleration_data_t);
  err_code |= ruuvi_interface_lis2dh12_fifo_read(&data_len, data);
 
  float x[32];
  float y[32];
  float z[32];
  for(int ii = 0; ii < data_len; ii++)
  {
    x[ii] = data[ii].x_g;
    y[ii] = data[ii].y_g;
    z[ii] = data[ii].z_g;
  }
  m_p2p[0] = ruuvi_library_peak2peak(x, data_len);
  m_p2p[1] = ruuvi_library_peak2peak(y, data_len);
  m_p2p[2] = ruuvi_library_peak2peak(z, data_len);
  m_rms[0] = ruuvi_library_rms(x, data_len);
  m_rms[1] = ruuvi_library_rms(y, data_len);
  m_rms[2] = ruuvi_library_rms(z, data_len);
  m_var[0] = ruuvi_library_variance(x, data_len);
  m_var[1] = ruuvi_library_variance(y, data_len);
  m_var[2] = ruuvi_library_variance(z, data_len);

  char message[128] = {0};
  snprintf(message, sizeof(message), "P2P: X: %0.3f, Y: %0.3f, Z: %0.3f\r\n", m_p2p[0], m_p2p[1], m_p2p[2]);
  ruuvi_platform_log(RUUVI_INTERFACE_LOG_INFO, message);
  snprintf(message, sizeof(message), "RMS: X: %0.3f, Y: %0.3f, Z: %0.3f\r\n", m_rms[0], m_rms[1], m_rms[2]);
  ruuvi_platform_log(RUUVI_INTERFACE_LOG_INFO, message);
  snprintf(message, sizeof(message), "DEV: X: %0.3f, Y: %0.3f, Z: %0.3f\r\n\r\n", sqrtf(m_var[0]), sqrtf(m_var[1]), sqrtf(m_var[2]));
  ruuvi_platform_log(RUUVI_INTERFACE_LOG_INFO, message);
  
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
}

static void on_fifo (ruuvi_interface_gpio_evt_t event)
{
  ruuvi_platform_scheduler_event_put(NULL, 0, task_acceleration_fifo_full_task);
}


static void on_movement (ruuvi_interface_gpio_evt_t event)
{
  ruuvi_platform_log(RUUVI_INTERFACE_LOG_DEBUG, "Activity\r\n");
  m_nbr_movements++;
}

ruuvi_driver_status_t task_acceleration_configure(ruuvi_driver_sensor_configuration_t* config)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_platform_log(RUUVI_INTERFACE_LOG_INFO, "\r\nAttempting to configure accelerometer with:\r\n");
  ruuvi_interface_log_sensor_configuration(RUUVI_INTERFACE_LOG_INFO, config, "g");
  err_code |= acceleration_sensor.configuration_set(&acceleration_sensor, config);
  RUUVI_DRIVER_ERROR_CHECK(err_code, ~RUUVI_DRIVER_ERROR_FATAL);
  ruuvi_platform_log(RUUVI_INTERFACE_LOG_INFO, "Actual configuration:\r\n");
  ruuvi_interface_log_sensor_configuration(RUUVI_INTERFACE_LOG_INFO, config, "g");
  return err_code;
}

ruuvi_driver_status_t task_acceleration_init(void)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_driver_bus_t bus = RUUVI_DRIVER_BUS_NONE;
  ruuvi_driver_sensor_configuration_t config;
  config.samplerate    = APPLICATION_ACCELEROMETER_SAMPLERATE;
  config.resolution    = APPLICATION_ACCELEROMETER_RESOLUTION;
  config.scale         = APPLICATION_ACCELEROMETER_SCALE;
  config.dsp_function  = APPLICATION_ACCELEROMETER_DSPFUNC;
  config.dsp_parameter = APPLICATION_ACCELEROMETER_DSPPARAM;
  config.mode          = APPLICATION_ACCELEROMETER_MODE;
  uint8_t handle = 0;
  m_nbr_movements = 0;

  // Initialize timer for accelerometer task. Note: the timer is not started.
  err_code |= ruuvi_platform_timer_create(&acceleration_timer, RUUVI_INTERFACE_TIMER_MODE_REPEATED, task_acceleration_timer_cb);

  #if RUUVI_BOARD_ACCELEROMETER_LIS2DH12_PRESENT
    err_code = RUUVI_DRIVER_SUCCESS;
    // Only SPI supported for now
    bus = RUUVI_DRIVER_BUS_SPI;
    handle = RUUVI_BOARD_SPI_SS_ACCELEROMETER_PIN;
    err_code |= ruuvi_interface_lis2dh12_init(&acceleration_sensor, bus, handle);
    RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_ERROR_NOT_FOUND);

    if(RUUVI_DRIVER_SUCCESS == err_code)
    {

      err_code |= task_acceleration_configure(&config);
      // float ths = APPLICATION_ACCELEROMETER_ACTIVITY_THRESHOLD;
      // err_code |= ruuvi_interface_lis2dh12_activity_interrupt_use(true, &ths);
      err_code |= ruuvi_interface_lis2dh12_fifo_use(true);
      err_code |= ruuvi_interface_lis2dh12_fifo_interrupt_use(true);

      // Let pins settle
      ruuvi_platform_delay_ms(10);
      // Setup FIFO and activity interrupts
      err_code |= ruuvi_platform_gpio_interrupt_enable(RUUVI_BOARD_INT_ACC1_PIN, RUUVI_INTERFACE_GPIO_SLOPE_LOTOHI, RUUVI_INTERFACE_GPIO_MODE_INPUT_NOPULL, on_fifo);

      // err_code |= ruuvi_platform_gpio_interrupt_enable(RUUVI_BOARD_INT_ACC2_PIN, RUUVI_INTERFACE_GPIO_SLOPE_LOTOHI, RUUVI_INTERFACE_GPIO_MODE_INPUT_NOPULL, on_movement);
      char msg[APPLICATION_LOG_BUFFER_SIZE] = { 0 };
      // snprintf(msg, sizeof(msg), "Configured interrupt threshold at %.3f mg\r\n", ths);
      // ruuvi_platform_log(RUUVI_INTERFACE_LOG_INFO, msg);

      return err_code;
    }
  #endif

  // Return error if usable acceleration sensor was not found.
  return RUUVI_DRIVER_ERROR_NOT_FOUND;
}

ruuvi_driver_status_t task_acceleration_data_log(const ruuvi_interface_log_severity_t level)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_interface_acceleration_data_t data;
  if(NULL == acceleration_sensor.data_get) { return RUUVI_DRIVER_ERROR_INVALID_STATE; }

  err_code |= acceleration_sensor.data_get(&data);
  char message[128] = {0};
  snprintf(message, sizeof(message), "Time: %lu\r\n", (uint32_t)(data.timestamp_ms&0xFFFFFFFF));
  ruuvi_platform_log(level, message);
  snprintf(message, sizeof(message), "X: %.3f\r\n", data.x_g);
  ruuvi_platform_log(level, message);
  snprintf(message, sizeof(message), "Y: %.3f\r\n" ,data.y_g);
  ruuvi_platform_log(level, message);
  snprintf(message, sizeof(message), "Z: %.3f\r\n", data.z_g);
  ruuvi_platform_log(level, message);
  return err_code;
}

ruuvi_driver_status_t task_acceleration_data_get(ruuvi_interface_acceleration_data_t* const data)
{
  data->timestamp_ms = RUUVI_DRIVER_UINT64_INVALID;
  data->x_g = RUUVI_DRIVER_FLOAT_INVALID;
  data->y_g = RUUVI_DRIVER_FLOAT_INVALID;
  data->z_g = RUUVI_DRIVER_FLOAT_INVALID;
  if(NULL == data) { return RUUVI_DRIVER_ERROR_NULL; }
  if(NULL == acceleration_sensor.data_get) { return RUUVI_DRIVER_ERROR_INVALID_STATE; }
  
  return acceleration_sensor.data_get(data);
}

ruuvi_driver_status_t task_acceleration_on_button(void)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  err_code |= task_acceleration_data_log(RUUVI_INTERFACE_LOG_INFO);
  return err_code;
}

ruuvi_driver_status_t task_acceleration_movement_count_get(uint8_t * const count)
{
  *count = m_nbr_movements;
  return RUUVI_DRIVER_SUCCESS;
}

ruuvi_driver_status_t task_acceleration_fifo_use(const bool enable)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  if(true == enable)
  {
    err_code |= ruuvi_interface_lis2dh12_fifo_use(true);
    err_code |= ruuvi_interface_lis2dh12_fifo_interrupt_use(true);
  }
  if(false == enable)
  {
    err_code |= ruuvi_interface_lis2dh12_fifo_use(false);
    err_code |= ruuvi_interface_lis2dh12_fifo_interrupt_use(false);
  }
  return err_code;
}

ruuvi_driver_status_t task_acceleration_p2p_get(ruuvi_interface_acceleration_data_t* const data)
{
  data->x_g = m_p2p[TASK_ACCELERATION_X_INDEX];
  data->y_g = m_p2p[TASK_ACCELERATION_Y_INDEX];
  data->z_g = m_p2p[TASK_ACCELERATION_Z_INDEX];
  return RUUVI_DRIVER_SUCCESS;
}

ruuvi_driver_status_t task_acceleration_rms_get(ruuvi_interface_acceleration_data_t* const data)
{
  data->x_g = m_rms[TASK_ACCELERATION_X_INDEX];
  data->y_g = m_rms[TASK_ACCELERATION_Y_INDEX];
  data->z_g = m_rms[TASK_ACCELERATION_Z_INDEX];
  return RUUVI_DRIVER_SUCCESS;
}

ruuvi_driver_status_t task_acceleration_dev_get(ruuvi_interface_acceleration_data_t* const data)
{
  data->x_g = sqrtf(m_var[TASK_ACCELERATION_X_INDEX]);
  data->y_g = sqrtf(m_var[TASK_ACCELERATION_Y_INDEX]);
  data->z_g = sqrtf(m_var[TASK_ACCELERATION_Z_INDEX]);
  return RUUVI_DRIVER_SUCCESS;
}