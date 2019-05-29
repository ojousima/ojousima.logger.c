#include "application_config.h"
#include "analysis/ruuvi_library_peak2peak.h"
#include "analysis/ruuvi_library_rms.h"
#include "analysis/ruuvi_library_variance.h"
#include "ruuvi_boards.h"
#include "ruuvi_driver_error.h"
#include "ruuvi_driver_sensor.h"
#include "ruuvi_interface_acceleration.h"
#include "ruuvi_interface_communication.h"
#include "ruuvi_interface_communication_ble4_advertising.h"
#include "ruuvi_interface_gpio.h"
#include "ruuvi_interface_gpio_interrupt.h"
#include "ruuvi_interface_lis2dh12.h"
#include "ruuvi_interface_log.h"
#include "ruuvi_interface_rtc.h"
#include "ruuvi_interface_scheduler.h"
#include "ruuvi_interface_timer.h"
#include "ruuvi_interface_yield.h"

#include "task_acceleration.h"
#include "task_adc.h"
#include "task_advertisement.h"
#include "task_environmental.h"
#include "task_gatt.h"
#include "task_led.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

static ruuvi_interface_timer_id_t m_measurement_timer;
static ruuvi_driver_sensor_t acceleration_sensor = {0};
static uint8_t m_nbr_movements;
static task_acceleration_data_t bdata[APPLICATION_ACCELEROMETER_DATASETS * 32];
static size_t series_counter = 0;
static size_t series_length = 0;
static float m_p2p[3];
static float m_rms[3];
static float m_var[3];
static uint8_t m_scale = APPLICATION_ACCELEROMETER_SCALE; //<! current scale of accelerometer
static uint8_t m_series_scale = 0;                  //<! scale of last accelerometer run
static float m_activity_ths = 0.1f; // TODO: define
static uint64_t m_last_run = 0; 

static int8_t f2i(float value)
{
  value *= (126.0/m_scale);
  if(value > 125) return 125;
  if(value < -125) return -125;
  return (int8_t) lroundf(value);
}

static float i2f(const int8_t value)
{
  return value / (126.0/m_scale);
}


ruuvi_driver_status_t task_acceleration_configure(ruuvi_driver_sensor_configuration_t*
    config)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO,
                      "\r\nAttempting to configure accelerometer with:\r\n");
  ruuvi_interface_log_sensor_configuration(RUUVI_INTERFACE_LOG_INFO, config, "g");
  err_code |= acceleration_sensor.configuration_set(&acceleration_sensor, config);
  ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, "Actual configuration:\r\n");
  ruuvi_interface_log_sensor_configuration(RUUVI_INTERFACE_LOG_INFO, config, "g");
  m_scale = config->scale;
  RUUVI_DRIVER_ERROR_CHECK(err_code, ~RUUVI_DRIVER_ERROR_FATAL);
  return err_code;
}

void task_acceleration_enter_measuring(void* p_event_data, uint16_t event_size)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_driver_sensor_configuration_t config;
  config.samplerate    = RUUVI_DRIVER_SENSOR_CFG_MAX;
  config.resolution    = APPLICATION_ACCELEROMETER_RESOLUTION;
  config.scale         = m_scale;
  config.dsp_function  = APPLICATION_ACCELEROMETER_DSPFUNC;
  config.dsp_parameter = APPLICATION_ACCELEROMETER_DSPPARAM;
  config.mode          = RUUVI_DRIVER_SENSOR_CFG_CONTINUOUS;
  err_code |= task_acceleration_configure(&config);
  series_length = 0;
  float ths = APPLICATION_ACCELEROMETER_ACTIVITY_THRESHOLD;
  err_code |= ruuvi_interface_lis2dh12_activity_interrupt_use(false, &ths);
  err_code |= ruuvi_interface_lis2dh12_fifo_use(true);
  err_code |= ruuvi_interface_lis2dh12_fifo_interrupt_use(true);
  task_led_write(RUUVI_BOARD_LED_GREEN, TASK_LED_ON);
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
}

static void task_acceleration_enter_standby(void* p_event_data, uint16_t event_size)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_driver_sensor_configuration_t config;
  config.samplerate    = 10; // TODO: Define
  config.resolution    = APPLICATION_ACCELEROMETER_RESOLUTION;
  config.scale         = m_scale;
  config.dsp_function  = APPLICATION_ACCELEROMETER_DSPFUNC;
  config.dsp_parameter = APPLICATION_ACCELEROMETER_DSPPARAM;
  config.mode          = RUUVI_DRIVER_SENSOR_CFG_CONTINUOUS;
  err_code |= task_acceleration_configure(&config);
  if(m_activity_ths > m_scale * 0.5)
  {
    m_activity_ths = m_scale * 0.5;
  }
  err_code |= ruuvi_interface_lis2dh12_activity_interrupt_use(true, &m_activity_ths);
  err_code |= ruuvi_interface_lis2dh12_fifo_use(false);
  err_code |= ruuvi_interface_lis2dh12_fifo_interrupt_use(false);
  task_led_write(RUUVI_BOARD_LED_GREEN, TASK_LED_OFF);
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
}

/**
 *  Calculates the threshold to trigger a new sampling sequence. 
 *  Scales down accelerometer threshold if needed
 */
static void task_acceleration_calculate_activity_threshold()
{
  
  float rms_x_accu = 0;
  float rms_y_accu = 0;
  float rms_z_accu = 0;
  float peak = 0;
  for(size_t iii = 0; iii < series_length; iii++)
  {
    const float x = i2f(bdata[iii].accx);
    const float y = i2f(bdata[iii].accy);
    const float z = i2f(bdata[iii].accz);
    peak = (x > peak) ? x : peak;
    peak = (y > peak) ? y : peak;
    peak = (z > peak) ? z : peak;
    rms_x_accu += x*x;
    rms_y_accu += y*y;
    rms_z_accu += z*z;
  }

  rms_x_accu = sqrtf(rms_x_accu / series_length);
  rms_y_accu = sqrtf(rms_y_accu / series_length);
  rms_z_accu = sqrtf(rms_z_accu / series_length);
  float rms = rms_x_accu > rms_y_accu ? rms_x_accu : rms_y_accu;
  rms = rms_z_accu > rms_y_accu ? rms_z_accu : rms_y_accu;

  m_activity_ths = 2 * peak + 1 * rms;
  char message[128] = {0};
  snprintf(message, sizeof(message), "Setting threshold to: %0.3f g\r\n", m_activity_ths);
  ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, message);

  // scale down if needed
  if(peak < m_scale / 4)
  {
    if (m_scale > 2)
    { 
      ruuvi_interface_log(RUUVI_INTERFACE_LOG_DEBUG, "Scaling down\r\n");
      m_scale /= 2; 
    }
  }
}

static void task_acceleration_fifo_full_task(void *p_event_data, uint16_t event_size)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_interface_acceleration_data_t data[32];
  size_t data_len = (sizeof(data) / sizeof(ruuvi_interface_acceleration_data_t));
  err_code |= ruuvi_interface_lis2dh12_fifo_read(&data_len, data);
  ruuvi_interface_log(RUUVI_INTERFACE_LOG_DEBUG, "FIFO\r\n");
  bool clipped = false;
  if(!data_len)
  {
    return;
  }

  for(size_t ii = 0; ii < data_len; ii++)
  {
    // Decimate 1/2 samples
    if(ii % 2)
    {
      bdata[series_length].accx = f2i(data[ii/2].x_g);
      bdata[series_length].accy = f2i(data[ii/2].y_g);
      bdata[series_length].accz = f2i(data[ii/2].z_g);

      // Check for clipped values. Sample is considered clipped if the value is over 80 % of maximum value, i.e. over
      // 100 or under -100.
      if(bdata[series_length].accx > 100 || bdata[series_length].accx < -100 ||
         bdata[series_length].accy > 100 || bdata[series_length].accy < -100 ||
         bdata[series_length].accz > 100 || bdata[series_length].accz < -100)
         {
           clipped = true;
         }
      series_length ++;
    }
  }

  /* Broadcast mode
  if(series_length >= (APPLICATION_ACCELEROMETER_DATASETS-1) * 32)
  {
    float x[APPLICATION_ACCELEROMETER_DATASETS * 32];
    float y[APPLICATION_ACCELEROMETER_DATASETS * 32];
    float z[APPLICATION_ACCELEROMETER_DATASETS * 32];
    data_len = series_length;
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
    ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, message);
    snprintf(message, sizeof(message), "RMS: X: %0.3f, Y: %0.3f, Z: %0.3f\r\n", m_rms[0], m_rms[1], m_rms[2]);
    ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, message);
    snprintf(message, sizeof(message), "DEV: X: %0.3f, Y: %0.3f, Z: %0.3f\r\n\r\n", sqrtf(m_var[0]), sqrtf(m_var[1]), sqrtf(m_var[2]));
    ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, message);

    series_counter = 0;
    series_length = 0;
    // signal that data should be updated.
    err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_acceleration_enter_standby);
    err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_advertisement_scheduler_task);

    RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
  }
  */

  /* Stream mode */

  // If clipped sample was detected, rerun with larger scale
  if(clipped)
  {
    ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, "Clip detected\r\n");
    if(m_scale < 16)
    { 
      m_scale *= 2; 
      err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_acceleration_enter_standby);
      err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_acceleration_enter_measuring);
    }
  }

  /* If Full data is collected */
  if(series_length >= (APPLICATION_ACCELEROMETER_DATASETS-1) * 32)
  {
    // Signal that data should be updated. Enter standby, sample environmental sensor and ADC.
    //
    err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_acceleration_enter_standby);
    err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_environmental_scheduler_task);
    err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_adc_scheduler_task);
    err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_gatt_scheduler_task);
    series_counter++;
    m_series_scale = m_scale;
    m_last_run = ruuvi_interface_rtc_millis();
    // Calculate threshold for activity trigger as well as downscaling.
    task_acceleration_calculate_activity_threshold();
  }
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
}

static void on_timer(void* p_context)
{
  // If last measurement has been over a day ago, rerun
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;

  if((ruuvi_interface_rtc_millis() - m_last_run) > 24*60*60*1000)
  {
    err_code = ruuvi_interface_scheduler_event_put(NULL, 0,
                                     task_acceleration_enter_measuring);
  }

  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
}

static void on_fifo(ruuvi_interface_gpio_evt_t event)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_acceleration_fifo_full_task);
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
}

static void on_movement(ruuvi_interface_gpio_evt_t event)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_acceleration_enter_measuring);
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
}

ruuvi_driver_status_t task_acceleration_init(void)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_driver_bus_t bus = RUUVI_DRIVER_BUS_NONE;
  uint8_t handle = 0;
  m_nbr_movements = 0;
  ruuvi_interface_timer_create(&m_measurement_timer, RUUVI_INTERFACE_TIMER_MODE_REPEATED, on_timer);
  #if RUUVI_BOARD_ACCELEROMETER_LIS2DH12_PRESENT
  err_code = RUUVI_DRIVER_SUCCESS;
  // Only SPI supported for now
  bus = RUUVI_DRIVER_BUS_SPI;
  handle = RUUVI_BOARD_SPI_SS_ACCELEROMETER_PIN;
  err_code |= ruuvi_interface_lis2dh12_init(&acceleration_sensor, bus, handle);
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_ERROR_NOT_FOUND);

  if(RUUVI_DRIVER_SUCCESS == err_code)
  {
    ruuvi_driver_status_t err_code = ruuvi_interface_scheduler_event_put(NULL, 0,
                                   task_acceleration_enter_measuring);

    // Setup FIFO and activity interrupts
    ruuvi_interface_gpio_id_t fifo_pin;
    fifo_pin.pin = RUUVI_BOARD_INT_ACC1_PIN;
    err_code |= ruuvi_interface_gpio_interrupt_enable(fifo_pin, RUUVI_INTERFACE_GPIO_SLOPE_LOTOHI, RUUVI_INTERFACE_GPIO_MODE_INPUT_PULLDOWN, on_fifo);
    err_code |= ruuvi_interface_lis2dh12_fifo_use(true);
    err_code |= ruuvi_interface_lis2dh12_fifo_interrupt_use(true);
    ruuvi_interface_gpio_id_t act_pin;
    act_pin.pin = RUUVI_BOARD_INT_ACC2_PIN;
    err_code |= ruuvi_interface_gpio_interrupt_enable(act_pin, RUUVI_INTERFACE_GPIO_SLOPE_LOTOHI, RUUVI_INTERFACE_GPIO_MODE_INPUT_PULLDOWN, on_movement);

    err_code |= ruuvi_interface_timer_start(m_measurement_timer, APPLICATION_ACCELEROMETER_TIMER_INTERVAL);

    return err_code;
  }

  #endif
  // Return error if usable acceleration sensor was not found.
  return RUUVI_DRIVER_ERROR_NOT_FOUND;
}

ruuvi_driver_status_t task_acceleration_data_log(const ruuvi_interface_log_severity_t
    level)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_interface_acceleration_data_t data;

  if(NULL == acceleration_sensor.data_get) { return RUUVI_DRIVER_ERROR_INVALID_STATE; }

  err_code |= acceleration_sensor.data_get(&data);
  char message[128] = {0};
  snprintf(message, sizeof(message), "Time: %lu\r\n",
           (uint32_t)(data.timestamp_ms & 0xFFFFFFFF));
  ruuvi_interface_log(level, message);
  snprintf(message, sizeof(message), "X: %.3f\r\n", data.x_g);
  ruuvi_interface_log(level, message);
  snprintf(message, sizeof(message), "Y: %.3f\r\n", data.y_g);
  ruuvi_interface_log(level, message);
  snprintf(message, sizeof(message), "Z: %.3f\r\n", data.z_g);
  ruuvi_interface_log(level, message);
  return err_code;
}

ruuvi_driver_status_t task_acceleration_data_get(ruuvi_interface_acceleration_data_t*
    const data)
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
  // No implementation needed
  return RUUVI_DRIVER_SUCCESS;
}

ruuvi_driver_status_t task_acceleration_movement_count_get(uint8_t* const count)
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

void task_acceleration_get_samples(task_acceleration_data_t** p_event_data, size_t* nsamples)
{
  *p_event_data = bdata;
  *nsamples = series_length;
}

uint64_t task_acceleration_get_data_age(void)
{
  return (ruuvi_interface_rtc_millis() - m_last_run) / 1000;
}

uint16_t task_acceleration_get_series_count(void)
{
  return series_counter;
}

uint8_t task_acceleration_get_scale(void)
{
  return m_series_scale;
}

uint16_t task_acceleration_get_threshold(void)
{
  return (uint16_t)m_activity_ths*1000;
}