/**
 * Ruuvi Firmware 3.x GATT tasks.
 *
 * License: BSD-3
 * Author: Otso Jousimaa <otso@ojousima.net>
 **/

#include "application_config.h"
#include "ruuvi_boards.h"
#include "ruuvi_driver_error.h"
#include "ruuvi_interface_communication_ble4_gatt.h"
#include "ruuvi_interface_communication_radio.h"
#include "ruuvi_interface_communication.h"
#include "ruuvi_interface_environmental.h"
#include "ruuvi_interface_log.h"
#include "ruuvi_interface_rtc.h"
#include "ruuvi_interface_scheduler.h"
#include "ruuvi_interface_watchdog.h"
#include "task_acceleration.h"
#include "task_adc.h"
#include "task_environmental.h"
#include "task_gatt.h"
#include "ringbuffer.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Buffer for GATT data
static uint8_t buffer[400];
static ringbuffer_t tx_buffer;
static size_t buffer_index = 0;
static volatile bool connected = false;

static ruuvi_interface_communication_t channel;


// Push out queued gatt messages
static void task_gatt_queue_process(void* p_event_data, uint16_t event_size)
{
  if(NULL == channel.send) { return; }

  if(ringbuffer_empty(&tx_buffer))
  {
    ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, "Buffer processed\r\n");
    return;
  }

  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  static uint64_t last = 0;
  static uint8_t processed = 0;
  char log[128];
  snprintf(log, 128, "Processed %d elements in %lu ms\r\n", processed,
           (uint32_t)(ruuvi_interface_rtc_millis() - last));
  ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, log);
  processed = 0;
  ruuvi_interface_communication_message_t* p_msg;
  // Queue as many transmissions as possible
  last = ruuvi_interface_rtc_millis();

  do
  {
    if(buffer_index > ringbuffer_get_count(&tx_buffer)) { break; }

    // Bluetooth driver takes address of data. Therefore data must be stored
    // until it is sent - do not discard here.
    p_msg = ringbuffer_peek_at(&tx_buffer, buffer_index);
    err_code = channel.send(p_msg);

    if(RUUVI_DRIVER_SUCCESS == err_code)
    {
      buffer_index++;
      processed++;
    }
  } while(RUUVI_DRIVER_SUCCESS == err_code);
}

ruuvi_driver_status_t task_gatt_init(void)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_interface_communication_ble4_gatt_dis_init_t dis = {0};
  uint64_t id;
  err_code |= ruuvi_interface_communication_id_get(&id);
  size_t index = 0;

  for(size_t ii = 0; ii < 8; ii ++)
  {
    index += snprintf(dis.deviceid + index, sizeof(dis.deviceid) - index, "%02X",
                      (uint8_t)(id >> ((7 - ii) * 8)) & 0xFF);

    if(ii < 7) { index += snprintf(dis.deviceid + index, sizeof(dis.deviceid) - index, ":"); }
  }

  memcpy(dis.fw_version, APPLICATION_FW_VERSION, sizeof(APPLICATION_FW_VERSION));
  memcpy(dis.model, RUUVI_BOARD_MODEL_STRING, sizeof(RUUVI_BOARD_MODEL_STRING));
  memcpy(dis.manufacturer, RUUVI_BOARD_MANUFACTURER_STRING,
         sizeof(RUUVI_BOARD_MANUFACTURER_STRING));
  err_code |= ruuvi_interface_communication_ble4_gatt_init();
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
  err_code |= ruuvi_interface_communication_ble4_gatt_nus_init(&channel);
  channel.on_evt = task_gatt_on_gatt;
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
  err_code |= ruuvi_interface_communication_ble4_gatt_dfu_init();
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
  err_code |= ruuvi_interface_communication_ble4_gatt_dis_init(&dis);
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
  err_code |= ruuvi_interface_communication_ble4_gatt_advertise_connectablity(true, "Ruuvi",
              true, true);
  RUUVI_DRIVER_ERROR_CHECK(err_code, RUUVI_DRIVER_SUCCESS);
  size_t max_messages = sizeof(buffer) / sizeof(ruuvi_interface_communication_message_t);
  ringbuffer_init(&tx_buffer, max_messages, sizeof(ruuvi_interface_communication_message_t),
                  &buffer);
  return err_code;
}


//ruuvi_driver_status_t task_gatt_on_accelerometer(ruuvi_interface_communication_evt_t evt)
//{
//  /*if(evt == FIFO_FULL)
//  {
//    // FIFO READ
//
//    // IF CONNECTED
//      // FIFO SEND
//  }*/
//  return RUUVI_DRIVER_SUCCESS;
//}
/*
ruuvi_driver_status_t task_gatt_on_advertisement(ruuvi_interface_communication_evt_t evt, void* p_data, size_t data_len)
{
  return RUUVI_DRIVER_SUCCESS;
}
*/
//ruuvi_driver_status_t task_gatt_on_button(ruuvi_interface_communication_evt_t evt)
//{
//  // BECOME_CONNECTABLE
//  return RUUVI_DRIVER_SUCCESS;
//}

ruuvi_driver_status_t task_gatt_on_gatt(ruuvi_interface_communication_evt_t evt,
                                        void* p_data, size_t data_len)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  ruuvi_interface_communication_message_t msg = {0};

  switch(evt)
  {
    case RUUVI_INTERFACE_COMMUNICATION_CONNECTED:
      ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, "Connected \r\n");
      memcpy(msg.data, "Hello! Here's data!", 20);
      msg.data_length = 20;
      buffer_index = 0;
      task_gatt_send(&msg);
      connected = true;
      break;

    // TODO: Handle case where connection was made but NUS was not registered
    case RUUVI_INTERFACE_COMMUNICATION_DISCONNECTED:
      ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, "Disconnected \r\n");
      // Todo: data advertising
      ruuvi_interface_communication_ble4_gatt_advertise_connectablity(true, "Ruuvi", true,
          true);
      connected = false;
      break;

    case RUUVI_INTERFACE_COMMUNICATION_SENT:
      // Message has been sent. Decrease index, process queue.
      buffer_index--;
      ringbuffer_popqueue(&tx_buffer, &msg);

      if(0 == buffer_index)
      {
        ruuvi_interface_scheduler_event_put(NULL, 0,  task_gatt_queue_process);
        ruuvi_interface_watchdog_feed();
      }

      break;

    case RUUVI_INTERFACE_COMMUNICATION_RECEIVED:
      // Schedule handling command
      ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, "RX \r\n");
      char* incoming = (char*) p_data;
      if(data_len && incoming[0] == 'S')
      {
        err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_gatt_scheduler_task);
      }
      if(data_len && incoming[0] == 'R')
      {
        err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_acceleration_enter_measuring);
      }
      break;

    default:
      break;
  }

  return err_code;
}

/*
ruuvi_driver_status_t task_gatt_on_nfc(ruuvi_interface_communication_evt_t evt, void* p_data, size_t data_len)
{
  // BECOME_CONNECTABLE
  return RUUVI_DRIVER_SUCCESS;
}
*/

ruuvi_driver_status_t task_gatt_send(ruuvi_interface_communication_message_t* const msg)
{
  if(NULL == msg)          { return RUUVI_DRIVER_ERROR_NULL; }

  if(NULL == channel.send) { return RUUVI_DRIVER_ERROR_INVALID_STATE; }

  // Add to queue if there's room
  if(ringbuffer_full(&tx_buffer))
  {
    ruuvi_interface_log(RUUVI_INTERFACE_LOG_WARNING, "FIFO out of space\r\n");
    return RUUVI_DRIVER_ERROR_NO_MEM;
  }

  ringbuffer_push(&tx_buffer, msg);
  // schedule queue to be transmitted if tx is not already ongoing
  ruuvi_interface_scheduler_event_put(NULL, 0,  task_gatt_queue_process);
  return RUUVI_DRIVER_SUCCESS;
}

//handler for scheduled gatt action  - TODO: separate data encoding logic
void task_gatt_scheduler_task(void* p_event_data, uint16_t event_size)
{
  ruuvi_driver_status_t err_code = RUUVI_DRIVER_SUCCESS;
  static size_t current_index = 0;
  // Get stored acceleration data. 
  task_acceleration_data_t* p_data;
  size_t data_len;
  task_acceleration_get_samples(&p_data, &data_len);

  // If queue is processed, return
  if((current_index + 5 >= data_len) || !connected) 
  { 
    current_index = 0;
    return; 
  } 
  
  // queue messages
  while((current_index + 5 < data_len) && RUUVI_DRIVER_SUCCESS == err_code)
  {
    ruuvi_interface_communication_message_t msg = {0};

    // Send header data
    if(current_index == 0)
    {
    msg.data[2] = 0xF0;
    // Send sample index
    uint16_t sample = task_acceleration_get_series_count();
    msg.data[3] = sample >> 8 & 0xFF;
    msg.data[4] = sample >> 0 & 0xFF;
    
    //Send battery voltage at the time of the sample
    ruuvi_interface_adc_data_t battery;
    task_adc_battery_get(&battery);
    uint16_t voltage = (uint16_t)(battery.adc_v*1000);
    msg.data[5] = voltage >> 8 & 0xFF;
    msg.data[6] = voltage >> 0 & 0xFF;

    // Send environmental data at the time of the sample
    ruuvi_interface_environmental_data_t env;
    task_environmental_data_get(&env);
    int16_t temp  = (int16_t)(env.temperature_c*100);
    uint16_t pres = (uint16_t)(env.pressure_pa/10);
    uint16_t humi = (uint16_t)(env.humidity_rh*100);
    msg.data[7] = temp >> 8 & 0xFF;
    msg.data[8] = temp >> 0 & 0xFF;
    msg.data[9] = pres >> 8 & 0xFF;
    msg.data[10] = pres >> 0 & 0xFF;
    msg.data[11] = humi >> 8 & 0xFF;
    msg.data[12] = humi >> 0 & 0xFF;

    // Send age of sample in seconds
    uint32_t age = (uint32_t) task_acceleration_get_data_age();
    msg.data[13] = age >> 24 & 0xFF;
    msg.data[14] = age >> 16 & 0xFF;
    msg.data[15] = age >> 8 & 0xFF;
    msg.data[16] = age >> 0 & 0xFF;

    // Send scale of samples
    uint8_t scale = task_acceleration_get_scale();
    msg.data[17] = scale;

    // Send activity threshold in milli-g
    uint16_t threshold = task_acceleration_get_threshold();
    msg.data[18] = threshold >> 8 & 0xFF;
    msg.data[19] = threshold >> 0 & 0xFF;

    msg.data_length = 20;
    err_code = task_gatt_send(&msg);
    memset(&msg, 0, sizeof(msg));
    
    }

    // Send acceleration data
    msg.data[2] = 0xF1;
    msg.data[3] = (current_index >> 8) & 0xFF;
    msg.data[4] = (current_index >> 0) & 0xFF;

    msg.data[5] = (p_data[current_index + 0]).accx;
    msg.data[6] = (p_data[current_index + 0]).accy;
    msg.data[7] = (p_data[current_index + 0]).accz;

    msg.data[8]  = (p_data[current_index + 1]).accx;
    msg.data[9]  = (p_data[current_index + 1]).accy;
    msg.data[10] = (p_data[current_index + 1]).accz;

    msg.data[11] = (p_data[current_index + 2]).accx;
    msg.data[12] = (p_data[current_index + 2]).accy;
    msg.data[13] = (p_data[current_index + 2]).accz;

    msg.data[14] = (p_data[current_index + 3]).accx;
    msg.data[15] = (p_data[current_index + 3]).accy;
    msg.data[16] = (p_data[current_index + 3]).accz;

    msg.data[17] = (p_data[current_index + 4]).accx;
    msg.data[18] = (p_data[current_index + 4]).accy;
    msg.data[19] = (p_data[current_index + 4]).accz;
    msg.data_length = 20;
    err_code |= task_gatt_send(&msg);
    if(RUUVI_DRIVER_SUCCESS == err_code) { current_index += 5; }
  }
  // Queue next tx
  err_code |= ruuvi_interface_scheduler_event_put(NULL, 0, task_gatt_scheduler_task);
  ruuvi_interface_log(RUUVI_INTERFACE_LOG_INFO, "Batch prepared\r\n");
}