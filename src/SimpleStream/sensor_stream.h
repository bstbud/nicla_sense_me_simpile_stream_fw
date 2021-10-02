#ifndef __SENSOR_CONFIG_H__
#define __SENSOR_CONFIG_H__

#include <ArduinoJson.h>

#define USE_BLE 1

#define USE_SECOND_SERIAL_PORT_FOR_OUTPUT 0

#define CFG_IMITATE_NANO33BLE 0

#define CFG_LED_ON_INTERVAL 50
#define CFG_LED_OFF_INTERVAL 3000


#define ENABLE_ACCEL 1
#define ENABLE_GYRO  1
#define ENABLE_MAG   0
#define ENABLE_TEMP  0
#define ENABLE_HUMID 0
#define ENABLE_BARO  0  //ambient pressure sensor
#define ENABLE_GAS   0

#define ENABLE_AUDIO 0


#define ODR_ACC 100
#define ODR_GYR ODR_ACC
#define ODR_MAG 25
#define ODR_TEMP 1
#define ODR_HUMID 1
#define ODR_BARO  1
#define ODR_GAS  1

const int WRITE_BUFFER_SIZE = 256;  //about 164B

typedef unsigned long time_ms_t;

/**
 * BLE Settings
 */


#if USE_BLE
#define MAX_NUMBER_OF_COLUMNS 10
#define MAX_SAMPLES_PER_PACKET 1
#else
#define MAX_NUMBER_OF_COLUMNS 20
#define MAX_SAMPLES_PER_PACKET 6
#endif  // USE_BLE

#define DELAY_BRDCAST_JSON_DESC_SENSOR_CONFIG 1000

/**
 * Serial Port Settings
 */
#if USE_SECOND_SERIAL_PORT_FOR_OUTPUT
#define SERIAL_BAUD_RATE 115200 *4
#else
#define SERIAL_BAUD_RATE 115200 * 8
#endif //USE_SECOND_SERIAL_PORT_FOR_OUTPUT


#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG || ENABLE_TEMP || ENABLE_HUMID || ENABLE_BARO || ENABLE_GAS
int setup_sensors(JsonDocument& config_message, int column_start);
int16_t* get_sensor_data_buffer();
int update_sensor_data_col(int startIndex);


#endif  //#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG

#endif  //__SENSOR_CONFIG_H__
