/*
 * This sketch shows how nicla can be used in standalone mode.
 * Without the need for an host, nicla can run sketches that
 * are able to configure the bhi sensors and are able to read all
 * the bhi sensors data.
*/

#include "Arduino.h"
#include "sensor_stream.h"

#include "Arduino_BHY2.h"
#include "Nicla_System.h"   //must be placed after "ArduinoJSon.h"

#if USE_BLE
#include "ArduinoBLE.h"

const char* nameOfPeripheral = "Nicla Sense ME DCL Endpoint";
const char* uuidOfService    = "16480000-0525-4ad5-b4fb-6dd83f49546b";
const char* uuidOfConfigChar = "16480001-0525-4ad5-b4fb-6dd83f49546b";
const char* uuidOfDataChar   = "16480002-0525-4ad5-b4fb-6dd83f49546b";

bool WRITE_BUFFER_FIXED_LENGTH = false;

// BLE Service
static
BLEService sensorColService(uuidOfService);

// RX / TX Characteristics
BLECharacteristic configChar(uuidOfConfigChar,
                             BLERead,
                             WRITE_BUFFER_SIZE,
                             WRITE_BUFFER_FIXED_LENGTH);
BLEDescriptor     configNameDescriptor("2901", "Sensor Configuration");
BLECharacteristic sensorDataChar(uuidOfDataChar,
                                 BLERead | BLENotify,
                                 20,
                                 WRITE_BUFFER_FIXED_LENGTH);
BLEDescriptor     sensorDataDescriptor("2901", "Sensor Data TX");
#endif  // USE_BLE

static int8_t ble_output_buffer[WRITE_BUFFER_SIZE];

extern int actual_odr;

static unsigned long currentMs;
static unsigned long previousMs;
static uint16_t interval = 1000 / ODR_ACC;
static bool config_received = false;
static bool ble_connected;

DynamicJsonDocument config_message(256);
#if USE_SECOND_SERIAL_PORT_FOR_OUTPUT
auto& dataOutSerial = Serial1;
#else
auto& dataOutSerial = Serial;
#endif //USE_SECOND_SERIAL_PORT_FOR_OUTPUT

static
int column_index = 0;

static void sendJsonConfig()
{
#if USE_BLE
    serializeJson(config_message, ble_output_buffer, WRITE_BUFFER_SIZE);
    configChar.writeValue(ble_output_buffer, WRITE_BUFFER_SIZE);
#else
    serializeJson(config_message, (void *)ble_output_buffer, WRITE_BUFFER_SIZE);
    dataOutSerial.println((char*)ble_output_buffer);
    dataOutSerial.flush();
#endif  // USE_BLE
}

#if USE_BLE
/*
 * LEDS
 */

uint32_t ledColor  = off;
void connectedLight()
{
    if (ledColor != blue) {
        //nicla::leds.setColor(blue);
    }
}

void disconnectedLight()
{
    if (ledColor != green) {
        //nicla::leds.setColor(green);
    }
}

uint32_t get_free_memory_size()
{
    const uint32_t START = 16 * 1024;
    uint8_t *mem;
    uint32_t size = START;
    uint32_t floor = 0;

    while (size > 0) {
        mem = (uint8_t *) malloc (size);
        if (mem != NULL) {
            free(mem);
            floor = size;
            size = (size << 1);
            break;
        } else {
            size = size >> 1;
        }
    }

    while (size > floor) {
        mem = (uint8_t *) malloc (size);
        if (mem != NULL) {
            free(mem);
            break;
        } else {
            size--;
        }
    }

    Serial.print("free mem size: ");
    Serial.println(String(size));
    return size;
}

void onBLEConnected(BLEDevice central)
{
    Serial.print("Connected event, central: ");
    Serial.println(central.address());
    connectedLight();
    ble_connected = true;
}

void onBLEDisconnected(BLEDevice central)
{
    Serial.print("Disconnected event, central: ");
    Serial.println(central.address());
    disconnectedLight();
    BLE.setConnectable(true);
    ble_connected = false;
}


static void setup_ble()
{
    if (!BLE.begin())
    {
        Serial.println("starting BLE failed!");
        while (1)
            ;
    }

    BLE.setLocalName(nameOfPeripheral);
    BLE.setAdvertisedService(sensorColService);
    //BLE.setConnectionInterval(0x0006, 0x0007);  // 1.25 to 2.5ms
    //BLE.noDebug();

    configChar.addDescriptor(configNameDescriptor);
    sensorDataChar.addDescriptor(sensorDataDescriptor);
    sensorColService.addCharacteristic(configChar);
    sensorColService.addCharacteristic(sensorDataChar);

    delay(1000);
    BLE.addService(sensorColService);

    // Bluetooth LE connection handlers.
    BLE.setEventHandler(BLEConnected, onBLEConnected);
    BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

    BLE.advertise();

    Serial.println("Bluetooth device active, waiting for connections...");
}
#endif  //#if USE_BLE

void setup()
{
  Serial.begin(115200);
  while(!Serial);

  NiclaSettings niclaSettings(NICLA_I2C, 0, NICLA_VIA_ESLOV, 0);
  BHY2.begin(niclaSettings);

#if USE_BLE
    setup_ble();
#endif  // USE_BLE

#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG || ENABLE_GAS || ENABLE_TEMP
    column_index = setup_sensors(config_message, column_index);
#endif
    config_message["samples_per_packet"] = MAX_SAMPLES_PER_PACKET;

    delay(1000);
    sendJsonConfig();

    get_free_memory_size();
}


static uint8_t packetNum      = 0;
static uint8_t sensorRawIndex = 0;

void loop()
{
    bool connected_to_host = false;
    BHY2.update();

    currentMs = millis();
#if USE_BLE
    BLEDevice central = BLE.central();
    if (central)
    {
        if (central.connected())
        {
            //connectedLight();
        }
    }
    else
    {
        //disconnectedLight();
    }

    if (ble_connected) {
        connected_to_host = true;
    }
#else
    if (!config_received)
    {
        sendJsonConfig();
        delay(DELAY_BRDCAST_JSON_DESC_SENSOR_CONFIG);
        if (dataOutSerial.available() > 0)
        {
            String rx = dataOutSerial.readString();
            Serial.println(rx);
            if (rx.equals("connect") || rx.equals("cnnect"))
            {
#if USE_SECOND_SERIAL_PORT_FOR_OUTPUT
                Serial.println("Got Connect message");

#endif
                config_received = true;
            }
            else
            {
            }
        }

    }
    else
    {
        if (dataOutSerial.available() > 0)
        {
            String rx = dataOutSerial.readString();
            if( rx.equals("disconnect"))
            {
                config_received = false;
            }
        }
    }

    if (config_received)
    {
        connected_to_host = true;
    }
#endif

    if (connected_to_host)
    {
        //CHECKME
        if ((int32_t)currentMs - (int32_t)previousMs >= interval)
        {
            // save the last time you blinked the LED
            previousMs = currentMs;
#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
            sensorRawIndex = update_sensor_data_col(sensorRawIndex);
            packetNum++;
            int16_t* pData = get_sensor_data_buffer();
#if USE_BLE
            sensorDataChar.writeValue((void*) pData, sensorRawIndex * sizeof(int16_t));
            sensorRawIndex = 0;
            memset(pData, 0, MAX_NUMBER_OF_COLUMNS * MAX_SAMPLES_PER_PACKET * sizeof(int16_t));
#else
            if (packetNum == MAX_SAMPLES_PER_PACKET)
            {
                dataOutSerial.write((uint8_t*) pData, sensorRawIndex * sizeof(int16_t));
                dataOutSerial.flush();
                sensorRawIndex = 0;
                memset(pData, 0, MAX_NUMBER_OF_COLUMNS * MAX_SAMPLES_PER_PACKET * sizeof(int16_t));
                packetNum = 0;
            }
#endif  // USE_BLE

#endif  //#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
        }
    }
}
