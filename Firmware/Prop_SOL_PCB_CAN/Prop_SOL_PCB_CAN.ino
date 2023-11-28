#include <Wire.h>
#include <Adafruit_INA260.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"

// Task handles
TaskHandle_t transmitTaskHandle;
TaskHandle_t receiveTaskHandle;

// Function prototypes
void transmitTask(void *pvParameters);
void receiveTask(void *pvParameters);
void powerTask(void *pvParameters);
void commandTask(void *pvParameters);

// CAN TWAI message to send
twai_message_t txMessage;

// Pins used to connect to CAN bus transceiver:
#define CAN_RX 10
#define CAN_TX 9
int canTXRXcount[2] = {0,0};

#define CHANNEL_0_PIN 11
#define CHANNEL_1_PIN 12
#define CHANNEL_2_PIN 13
#define CHANNEL_3_PIN 14
#define CHANNEL_4_PIN 18
#define EXTERNAL_POWER_PIN 8

Adafruit_INA260 ina260;
QueueHandle_t powerQueue;
QueueHandle_t channelQueue;

struct PowerData {
  float voltage;
  float current;
} data;

int printCAN = 1;

struct PinStatus {
  volatile int pinStatus[5];
} pindata;



void setup() {
  Serial.begin(921600);
  Wire.begin(16, 17); // SDA on GPIO 16, SCL on GPIO 17
  pinMode(EXTERNAL_POWER_PIN, INPUT_PULLUP);
  pinMode(CHANNEL_0_PIN, OUTPUT);
  pinMode(CHANNEL_1_PIN, OUTPUT);
  pinMode(CHANNEL_2_PIN, OUTPUT);
  pinMode(CHANNEL_3_PIN, OUTPUT);
  pinMode(CHANNEL_4_PIN, OUTPUT);

  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  
  ina260.setAveragingCount(INA260_COUNT_16);   // set the number of samples to average
  // set the time over which to measure the current and bus voltage
  ina260.setVoltageConversionTime(INA260_TIME_140_us);
  ina260.setCurrentConversionTime(INA260_TIME_140_us);
  ina260.setMode(INA260_MODE_CONTINUOUS);

  powerQueue = xQueueCreate(1, sizeof(struct PowerData));
  channelQueue = xQueueCreate(1, sizeof(struct PinStatus));

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install and start the TWAI driver
  esp_err_t canStatus = twai_driver_install(&g_config, &t_config, &f_config);
  if (canStatus == ESP_OK) {
    Serial.println("CAN Driver installed");
  } else {
    Serial.println("CAN Driver installation failed");
  }
  twai_start();
  // Prepare the message to send
  txMessage.identifier = 0x11;           // Example identifier
  txMessage.flags = TWAI_MSG_FLAG_EXTD;  // Example flags (extended frame)
  txMessage.data_length_code = 8;        // Example data length (8 bytes)
  txMessage.data[0] = 0xFF;              // Reserved for message type
  txMessage.data[1] = 0xFF;              // Sensor 1: Temperature
  txMessage.data[2] = 0xFF;              // Sensor 2: Humidity
  txMessage.data[3] = 0xFF;              // Sensor 3: X axis Acceleration
  txMessage.data[4] = 0xFF;              // Sensor 4: Y axis Acceleration
  txMessage.data[5] = 0xFF;              // Sensor 5: Z axis Acceleration
  txMessage.data[6] = 0xFF;              // Sensor 6: Power Use in W
  txMessage.data[7] = 0xFF;              // Reserved for message Count

  Serial.println("CAN Started");

  // Create and start the tasks
  xTaskCreatePinnedToCore(powerTask, "PowerTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(commandTask, "CommandTask", 4096, NULL, 1, NULL, 0);
   // Create transmit task
  xTaskCreatePinnedToCore(transmitTask, "Transmit Task", 4096, NULL, 1, &transmitTaskHandle, 0);
  // Create receive task
  xTaskCreatePinnedToCore(receiveTask, "Receive Task", 2048, NULL, 1, &receiveTaskHandle, 1);

  Serial.println("RTOS Tasks Created");
 }

void loop() {
  // Check if there's power data in the queue
  struct PowerData data;
  struct PinStatus pindata;
  if (xQueueReceive(powerQueue, &data, 0) == pdTRUE) {
    Serial.print("Bus Voltage: ");
    Serial.print(data.voltage);
    Serial.print(" V, Current: ");
    Serial.print(data.current);
    Serial.println(" A");
  }
  if (xQueueReceive(channelQueue, &pindata, 0) == pdTRUE) {
    for (int i = 0; i < 5; i++){
      Serial.print("Channel ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println(pindata.pinStatus[i]);
    }

    txMessage.identifier = 0x51;           // Example identifier
    txMessage.flags = TWAI_MSG_FLAG_EXTD;  // Example flags (extended frame)
    txMessage.data_length_code = 8;        // Example data length (8 bytes)          
    twai_transmit(&txMessage, pdMS_TO_TICKS(1));
  }
}

void powerTask(void *pvParameters) {
  (void)pvParameters;

  while (1) {
    float voltage = ina260.readBusVoltage()/1000; // mV -> V
    float current = ina260.readCurrent()/1000; // mA -> A
    struct PowerData data;
    data.voltage = voltage;
    data.current = current;

    // Send power data to the queue
    xQueueSend(powerQueue, &data, portMAX_DELAY);

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Adjust the delay as needed
  }
}

void commandTask(void *pvParameters) {
  (void)pvParameters;
  struct PinStatus pindata;
  while (1) {
    char command = Serial.read();
    char mode;
    int channel;
    switch (command) {
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
        mode = Serial.read();
        int channelPin;
        if (command == '0') channelPin = CHANNEL_0_PIN;
        else if (command == '1') channelPin = CHANNEL_1_PIN;
        else if (command == '2') channelPin = CHANNEL_2_PIN;
        else if (command == '3') channelPin = CHANNEL_3_PIN;
        else if (command == '4') channelPin = CHANNEL_4_PIN;

        if (command == '0') channel = 0;
        else if (command == '1') channel = 1;
        else if (command == '2') channel = 2;
        else if (command == '3') channel = 3;
        else if (command == '4') channel = 4;

        if (mode == '0') {
          digitalWrite(channelPin, LOW);
          pindata.pinStatus[channel] = 0;
          txMessage.data[channel] = 0;
        } else if (mode == '1') {
          digitalWrite(channelPin, HIGH);
          pindata.pinStatus[channel] = 1;
          txMessage.data[channel] = 1;
        }
        xQueueSend(channelQueue, &pindata, portMAX_DELAY);
        break;
      default:
        //Serial.println("Invalid command");
        delay(10);
    }
  }
}

void transmitTask(void *pvParameters) {
  static int count;
  Serial.println("Start Transmit DAQ");
  while (1) {
    yield();
    delay(10);
  }
}

void receiveTask(void *pvParameters) {
  static String receivedCANmessagetoprint;
  while(1){
    if (printCAN == 1){
      twai_message_t rxMessage;
      esp_err_t receiveerror = twai_receive(&rxMessage, pdMS_TO_TICKS(50));
      if (receiveerror == ESP_OK) {
        //Print the received message
        canTXRXcount[1] += 1;
        receivedCANmessagetoprint += "Total Can Received: ";
        receivedCANmessagetoprint += String(canTXRXcount[1]) + "\n";
        receivedCANmessagetoprint += "Received Message - ID: 0x";
        receivedCANmessagetoprint += String(rxMessage.identifier, HEX);
        receivedCANmessagetoprint += ", DLC: ";
        receivedCANmessagetoprint += String(rxMessage.data_length_code);
        receivedCANmessagetoprint += ", Data: ";
        for (uint8_t i = 0; i < rxMessage.data_length_code; i++) {
          receivedCANmessagetoprint += String(rxMessage.data[i], HEX);
          receivedCANmessagetoprint += " ";
        }
        receivedCANmessagetoprint += "\n";
        Serial.println(receivedCANmessagetoprint);
        receivedCANmessagetoprint = "";
      }
    }
  }
}

