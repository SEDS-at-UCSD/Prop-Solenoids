#include <Wire.h>
#include <Adafruit_INA260.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"

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

volatile int pinStatus[5] = {0,0,0,0,0};

Adafruit_INA260 ina260;
QueueHandle_t powerQueue;

struct PowerData {
  float voltage;
  float current;
} data;


void powerTask(void *pvParameters) {
  (void)pvParameters;

  while (1) {
    float voltage = ina260.readBusVoltage()/1000; // mV -> V
    float current = ina260.readCurrent()/1000; // mA -> A

    // Create a struct to hold power data
    struct PowerData {
      float voltage;
      float current;
    } data;

    data.voltage = voltage;
    data.current = current;

    // Send power data to the queue
    xQueueSend(powerQueue, &data, portMAX_DELAY);

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Adjust the delay as needed
  }
}

void command2pin(char command, char mode){
  int channelPin, statusPin;
  if (command == '0') {channelPin = CHANNEL_0_PIN; statusPin = 0;}
  else if (command == '1') {channelPin = CHANNEL_1_PIN; statusPin = 1;}
  else if (command == '2') {channelPin = CHANNEL_2_PIN; statusPin = 2;}
  else if (command == '3') {channelPin = CHANNEL_3_PIN; statusPin = 3;}
  else if (command == '4') {channelPin = CHANNEL_4_PIN; statusPin = 4;}

  if (mode == '0') {
    pinStatus[statusPin] = 0;
    digitalWrite(channelPin, LOW);
    //Serial.print("Channel ");
    //Serial.print(command);
    //Serial.println(" turned OFF");
  } else if (mode == '1') {
    pinStatus[statusPin] = 1;
    digitalWrite(channelPin, HIGH);
    //Serial.print("Channel ");
    //Serial.print(command);
    //Serial.println(" turned ON");
  } else {
    Serial.println("Invalid mode");
  }

}

void receiveTask(void *pvParameters) {
  static String receivedCANmessagetoprint;
  while(1){
    twai_message_t rxMessage;
    esp_err_t receiveerror = twai_receive(&rxMessage, pdMS_TO_TICKS(50));
    if (receiveerror == ESP_OK) {
      if (String(rxMessage.identifier, HEX) == "5f"){ //Check if Write Command issued at 0x5f (Conversion always lower case)
        Serial.println("RECEIVED CAN");
        char commands[5] = {'0','1','2','3','4'};
        char modes[2] = {'0','1'};
        for (int i = 0; i < 5; i++){
          int mode = rxMessage.data[i];
          command2pin(commands[i],modes[mode]);
        }
      }
      else if (rxMessage.identifier > 0x1F){
        Serial.print("Received from: ");
        Serial.println(String(rxMessage.identifier, HEX));
      }
      


      /** This is for generic printing
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
      **/
    }
  }
}

void commandTask(void *pvParameters) {
  (void)pvParameters;

  while (1) {
    char command = Serial.read();
    char mode = Serial.read();;
    switch (command) {
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
        command2pin(command,mode);
        break;
      case 'e':
        pinMode(EXTERNAL_POWER_PIN, INPUT_PULLUP);
        Serial.print("Analog Ext Power Pin:");
        Serial.print(analogRead(EXTERNAL_POWER_PIN));
        if (digitalRead(EXTERNAL_POWER_PIN) == LOW) {
          Serial.println("\t External power is connected.");
        } else {
          Serial.println("\t External power is not connected.");
        }
        break;
      default:
        //Serial.println("Invalid command");
        delay(10);
    }
  }
}

void setup() {
  Serial.begin(921600);
  
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

  txMessage.identifier = 0x51;           // Solenoid Board identifier Board 0x5X
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
  
  ina260.setAveragingCount(INA260_COUNT_4);   // set the number of samples to average
  // set the time over which to measure the current and bus voltage
  ina260.setVoltageConversionTime(INA260_TIME_140_us);
  ina260.setCurrentConversionTime(INA260_TIME_140_us);
  ina260.setMode(INA260_MODE_CONTINUOUS);

  powerQueue = xQueueCreate(1, sizeof(struct PowerData));

  // Create and start the tasks
  xTaskCreatePinnedToCore(receiveTask, "Receive Task", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(powerTask, "PowerTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(commandTask, "CommandTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
  // Check if there's power data in the queue
  struct PowerData data;
  if (xQueueReceive(powerQueue, &data, 0) == pdTRUE) {
    Serial.print("Bus Voltage: ");
    Serial.print(data.voltage);
    Serial.print(" V, Current: ");
    Serial.print(data.current);
    Serial.println(" A");

    for (int i = 0; i < 5; i++){
      txMessage.data[i] = pinStatus[i];
      Serial.print("\t Pin ");
      Serial.print(i);
      Serial.print(":");
      Serial.print(pinStatus[i]);
      }
      Serial.println();

      twai_transmit(&txMessage, pdMS_TO_TICKS(1));
      Serial.println("CAN sent");
  }
  
}
