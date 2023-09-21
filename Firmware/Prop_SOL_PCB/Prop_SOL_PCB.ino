#include <Wire.h>
#include <Adafruit_INA260.h>

#define CHANNEL_0_PIN 11
#define CHANNEL_1_PIN 12
#define CHANNEL_2_PIN 13
#define CHANNEL_3_PIN 14
#define CHANNEL_4_PIN 18
#define EXTERNAL_POWER_PIN 8

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

void commandTask(void *pvParameters) {
  (void)pvParameters;

  while (1) {
    char command = Serial.read();
    char mode;
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

        if (mode == '0') {
          digitalWrite(channelPin, LOW);
          Serial.print("Channel ");
          Serial.print(command);
          Serial.println(" turned OFF");
        } else if (mode == '1') {
          digitalWrite(channelPin, HIGH);
          Serial.print("Channel ");
          Serial.print(command);
          Serial.println(" turned ON");
        } else {
          Serial.println("Invalid mode");
        }
    
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

  // Create and start the tasks
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
  }
}
