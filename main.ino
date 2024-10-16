#define TINY_GSM_MODEM_SIM7600 // Initialize SIM Module

// #define USING_WIFI  # Un-comment to use Wi-Fi
#include <Arduino.h>
#include "SoftwareSerial.h"

#include "MAX30105.h"              // MAX30102
#include <heartRate.h>             // MAX30102 - HeartRate
#include <spo2_algorithm.h>        // MAX30102 - SpO2
#include "Protocentral_MAX30205.h" // Temp sensor
#include <TinyGPSpp.h>             // GPS Module

#include <Wire.h>
#include "ArduinoJson.h"

#ifdef USING_WIFI

#include <WiFi.h>
#include <WiFiClient.h>
// #else
// #include <TinyGsmClient.h>
#endif

#include <TinyGsmClient.h>

#include <PubSubClient.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "OneButton.h"
#include <MedianFilter.h>

#include <FreeRTOS/FreeRTOS.h>

MedianFilter heartrate_filter(31, 80);
MedianFilter spo2_filter(31, 100);

#define EGC_LO_MINUS_PIN 25
#define EGC_LO_PLUS_PIN 26
#define EGC_SIG_PIN 36
#define GPS_RX_PIN 5
#define LED_PIN 2

#define measure_button_pin 0

const char apn[] = "v-internet";

const char gprsUser[] = "";
const char gprsPass[] = "";

const char *ssid = "Ha Thuy Anh";
const char *password = "12345789";

const char *broker = "mqtt.thingsboard.cloud";
// username and password
const char *mqtt_username = "lX9xUvQkYnsqDkMfRp6i";
const char *mqtt_password = "";

const char *topic = "v1/devices/me/telemetry";
const char *ecg_enable_topic = "v1/devices/me/rpc/request/+";
// {"method":"setState","params":true}

const char *topic_heartrate = "heartrate";
const char *topic_spo2 = "spo2";
const char *topic_temp = "temp";
const char *topic_ecg = "ecg";
const char *topic_gps = "gps";
const char *topic_fall = "fall";

#Phone number receive warning messages
const char *phone_number = "0336902861";

#define SerialAT Serial2

SoftwareSerial gpsSerial(GPS_RX_PIN, -1); // RX, TX

TinyGsm modem(SerialAT);

#ifdef USING_WIFI

WiFiClient client;

#else

TinyGsmClient client(modem);

#endif

PubSubClient mqtt(client);

MAX30105 heartrate_max30102_sensor;
MAX30205 temp_max30205_sensor;
Adafruit_MPU6050 mpu6050_sensor;

OneButton measure_button(measure_button_pin); // active low, pullup active

#define turn_on_led digitalWrite(LED_PIN, HIGH)
#define turn_off_led digitalWrite(LED_PIN, LOW)

void setup_ecg();
void setup_serial();
void setup_mqtt();
void setup_max30102();
void setup_network();
void setup_gps();
void setup_temp_MAX30205();
void setup_mpu6050();
void setup_led();
void setup_button();

void mqttCallback(char *topic, byte *payload, unsigned int len);
boolean mqtt_reconnect();
void publish_data(const char *topic, String data);

void read_ecg();
void read_max30102();
void read_gps();
void read_temp_MAX30205();
void read_mpu6050();
void sim_handler();
void reconnect_network();
void mqtt_handler();
void button_handler();
void threshold_handler();

void print_debug();

void TaskReadSensor(void *pvParameters);
void TaskReadGPS(void *pvParameters);
void TaskHeartRate(void *pvParameters);
void TaskButton(void *pvParameters);

void TaskECG(void *pvParameters);

TaskHandle_t TaskHeartRateHandle = NULL;

int ecg_value = 0;
float temp_value = 0;
int32_t heartrate_value = 0;
int32_t spo2_value = 0;

int32_t last_heartrate_value = 0;
int32_t last_spo2_value = 0;

String gps_lat = "";
String gps_lon = "";
bool isFallDetected = false;

int request_id = 0;

bool is_measure_button_pressed = false;

int32_t bufferLength; // data length
int8_t validSPO2;
int8_t validHeartRate;   // indicator to show if the heart rate calculation is valid
uint32_t irBuffer[100];  // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data

int16_t totalAcc;

unsigned long last_print_debug_millis = 0;
unsigned long print_debug_time = 500;

unsigned long lasttime_send_mqtt = 0;
unsigned long time_send_mqtt = 1000;

unsigned long lasttime_send_mqtt_ecg = 0;
unsigned long time_send_mqtt_ecg = 10;

//**********************************SET UP*****************************************
void setup()
{
  setup_serial();
  setup_network();
  setup_led();
  setup_ecg();
  setup_max30102();
  setup_mpu6050();
  setup_temp_MAX30205();
  setup_mqtt();
  setup_gps();
  setup_button();

  xTaskCreatePinnedToCore(TaskReadSensor, "TaskReadSensor", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskReadGPS, "TaskReadGPS", 10000, NULL, 1, NULL, 0);

  xTaskCreatePinnedToCore(TaskHeartRate, "TaskHeartRate", 10000, NULL, 1, &TaskHeartRateHandle, 0);

  xTaskCreatePinnedToCore(TaskButton, "TaskButton", 5000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskECG, "TaskECG", 5000, NULL, 1, NULL, 1);
}
//**********************************LOOP***************************************

void loop()
{
  sim_handler();
  mqtt_handler();

  threshold_handler();
  //   print_debug();
}

//**********************************ECG MODE***********************************
void TaskECG(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    if (is_measure_button_pressed)
    {
      read_ecg();
      //   publish_data(topic_ecg, String(ecg_value));
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
//*****************************************************************************

// Button to change ECG mode
void TaskButton(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    button_handler();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void TaskHeartRate(void *pvParameters)
{
  //   (void)pvParameters;
  setup_max30102();
  bufferLength = 100;
  for (byte i = 0; i < bufferLength; i++)
  {
    while (heartrate_max30102_sensor.available() == false)
    {
      heartrate_max30102_sensor.check();
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    redBuffer[i] = heartrate_max30102_sensor.getRed();
    irBuffer[i] = heartrate_max30102_sensor.getIR();
    heartrate_max30102_sensor.nextSample();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
                                         &spo2_value, &validSPO2,
                                         &heartrate_value, &validHeartRate);
  bool haveBeat = false;
  for (;;)
  {
    // Check beat
    long irValue = heartrate_max30102_sensor.getIR();

    if (irValue > 5000)
    {
      //   read_max30102();
      haveBeat = true;
    }
    else
    {
      //   Serial.println("No beat detected");
      heartrate_value = 0;
      spo2_value = 0;
      setup_max30102();
    }

    unsigned long lasttime = millis();
    unsigned long timeout = 20000;

    while (haveBeat)
    {
      read_max30102();
      irValue = heartrate_max30102_sensor.getIR();
      if (millis() - lasttime > timeout || irValue < 5000)
      {
        haveBeat = false;
      }
    }

    // read_max30102();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void TaskReadSensor(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    // read_max30102();
    read_temp_MAX30205();
    read_mpu6050();
    print_debug();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void TaskReadGPS(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    read_gps();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup_led()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void setup_serial()
{
  Serial.begin(9600);
}

// bool reconnect_network
void setup_button()
{
  measure_button.attachClick([]()
                             {
    Serial.println("Button clicked");
    is_measure_button_pressed = !is_measure_button_pressed;
    // request_id += 1;

    Serial.print("request_id: ");
    Serial.println(request_id);
    // v1/devices/me/rpc/request/16
    if (is_measure_button_pressed) {
      publish_data(
        String("v1/devices/me/rpc/request/" + String(request_id)).c_str(),
        "{\"method\":\"setState\",\"params\":true}");

      publish_data(
        String("v1/devices/me/rpc/request/" + String(request_id)).c_str(),
        "{\"method\":\"setState\",\"params\":true}");
      turn_on_led;
    } else {
      publish_data(
        String("v1/devices/me/rpc/request/" + String(request_id)).c_str(),
        "{\"method\":\"setState\",\"params\":false}");
      publish_data(
        String("v1/devices/me/rpc/request/" + String(request_id)).c_str(),
        "{\"method\":\"setState\",\"params\":false}");
      turn_off_led;
    } });

  measure_button.attachDoubleClick(
      []()
      {
        Serial.println("Button double clicked");
      });

  measure_button.attachLongPressStart([]()
                                      {
    setup_max30102();
    Serial.println("setup_max30102");
    // kill task TaskHeartRateHandle and create new task
    vTaskDelete(TaskHeartRateHandle);
    Serial.println("TaskHeartRateHandle deleted");
    // setup_max30102();
    Serial.println("TaskHeartRateHandle created");
    xTaskCreatePinnedToCore(TaskHeartRate, "TaskHeartRate", 10000, NULL, 1,
                            &TaskHeartRateHandle, 0); });
}

bool isSendSMS = false;

unsigned long lasttime_send_sms = 0;
unsigned long time_send_sms = 10000;

void threshold_handler()
{ // Send SMS when data out of safe range

  // *********************Heart rate*****************************
  if (heartrate_value > 130 && heartrate_value > 0)
  {
    Serial.println("Heart rate is too high");
    // send sms
    if (!isSendSMS)
    {
      if (modem.sendSMS(phone_number, "Heart rate is too high " + String(heartrate_value) + " bpm"))
      {
        Serial.println("SMS sent");
        isSendSMS = true;
        lasttime_send_sms = millis();
      }
      else
      {
        Serial.println("SMS not sent");
      }
    }
  }

  if (heartrate_value < 60 && heartrate_value > 0)
  {
    Serial.println("Heart rate is too low");
    // send sms
    // if (modem.sendSMS(phone_number, "Heart rate is too low"+ String(heartrate_value) + " bpm")) {
    //   Serial.println("SMS sent");
    // } else {
    //   Serial.println("SMS not sent");
    // }

    if (!isSendSMS)
    {
      if (modem.sendSMS(phone_number, "Heart rate is too low " + String(heartrate_value) + " bpm"))
      {
        Serial.println("SMS sent");
        isSendSMS = true;
        lasttime_send_sms = millis();
      }
      else
      {
        Serial.println("SMS not sent");
      }
    }
  }

  // *********************SpO2 level*****************************
  if (spo2_value < 95 && spo2_value > 0)
  {
    Serial.println("SPO2 is too low");
    // send sms
    // if (modem.sendSMS(phone_number, "SPO2 is too low")) {
    //   Serial.println("SMS sent");
    // } else {
    //   Serial.println("SMS not sent");
    // }

    if (!isSendSMS)
    {
      if (modem.sendSMS(phone_number,
                        "SPO2 is too low " + String(spo2_value) + "%"))
      {
        Serial.println("SMS sent");
        isSendSMS = true;
        lasttime_send_sms = millis();
      }
      else
      {
        Serial.println("SMS not sent");
      }
    }
  }

  // *********************Body temperature*****************************
  // if (temp_value > 38 && temp_value > 0) {
  //   Serial.println("Temperature is too high");

  //   // send sms
  //   // if (modem.sendSMS(phone_number, "Temperature is too high")) {
  //   //   Serial.println("SMS sent");
  //   // } else {
  //   //   Serial.println("SMS not sent");
  //   // }

  //   if (!isSendSMS) {
  //     if (modem.sendSMS(phone_number, "Temperature is too high " + String(temp_value) + "°C")) {
  //       Serial.println("SMS sent");
  //       isSendSMS = true;
  //       lasttime_send_sms = millis();
  //     } else {
  //       Serial.println("SMS not sent");
  //     }
  //   }
  // }

  // if (temp_value < 36.5 && temp_value > 0) {
  //   Serial.println("Temperature is too low");

  //   // send sms
  //   // if (modem.sendSMS(phone_number, "Temperature is too low")) {
  //   //   Serial.println("SMS sent");
  //   // } else {
  //   //   Serial.println("SMS not sent");
  //   // }

  //   if (!isSendSMS) {
  //     if (modem.sendSMS(phone_number, "Temperature is too low " + String(temp_value) + "°C")) {
  //       Serial.println("SMS sent");
  //       isSendSMS = true;
  //       lasttime_send_sms = millis();
  //     } else {
  //       Serial.println("SMS not sent");
  //     }
  //   }
  // }

  if (isSendSMS)
  {
    if (millis() - lasttime_send_sms > time_send_sms)
    {
      isSendSMS = false;
      lasttime_send_sms = millis();
    }
  }
}

void button_handler()
{
  measure_button.tick();
}

void setup_mpu6050()
{
  if (!mpu6050_sensor.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");
  mpu6050_sensor.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu6050_sensor.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu6050_sensor.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu6050_sensor.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu6050_sensor.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu6050_sensor.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
}

void setup_temp_MAX30205()
{
  // temp_max30205_sensor.begin();
  Serial.println("MAX30205 Found!");
}

void setup_ecg()
{
  pinMode(EGC_LO_MINUS_PIN, OUTPUT);
  pinMode(EGC_LO_PLUS_PIN, OUTPUT);
  pinMode(EGC_SIG_PIN, INPUT);
}

bool beatDetected = false;

void read_max30102()
{

  //   for(;;) {
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  for (byte i = 75; i < 100; i++)
  {
    while (heartrate_max30102_sensor.available() == false)
    {
      heartrate_max30102_sensor.check();
      //   Serial.println("check sensor");
    }
    redBuffer[i] = heartrate_max30102_sensor.getRed();
    irBuffer[i] = heartrate_max30102_sensor.getIR();
    heartrate_max30102_sensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
                                         &spo2_value, &validSPO2,
                                         &heartrate_value, &validHeartRate);

  heartrate_filter.in(heartrate_value);
  heartrate_value = heartrate_filter.out();

  spo2_filter.in(spo2_value);
  spo2_value = spo2_filter.out();
}

void read_ecg()
{
  ecg_value = analogRead(EGC_SIG_PIN);
}

void read_temp_MAX30205()
{
  temp_value = random(30, 40);
}

void setup_max30102()
{

  Wire1.begin(18, 19);
  if (!heartrate_max30102_sensor.begin(Wire1, I2C_SPEED_FAST))
  {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
  }
  heartrate_max30102_sensor.setup();
}

void reconnect_network()
{
#ifdef USING_WIFI

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to WiFi");
  }

#else

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" success");

  if (modem.isNetworkConnected())
  {
    Serial.println("Network connected");
  }
  Serial.print(F("Connecting to "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, "", ""))
  {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" success");

  if (modem.isGprsConnected())
  {
    Serial.println("GPRS connected");
  }
#endif
}

void sim_handler()
{
#ifdef USING_WIFI

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi disconnected");
    if (WiFi.begin(ssid, password))
    {
      Serial.println("WiFi re-connected");
    }
  }

#else

  if (!modem.isNetworkConnected())
  {
    Serial.println("Network disconnected");
    if (!modem.waitForNetwork(180000L, false))
    {
      Serial.println(" fail");
      //   delay(10000);
    }
    if (modem.isNetworkConnected())
    {
      Serial.println("Network re-connected");
    }

    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected())
    {
      Serial.println("4G disconnected!");
      Serial.print(F("Connecting to "));
      Serial.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass))
      {
        Serial.println(" fail");
        // delay(10000);
        return;
      }
      if (modem.isGprsConnected())
      {
        Serial.println("4G reconnected");
      }
    }
  }
#endif
}

unsigned long lasttime_detect_fall = 0;
unsigned long time_detect_fall = 10000;

void read_mpu6050()
{
  sensors_event_t a, g, temp;
  mpu6050_sensor.getEvent(&a, &g, &temp);
  totalAcc =
      abs(a.acceleration.x) + abs(a.acceleration.y) + abs(a.acceleration.z);

  if (totalAcc > 50)
  { // Fall detected
    isFallDetected = true;
    Serial.println("Fall detected");
    if (modem.sendSMS(phone_number, "Patient fall detected, please take action!"))
    {
      Serial.println("SMS sent");
    }
    else
    {
      Serial.println("SMS not sent");
    }
  }
  else
  {

    // isFallDetected = false;
    if (millis() - lasttime_detect_fall > time_detect_fall)
    {
      isFallDetected = false;
      lasttime_detect_fall = millis();
    }
  }
}

bool gps_debug = false;

void read_gps()
{
  TinyGPSPlus gps;
  while (gpsSerial.available() > 0)
  {
    char c = gpsSerial.read();
    if (gps_debug)
    {
      Serial.write(c);
    }
    if (gps.encode(c))
    {
      gps_lat = String(gps.location.lat(), 6);
      gps_lon = String(gps.location.lng(), 6);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup_gps()
{
  gpsSerial.begin(9600);
}

void setup_network()
{

#ifdef USING_WIFI

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");

  // print wifi info
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

#else
  SerialAT.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  Serial.print("Modem Info: ");
  Serial.println(modemInfo);

  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" success");

  if (modem.isNetworkConnected())
  {
    Serial.println("Network connected");
  }
  Serial.print(F("Connecting to "));
  Serial.print(apn);
  if (!modem.gprsConnect(apn, "", ""))
  {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" success");

  if (modem.isGprsConnected())
  {
    Serial.println("4G connected");
  }
#endif
}

uint32_t lastReconnectAttempt = 0;
unsigned long oldtime_send_http = 0;
unsigned long time_send_http = 10;
void mqtt_handler()
{
  if (!mqtt.connected())
  {
    Serial.println("=== MQTT NOT CONNECTED ===");
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L)
    {
      lastReconnectAttempt = t;
      if (mqtt_reconnect())
      {
        lastReconnectAttempt = 0;
      }
    }
  }
  else
  {

    if (is_measure_button_pressed)
    {
      if (millis() - lasttime_send_mqtt_ecg > time_send_mqtt_ecg)
      {
        publish_data(topic_ecg, String(ecg_value));
        lasttime_send_mqtt_ecg = millis();
      }
    }

    if (millis() - lasttime_send_mqtt > time_send_mqtt)
    {
      String json = "{\"heartrate\": " + String(heartrate_value) + ", " + "\"spo2\": " + String(spo2_value) + ", " + "\"temp\": " + String(temp_value) + ", " + "\"lat\": \"" + gps_lat + "\", " + "\"lon\": \"" + gps_lon + "\", " + "\"fall\": \"" + (isFallDetected ? "FALL DETECTED" : "Normal") + "\", " + "\"ecg\": " + String(ecg_value) + "}";

      publish_data(topic, json);
      lasttime_send_mqtt = millis();
    }
  }

  mqtt.loop();
}

void setup_mqtt()
{
  // set mqtt server
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
}

void mqttCallback(char *topic, byte *payload, unsigned int len)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");

  String message;

  for (int i = 0; i < len; i++)
  {
    message += (char)payload[i];
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // get id from topic
  // v1/devices/me/rpc/request/16

  request_id = atoi(strrchr(topic, '/') + 1);

  Serial.print("request_id: ");
  Serial.println(request_id);

  // {"method":"setState","params":true}

  StaticJsonDocument<200> doc;
  deserializeJson(doc, message);

  // const char *method = doc["method"];
  // // bool params = doc["params"];
  // is_measure_button_pressed = doc["params"];

  if (doc.containsKey("params"))
  {
    is_measure_button_pressed = doc["params"].as<bool>();
    Serial.print("is_measure_button_pressed: ");
    Serial.println(is_measure_button_pressed);

    if (is_measure_button_pressed)
    {
      turn_on_led;
    }
    else
    {
      turn_off_led;
    }
  }
}

boolean mqtt_reconnect()
{

  Serial.print("Connecting to ");
  Serial.print(broker);

  int random_number = random(0, 1000);

  String client_id = "GsmClientTest" + String(random_number);

  // Connect to MQTT Broker
  boolean status =
      mqtt.connect(client_id.c_str(), mqtt_username, mqtt_password);

  if (status == false)
  {
    Serial.println(" fail");
    return false;
  }
  Serial.println(" success");
  // mqtt.publish("/controller/datasim/number", "GsmClientTest started");
  // mqtt.subscribe("/controller/datasim/number");
  Serial.println("============================================");

  mqtt.subscribe(ecg_enable_topic);
  Serial.print("subscribe to ");
  Serial.println(ecg_enable_topic);
  Serial.println("============================================");
  return mqtt.connected();
}

void publish_data(const char *topic, String data)
{
  if (mqtt.connected())
  {
    mqtt.publish(topic, data.c_str());
  }
}

void print_debug()
{
  if (millis() - last_print_debug_millis > print_debug_time)
  {
    Serial.print("ECG: ");
    Serial.print(ecg_value);
    Serial.print(" Temp: ");
    Serial.print(temp_value);
    Serial.print(" HR: ");
    Serial.print(heartrate_value);
    Serial.print(" SPO2: ");
    Serial.print(spo2_value);
    Serial.print(" GPS: ");
    Serial.print(gps_lat);
    Serial.print(" ");
    Serial.print(gps_lon);
    Serial.print(" Fall: ");
    Serial.print(isFallDetected);
    // totalAcc
    Serial.print(" Acc: ");
    Serial.println(totalAcc);
    last_print_debug_millis = millis();
  }
}