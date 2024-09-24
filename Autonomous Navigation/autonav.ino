
#include <WiFi.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <esp_now.h>
#include <AsyncTCP.h>
#include <NTPClient.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MQTT.h>
#include <SolarCalculator.h>
#include <BluetoothSerial.h>
#include <Adafruit_Sensor.h>
#include <AsyncElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MQTT_Client.h>

//******************************************************************************************************

//Adafruit.io Setup
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME "_Maverick_"
#define AIO_KEY "aio_hUMV517MwXr4cUstKSeAnpOPnu6F"

//******************************************************************************************************

// GPS
int GPS_Course;      // variable to hold the gps's determined course to destination
int Number_of_SATS;  // variable to hold the number of satellites acquired
boolean plotOn = false;
boolean geoSyncOn = false;

TinyGPSPlus gps;

//******************************************************************************************************

// BLUETOOTH
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run
`make menuconfig` to and enable it
#endif

  String str;  // raw string received from android to arduino
int blueData;  // stores the last value sent over via bluetooth

BluetoothSerial SerialBT;

//******************************************************************************************************

// WIFI
#define WLAN_SSID "TP-LINK_D8B6"
#define WLAN_PASS "farhan2758"

WiFiClient client;

//******************************************************************************************************

// NTP CLIENT
int utc_offset = +6;
const char* ntpServer = "asia.pool.ntp.org";
const int utcOffsetInSeconds = utc_offset * 3600;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, utcOffsetInSeconds);

//******************************************************************************************************

// MQTT CLIENT
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
const char gpsPlotdata[] PROGMEM = AIO_USERNAME "/feeds/GPS/csv";
Adafruit_MQTT_Publish GPS = Adafruit_MQTT_Publish(&mqtt, gpsPlotdata);

void MQTT_connect();

//******************************************************************************************************

// Waste Sorting Setup

typedef struct struct_message {
  char data[20];
} struct_message;

// Create a struct_message to hold incoming readings
struct_message myData;

int currentPos = 90;
int waste_servo = 5;
int desiredPos;
bool sortOn = false;
Servo wasteServo;

//******************************************************************************************************

// OTA
AsyncWebServer server(80);

//******************************************************************************************************

// CORE2
TaskHandle_t Loop0;

//******************************************************************************************************

// Buzzer
int buzzerPin = 15;


// Motor Driver Setup
int motor1Pin1 = 26;
int motor1Pin2 = 27;
int enable1Pin = 18;
int motor2Pin1 = 32;
int motor2Pin2 = 33;
int enable2Pin = 19;

const int freq = 30000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 250;
int slowdutyCycle = 200;

//******************************************************************************************************

// Compass Variables & Setup
int16_t mx, my, mz;   // variables to store x,y,z axis from compass (HMC5883L)
int desired_heading;  // initialize variable - stores value for the new desired heading
int compass_heading;  // initialize variable - stores value calculated from compass readings
int compass_dev = 5;  // the amount of deviation that is allowed in the compass heading - Adjust as Needed, setting this variable too low will cause the robot to continuously pivot left and right, setting this variable too high will cause the robot to veer off course

int Heading_A;  // variable to store compass heading
int Heading_B;  // variable to store compass heading in Opposite direction
int pass = 0;   // variable to store which pass the robot is on

// Compass Calibration Variables & Setup
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int minZ = 0;
int maxZ = 0;
int offX = 0;
int offY = 0;
int offZ = 0;

int offsetX = 7;
int offsetY = -25;  //-18
int offsetZ = -21;  //-22

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(5842);

//******************************************************************************************************

// Solar Tracker Setup
double latitude = 25.1062,
       longitude = 89.0199,
       azimuth, elevation;

int elevation_servo = 4;
int azimuth_servo = 23;

boolean dockOn = false;

Servo aziServo;
Servo eleServo;

//******************************************************************************************************

// Sweep Setup
int pos = 0;  // variable to store the servo position
int sweep_servo = 13;
Servo sweepServo;

//******************************************************************************************************

// Ping Sensor Variable & Setup
boolean pingOn = false;

int trigPin = 25;
int echoPin = 34;
long duration, inches;
int Ping_distance;

unsigned long currentMillis = 0;
unsigned long previousMillis = 0;  // Store last time Ping was updated
const long interval = 200;         // Ping the Distance every X miliseconds

//*****************************************************************************************************

// GPS Variable
const double HOME_LAT = 25.106210;
const double HOME_LNG = 89.019904;

unsigned long Distance_To_Home;    // variable for storing the distance to destination
unsigned long Distance_To_Preset;  // variable for storing the distance to preset destination
float Distance_Travelled;          // variable for storing the distance travelled from defined location
int ac = 0;                        // GPS array counter
int wpCount = 0;                   // GPS waypoint counter
double Home_LATarray[25];          // variable for storing the destination Latitude - up to 50 waypoints
double Home_LONarray[25];          // variable for storing the destination Longitude - up to 50 waypoints


float Preset_LATarray[5] = { 24.372775, 24.372991, 24.373116, 24.372865, 24.372753 };
float Preset_LONarray[5] = { 88.568578, 88.568833, 88.568532, 88.568362, 88.568587 };

int presetWaypoints = 5;
int currentPresetWaypoint = 0;
int increment = 0;

//*****************************************************************************************************

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);

  // Buzzer
  pinMode(buzzerPin, OUTPUT);

  // Connection to WiFi access point.
  {
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WLAN_SSID);

    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Wi-Fi Channel: ");
    Serial.println(WiFi.channel());
    Serial.println();

    indicatorBuzzer();
  }

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    NOWmalfunctionBuzzer();
    return;
  }

  // Init time-sync
  Serial.print("Synchronizing time");
  if (!syncTime()) {
    Serial.println("Time synchronization failed. Check NTP server and network.");
    Serial.println();
    TIMEmalfunctionBuzzer();
  } else {

    Serial.println("Time synchronized");
    Serial.println();

    indicatorBuzzer();
  }

  // Connection to BLE access point.
  {
    btStart();
    Serial.println("Bluetooth On!");
    SerialBT.begin("ESP32_Bluetooth");
    Serial.println("The device started, now you can pair it with bluetooth!");
    Serial.println();

    indicatorBuzzer();
  }

  // Connection to ASYNC access point.
  {
    AsyncElegantOTA.begin(&server);
    server.begin();
    Serial.println("ASYNC server started!");
    SerialBT.println("ASYNC server started!");
    Serial.println();

    indicatorBuzzer();
  }

  // Creating Core 2 access point.
  {
    xTaskCreatePinnedToCore(
      Loop0code,
      "Loop 0",
      10000,
      NULL,
      1,
      &Loop0,
      0);
  }

  // Ping Sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Servo
  sweepServo.attach(sweep_servo);
  wasteServo.attach(waste_servo);
  aziServo.attach(azimuth_servo);
  eleServo.attach(elevation_servo);

  // Compass
  if (!mag.begin()) {
    Serial.println("Ooops, no compass detected ... Check your wiring!");
    SerialBT.println("Ooops, no compass detected ... Check your wiring!");

    COMPASSmalfunctionBuzzer();
    while (1)
      ;
  } else {
    Serial.println("Compass Started");
    SerialBT.println("Compass Started");
    displaySensorDetails();

    indicatorBuzzer();
  }

  // Motor Driver
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // Configure PWM Functionalitites
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);

  ledcAttachPin(enable1Pin, pwmChannel1);
  ledcAttachPin(enable2Pin, pwmChannel2);

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  Startup();
}

//********************************************************************************************************

// Plot Loop
void Loop0code(void* pvParameters) {
  for (;;) {
    wasteSorting();
    sweep();
    gpsPlot();
  }
}

// Main Loop
void loop() {
  bluetooth();   // Run the Bluetooth procedure to see if there is any data being sent via BT
  getGPS();      // Update the GPS location
  getCompass();  // Update the Compass Heading
  ping();
  solarTracker();
}
