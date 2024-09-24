#include <WiFi.h>
#include <esp_now.h>
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/edgeimpulse/fomo.h>
#include <Waste_Segregation_with_ESP32-CAM_inferencing.h>

using eloq::camera;
using eloq::ei::fomo;

//******************************************************************************************************

// WIFI
#define WLAN_SSID "TP-LINK_D8B6"
#define WLAN_PASS "farhan2758"

WiFiClient client;

//******************************************************************************************************

// ESP-NOW

uint8_t broadcastAddress[] = { 0x7C, 0x9E, 0xBD, 0x62, 0x64, 0x74 };

// Define variables to store readings to be sent
bool espcamPaper,
  espcamMetal,
  espcamGlass,
  espcamPlastic,
  espcamBio;

// Define variables to store incoming readings
bool esp32Data;

// Variable to store if sending data was successful
String success;

typedef struct struct_message {
  bool servo,
    paper,
    metal,
    glass,
    plastic,
    bio;
} struct_message;

// Create a struct_message for ESP32-Cam to hold outgoing readings
struct_message outgoingReadings;

// Create a struct_message to hold incoming readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;



void setup() {
  delay(3000);
  Serial.begin(115200);

  // Connection to WiFi access point.
  {
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WLAN_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println();
  }

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Waste Segregation

  esp_now_register_send_cb(OnDataSent);   // get the status of Trasnmitted packet
  registerPeer();
  addPeer();
  esp_now_register_recv_cb(OnDataRecv);   // Register for a callback function that will be called when data is received

  // camera settings
  camera.pinout.aithinker();
  camera.brownout.disable();
  // NON-PSRAM FOMO only works on 96x96 (yolo) RGB565 images
  camera.resolution.yolo();
  camera.pixformat.rgb565();

  // init camera
  while (!camera.begin().isOk())
    Serial.println(camera.exception.toString());

  Serial.println("Camera OK");
  Serial.println("Put object in front of camera");
}



void loop() {
 
  // capture picture
  if (!camera.capture().isOk()) {
    Serial.println(camera.exception.toString());
    return;
  }

  // run FOMO
  if (!fomo.run().isOk()) {
    Serial.println(fomo.exception.toString());
    return;
  }

  // how many objects were found?
  Serial.printf("Found %d object(s) in %dms\n", fomo.count(), fomo.benchmark.millis());

  // if no object is detected, return
  if (!fomo.foundAnyObject())
    return;

  // Iterate through each detected object
  fomo.forEach([](int i, bbox_t bbox) {
    Serial.printf(
      "#%d) Found %s at (x = %d, y = %d) (size %d x %d). "
      "Proba is %.2f\n",
      i + 1,
      bbox.label,
      bbox.x,
      bbox.y,
      bbox.width,
      bbox.height,
      bbox.proba);

    if (strcmp(bbox.label, "paper") == 0) {
      outgoingReadings.paper = true;
      outgoingReadings.metal = false;
      outgoingReadings.glass = false;
      outgoingReadings.plastic = false;
      outgoingReadings.bio = false;
    } else if (strcmp(bbox.label, "metal") == 0) {
      outgoingReadings.paper = false;
      outgoingReadings.metal = true;
      outgoingReadings.glass = false;
      outgoingReadings.plastic = false;
      outgoingReadings.bio = false;
    } else if (strcmp(bbox.label, "glass") == 0) {
      outgoingReadings.paper = false;
      outgoingReadings.metal = false;
      outgoingReadings.glass = true;
      outgoingReadings.plastic = false;
      outgoingReadings.bio = false;
    } else if (strcmp(bbox.label, "plastic") == 0) {
      outgoingReadings.paper = false;
      outgoingReadings.metal = false;
      outgoingReadings.glass = false;
      outgoingReadings.plastic = true;
      outgoingReadings.bio = false;
    } else if (strcmp(bbox.label, "bio") == 0) {
      outgoingReadings.paper = false;
      outgoingReadings.metal = false;
      outgoingReadings.glass = false;
      outgoingReadings.plastic = false;
      outgoingReadings.bio = true;
    } else {
      // Unknown label, set all flags to false
      outgoingReadings.paper = false;
      outgoingReadings.metal = false;
      outgoingReadings.glass = false;
      outgoingReadings.plastic = false;
      outgoingReadings.bio = false;
    }
  });

  // Send the message if any of the labels are found
  if (outgoingReadings.paper || outgoingReadings.metal || outgoingReadings.glass ||
      outgoingReadings.plastic || outgoingReadings.bio) {
    sendMessage();
    // Reset all flags to avoid sending the same message repeatedly
    outgoingReadings.paper = false;
    outgoingReadings.metal = false;
    outgoingReadings.glass = false;
    outgoingReadings.plastic = false;
    outgoingReadings.bio = false;
  }
}