
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Data Delivery Success";
  } else {
    success = "Data Delivery Fail";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  SerialBT.print("Bytes received: ");
  SerialBT.println(len);
  indicatorBuzzer();
  espcamPaper = incomingReadings.paper;
  espcamMetal = incomingReadings.metal;
  espcamGlass = incomingReadings.glass;
  espcamPlastic = incomingReadings.plastic;
  espcamBio = incomingReadings.bio;

}

void registerPeer() {
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
}

void addPeer() {
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
}

void sendMessage() {

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&outgoingReadings, sizeof(outgoingReadings));

  if (result == ESP_OK) {
    Serial.println("Sent data with success");
    SerialBT.println("Sent data with success");
    indicatorBuzzer();
  } else {
    Serial.println("Error sending the data");
    SerialBT.println("Error sending the data");
    malfunctionBuzzer();
  }
}

void wasteSorting() {

  if (!espcamPlastic && !espcamGlass && !espcamMetal && !espcamBio && !espcamPaper) {
    desiredPos = 90;   // No waste detected, keep the servo at normal position
  } else if (espcamPlastic || espcamGlass || espcamMetal) {
    desiredPos = 180;  // Non-biodegradable waste detected, turn the servo to 180 degrees
  } else if (espcamBio || espcamPaper) {
    desiredPos = 0;    // Biodegradable waste detected, turn the servo to 0 degrees
  }

  // Check if the servo needs to move to the desired position
  if (desiredPos != currentPos) {
    wasteServo.write(desiredPos);  // Move the servo to the desired position
    currentPos = desiredPos;       // Update the current servo position
  }
}