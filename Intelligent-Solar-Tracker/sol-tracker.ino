void solarTracker() {
  // Update time from NTP server

  if (dockOn == true) {
    timeClient.update();

    if (timeClient.getMinutes() % 10 == 0) {
      Serial.println("Time Synchronized");
      SerialBT.println("Time Synchronized");
      indicatorBuzzer();
    }

    time_t local = timeClient.getEpochTime();
    time_t utc = local - utcOffsetInSeconds;

    tmElements_t localTime;
    breakTime(local, localTime);

    Serial.print("Local date and time: ");
    Serial.print(localTime.Year + 1970);  // Adjust for the epoch
    Serial.print("-");
    Serial.print(localTime.Month);
    Serial.print("-");
    Serial.print(localTime.Day);
    Serial.print(" ");
    Serial.print(localTime.Hour);
    Serial.print(":");
    Serial.print(localTime.Minute);
    Serial.print(":");
    Serial.println(localTime.Second);

    // Calculate the solar position, in degrees
    calcHorizontalCoordinates(utc, latitude, longitude, azimuth, elevation);
    Serial.println("Az: " + String(azimuth) + "  El: " + String(elevation));

    // Adjust Servo
    aziServo.write(map(azimuth, 90, 270, 180, 0));
    if (elevation < 0) elevation = 0;
    eleServo.write(90 - elevation);
  }
}

bool syncTime() {
  int attempts = 0;
  while (!timeClient.update()) {
    Serial.print(".");
    timeClient.forceUpdate();
    delay(100);

    attempts++;
    if (attempts > 3) {
      Serial.println();
      return false;
    }
  }
  Serial.println();
  return true;
}
void test_azimuth() {
  aziServo.write(90);  // Centre position
  delay(500);
  aziServo.write(60);  // Centre position
  delay(500);
  aziServo.write(120);  // Centre position
  delay(500);
  aziServo.write(90);  // Centre position
  delay(500);
}

void test_elevation() {
  for (int a = 5; a < 90; a = a + 2) {
    eleServo.write(a);  // Centre position
    delay(30);
  }
  eleServo.write(90);  // Centre position
  delay(1000);
}

void dockToggle() {
  if (dockOn == true) {
    dockOn = false;
    indicatorBuzzer();
    Serial.print("Initiating Un-Docking Procedures");
    SerialBT.print("Initiating Un-Docking Procedures");
  } else if (dockOn == false) {
    dockOn = true;
    indicatorBuzzer();
    Serial.print("Initiating Docking Procedures");
    SerialBT.print("Initiating Docking Procedures");
  }
}