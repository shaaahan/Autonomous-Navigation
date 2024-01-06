
void Startup() {

  sweepServo.detach();

  esp_task_wdt_init(150, true);
  esp_task_wdt_add(NULL);

  MQTT_connect();

  for (int i = 5; i >= 1; i--)  // Count down for X seconds
  {
    Serial1.print("Pause for Startup... ");
    Serial1.print(i);
    delay(1000);
  }

  SerialBT.println("Searching for Satellites  ");
  Serial.println("Searching for Satellites ");

  while (Number_of_SATS <= 4)  // Wait until x number of satellites are acquired before starting main loop
  {
    getGPS();                                        // Update gps data
    Number_of_SATS = (int)(gps.satellites.value());  // Query Tiny GPS for the number of Satellites Acquired
    bluetooth();                                     // Check to see if there are any bluetooth commands being received
  }

  setWaypoint();  // set intial waypoint to current location
  wpCount = 0;    // zero waypoint counter
  ac = 0;         // zero array counter

  Serial.println(Number_of_SATS);
  Serial.println(" Satellites Acquired");
  SerialBT.println("Satellites Acquired");
  gpsBuzzer();

  test_azimuth();
  delay(1000);
  test_elevation();
  delay(1000);

  startupBuzzer();
}