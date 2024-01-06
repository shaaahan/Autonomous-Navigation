
void calibrateCompass() {
  Serial.println("Calibrating... Move the magnetometer around in all directions.");
  SerialBT.println("Calibrating... Move the magnetometer around in all directions.");

  Serial.println("Press any key to stop calibration.");
  SerialBT.println("Press any key to stop calibration.");

  while (!SerialBT.available()) {

    sensors_event_t event;
    mag.getEvent(&event);

    // Determine Min / Max values
    if (event.magnetic.x < minX) minX = event.magnetic.x;
    if (event.magnetic.x > maxX) maxX = event.magnetic.x;
    if (event.magnetic.y < minY) minY = event.magnetic.y;
    if (event.magnetic.y > maxY) maxY = event.magnetic.y;
    if (event.magnetic.z < minZ) minZ = event.magnetic.z;
    if (event.magnetic.z > maxZ) maxZ = event.magnetic.z;

    delay(50);  // Adjust this delay according to your needs
  }

  // Calculate offsets
  offX = (maxX + minX) / 2;
  offY = (maxY + minY) / 2;
  offZ = (maxZ + minZ) / 2;

  Serial.println("Calibration complete!");
  Serial.print("Offset X: ");
  Serial.println(offX);
  Serial.print("Offset Y: ");
  Serial.println(offY);
  Serial.print("Offset Z: ");
  Serial.println(offZ);

  SerialBT.println("Calibration complete!");
  SerialBT.print("Offset X: ");
  SerialBT.println(offX);
  SerialBT.print("Offset Y: ");
  SerialBT.println(offY);
  SerialBT.print("Offset Z: ");
  SerialBT.println(offZ);

  indicatorBuzzer();

  offsetX = offX;
  offsetY = offY;
  offsetZ = offZ;
}

// *************************************************************************************************************************************************

void displaySensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

// *************************************************************************************************************************************************

void getGPS() {
  while (Serial2.available() > 0)
    gps.encode(Serial2.read());
}

// *************************************************************************************************************************************************

void setWaypoint() {

  if (wpCount >= 0) {
    SerialBT.print("GPS Waypoint ");
    SerialBT.print(wpCount + 1);
    SerialBT.print(" Set ");
    getGPS();      // get the latest GPS coordinates
    getCompass();  // update latest compass heading

    Home_LATarray[ac] = gps.location.lat();  // store waypoint in an array
    Home_LONarray[ac] = gps.location.lng();  // store waypoint in an array

    Serial.print("Waypoint #1: ");
    Serial.print(Home_LATarray[0], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[0], 6);
    Serial.print("Waypoint #2: ");
    Serial.print(Home_LATarray[1], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[1], 6);
    Serial.print("Waypoint #3: ");
    Serial.print(Home_LATarray[2], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[2], 6);
    Serial.print("Waypoint #4: ");
    Serial.print(Home_LATarray[3], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[3], 6);
    Serial.print("Waypoint #5: ");
    Serial.print(Home_LATarray[4], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[4], 6);
    Serial.print("Waypoint #6: ");
    Serial.print(Home_LATarray[5], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[5], 6);
    Serial.print("Waypoint #7: ");
    Serial.print(Home_LATarray[6], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[6], 6);
    Serial.print("Waypoint #8: ");
    Serial.print(Home_LATarray[7], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[7], 6);
    Serial.print("Waypoint #9: ");
    Serial.print(Home_LATarray[8], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[8], 6);
    Serial.print("Waypoint #10: ");
    Serial.print(Home_LATarray[9], 6);
    Serial.print(" ");
    Serial.println(Home_LONarray[9], 6);

    wpCount++;  // increment waypoint counter
    ac++;       // increment array counter

  } else {
    SerialBT.print("Waypoints Full");
  }
}


void setInitialwaypoint() {

  setWaypoint();  // set intial waypoint to current location
  wpCount = 0;    // zero waypoint counter
  ac = 0;         // zero array counter
}

// *************************************************************************************************************************************************

void clearWaypoints() {
  memset(Home_LATarray, 0, sizeof(Home_LATarray));  // clear the array
  memset(Home_LONarray, 0, sizeof(Home_LONarray));  // clear the array
  wpCount = 0;                                      // reset increment counter to 0
  ac = 0;

  SerialBT.print("GPS Waypoints Cleared");  // display waypoints cleared
}

// *************************************************************************************************************************************************

void getCompass() {

  sensors_event_t event;
  mag.getEvent(&event);

  // Apply offsets to the raw magnetometer readings
  float x = event.magnetic.x - offsetX;
  float y = event.magnetic.y - offsetY;

  // Calculate heading
  float heading = atan2(y, x);

  // Adjust for declination angle if needed
  float declinationAngle = (-0.0 + (19.0 / 60.0)) / (180 / PI);
  heading += declinationAngle;

  if (heading < 0)
    heading += 2 * PI;

  if (heading > 2 * PI)
    heading -= 2 * PI;

  float headingDegrees = heading * 180 / PI;
  compass_heading = static_cast<int>(headingDegrees);  // Convert to integer

  Serial.print("Heading (degrees): ");
  Serial.println(headingDegrees);
}

// *************************************************************************************************************************************************

void setHeading() {
  for (int i = 0; i <= 5; i++)  // Take several readings from the compass to insure accuracy
  {
    getCompass();  // get the current compass heading
  }

  desired_heading = compass_heading;  // set the desired heading to equal the current compass heading
  Heading_A = compass_heading;        // Set Heading A to current compass
  Heading_B = compass_heading + 180;  // Set Heading B to current compass heading + 180

  if (Heading_B >= 360)  // if the heading is greater than 360 then subtract 360 from it, heading must be between 0 and 360
  {
    Heading_B = Heading_B - 360;
  }

  SerialBT.print("Compass Heading Set: ");
  SerialBT.print(compass_heading);
  SerialBT.print(" Degrees");

  Serial.print("desired heading");
  Serial.println(desired_heading);
  Serial.print("compass heading");
  Serial.println(compass_heading);
}

// *************************************************************************************************************************************************

void gpsInfo() {
  Number_of_SATS = (int)(gps.satellites.value());                                                                                 //Query Tiny GPS for the number of Satellites Acquired
  Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination
  SerialBT.print("Lat:");
  SerialBT.print(gps.location.lat(), 6);
  SerialBT.print(" Lon:");
  SerialBT.print(gps.location.lng(), 6);
  SerialBT.print(" ");
  SerialBT.print(Number_of_SATS);
  SerialBT.print(" Sats ");
  SerialBT.print(Distance_To_Home);
  SerialBT.print("m");
  Serial.print("Distance to Waypoint ");
  Serial.println(Distance_To_Home);
}

void gpsPlot() {

  MQTT_connect();
  
  if (mqtt.connected()) {

    getGPS();

    float GPSlat = (gps.location.lat());   // variable to store latitude
    float GPSlng = (gps.location.lng());   // variable to store longitude
    float GPSalt = (gps.altitude.feet());  // variable to store altitude

    Distance_Travelled = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), HOME_LAT, HOME_LNG);  //Query Tiny GPS to Calculate Distance to Home

    char gpsbuffer[30];
    snprintf(gpsbuffer, sizeof(gpsbuffer),
             "%0.4f,%0.6f,%0.6f,%0.1f",
             Distance_Travelled, GPSlat, GPSlng, GPSalt);

    if ((GPSlng != 0) && (GPSlat != 0))  // If GPS longitude or latitude do not equal zero then Publish
    {
      GPS.publish(gpsbuffer);  // publish Combined Data to Adafruit IO
      Serial.println(gpsbuffer);
    }
  }
}
