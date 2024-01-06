
void goWaypoint() {
  SerialBT.println("Go to Waypoint");

  while (true) {                   // Start of Go_Home procedure
    bluetooth();                   // Run the Bluetooth procedure to see if there is any data being sent via BT
    if (blueData == 5) { break; }  // If a 'Stop' Bluetooth command is received then break from the Loop


    getCompass();  // Update Compass heading
    getGPS();      // Tiny GPS function that retrieves GPS data - update GPS location and delay time changed from 100 to 10

    if (millis() > 5000 && gps.charsProcessed() < 10)  // If no Data from GPS within 5 seconds then send error
      SerialBT.println(F("No GPS data: check wiring"));
    GPSmalfunctionBuzzer();

    Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination
    GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), Home_LATarray[ac], Home_LONarray[ac]);               //Query Tiny GPS for Course to Destination

    if (Distance_To_Home == 0)  // If the Vehicle has reached it's Destination, then Stop
    {
      StopCar();                              // Stop the robot after each waypoint is reached
      SerialBT.println("Mission Complete!");  // Print to Bluetooth device - "You have arrived"
      ac++;                                   // increment counter for next waypoint
      break;                                  // Break from Go_Home procedure and send control back to the Void Loop
                                              // go to next waypoint
    }


    if (abs(GPS_Course - compass_heading) <= 15)  // If GPS Course and the Compass Heading are within x degrees of each other then go Forward
                                                  // otherwise find the shortest turn radius and turn left or right
    {
      Forward();
    } else {
      int x = (GPS_Course - 360);       // x = the GPS desired heading - 360
      int y = (compass_heading - (x));  // y = the Compass heading - x
      int z = (y - 360);                // z = y - 360

      if ((z <= 180) && (z >= 0))  // if z is less than 180 and not a negative value then turn left otherwise turn right
      {
        SlowLeftTurn();
      } else {
        SlowRightTurn();
      }
    }
  }
  delay(2500);
}
