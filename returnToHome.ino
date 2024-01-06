void returnToHome() {

  SerialBT.println("Returning to Home");

  while (true) {
    bluetooth();
    if (blueData == 5) {
      StopCar();
      SerialBT.println("Return to Home Aborted");
      break;
    }

    getCompass();
    getGPS();

    if (millis() > 5000 && gps.charsProcessed() < 10) {
      SerialBT.println(F("No GPS data: check wiring"));
      GPSmalfunctionBuzzer();
      break;
    }

    Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), Home_LATarray[0], Home_LONarray[0]);
    GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), Home_LATarray[0], Home_LONarray[0]);

    if (Distance_To_Home == 0) {
      StopCar();
      SerialBT.println("Returned to Home!");
      break;
    }

    if (abs(GPS_Course - compass_heading) <= 15) {
      Forward();
    } else {
      int x = (GPS_Course - 360);
      int y = (compass_heading - (x));
      int z = (y - 360);

      if ((z <= 180) && (z >= 0)) {
        SlowLeftTurn();
      } else {
        SlowRightTurn();
      }
    }
  }
}
