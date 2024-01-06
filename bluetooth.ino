
void bluetooth() {
  while (SerialBT.available()) {
    {
      str = SerialBT.readStringUntil('\n');  // str is the temporary variable for storing the last string sent over bluetooth from Android device
    }

    blueData = (str.toInt());  //  convert the string 'str' into an integer and assign it to blueData
    Serial.print("BlueTooth Value ");
    Serial.println(blueData);

    // **************************************************************************************************************************************************

    switch (blueData) {
      case 1:
        SerialBT.println("Forward ");
        Forward();
        break;

      case 2:
        SerialBT.println("Reverse ");
        Reverse();
        break;

      case 3:
        SerialBT.println("Left ");
        LeftTurn();
        StopCar();
        break;

      case 4:
        SerialBT.println("Right ");
        RightTurn();
        StopCar();
        break;

      case 5:
        SerialBT.println("Stop Car ");
        StopCar();
        break;

      case 6:
        setWaypoint();
        break;

      case 7:
        goWaypoint();
        break;

      case 8:
        SerialBT.println("Turn Around ");
        turnAround();
        break;

      case 9:
        SerialBT.println("Compass Forward ");
        setHeading();
        Compass_Forward();
        break;

      case 10:
        setHeading();
        break;

      case 11:
        gpsInfo();
        break;

      case 12:
        SerialBT.println("Compass Turn Right ");
        CompassTurnRight();
        break;

      case 13:
        SerialBT.println("Compass Turn Left ");
        CompassTurnLeft();
        break;

      case 14:
        SerialBT.println("Calibrate Compass ");
        calibrateCompass();
        break;

      case 15:
        pingToggle();
        break;

      case 16:
        clearWaypoints();
        break;

      case 17:
        ac = 0;
        SerialBT.print("Waypoints Complete");
        break;

      case 18:
        dockToggle();
        break;

      case 19:
        setInitialwaypoint();
        break;

      case 20:
        returnToHome();
        break;
    }

    // **************************************************************************************************************************************************

    if (blueData) {
      if (blueData >= 1000) {
        SerialBT.print("Speed set To:  ");
        SerialBT.println(blueData - 1000);
        slowdutyCycle = (blueData - 1000);
        Serial.println();
        Serial.print("Turn Speed ");
        Serial.println(slowdutyCycle);
      }
    }
  }
  // **************************************************************************************************************************************************

  if (SerialBT.available() < 0) {
    SerialBT.println("No Bluetooth Data ");
  }
}