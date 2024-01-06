
void ping() {
  currentMillis = millis();

  if ((currentMillis - previousMillis >= interval) && (pingOn == true)) {
    previousMillis = currentMillis;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);

    int inches = (duration / 2) / 74;  // convert the time into a distance
    Ping_distance == inches;

    if ((inches < 12) && (blueData != 5)) {
      crashBuzzer();
      SerialBT.print("Crash!  ");
      SerialBT.println(inches);
      Reverse();  // Quick reverse to Stop quickly
      delay(1000);
      StopCar();
      blueData = 5;  // Set bluetooth value to "Stop"
    }
  }
}

void pingToggle() {
  if (pingOn == true) {
    pingOn = false;
    indicatorBuzzer();
    Serial.print("Collision Avoidance OFF");
    SerialBT.print("Collision Avoidance OFF");
  } else if (pingOn == false) {
    pingOn = true;
    sweep();
    indicatorBuzzer();
    Serial.print("Collision Avoidance ON");
    SerialBT.print("Collision Avoidance ON");
  }
}
