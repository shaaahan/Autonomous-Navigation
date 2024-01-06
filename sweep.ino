
void sweep() {

  if (pingOn == true) {

    sweepServo.attach(sweep_servo);

    for (pos = 60; pos <= 120; pos += 1)  // goes from 0 degrees to 180 degrees
    {                                     // in steps of 1 degree
      sweepServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                          // waits 15ms for the servo to reach the position
    }
    for (pos = 120; pos >= 60; pos -= 1)  // goes from 120 degrees to 60 degrees
    {
      sweepServo.write(pos);  // tell servo to go to position in variable 'pos'
      delay(15);              // waits 15ms for the servo to reach the position
    }

    sweepServo.write(90);  // tell servo to go to position in variable 'pos'
    delay(15);             // waits 15ms for the servo to reach the position
  }

  sweepServo.detach();
}
