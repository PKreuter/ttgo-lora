/**



return
- sensorState = HIGH|LOW
- sensorValue = 0 - 4095


**/


const int sensorPin = 4;

/** return HIGH or LOW
    HIGH = IR OK
    LOW  = IR nok, or broken
**/


int sensorState = LOW;
int sensorValue = 4095;

// analog IO of the IR Sensor DFRobot SEN0523
void getSensorValue() {
  sensorValue = analogRead(sensorPin);
  Serial.print("Sensor value: ");
  Serial.print(sensorValue);
  //if (sensorValue < 3000) {
  if (sensorValue < 1000) {
    sensorState = LOW;
    Serial.print(" - Sensor state: ");
  } else {
    sensorState = HIGH;
  }
  Serial.println(sensorState);
}




// digital IO of a button
void getButtonState() {
  sensorValue = digitalRead(sensorPin);
  Serial.print("Sensor value: ");
  Serial.print(sensorValue);
  // check if the pushbutton is pressed, if it is, the buttonState is HIGH
  if (sensorValue == HIGH) {
    sensorState = LOW;
    Serial.print(" - Sensor state: ");
  } else {
    sensorState = HIGH;
  }
  Serial.println(sensorState);
}

