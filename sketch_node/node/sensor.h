/**

  Sensor, Model Joy-IT® Ultraschall Abstandssensor
    Artikel Nr. SEN-US01
    Betriebsspannung 5V
    Betriebsstrom 15mA
  
  - wenn frei / unbelegt   = ON
  - wenn besetzt / belegt  = OFF

  return
  - sensorState = HIGH|LOW
  - sensorValue = 0 - xxxx

**/


// Generic
bool sensorState = LOW;
int sensorValue = 4095;

// Ultaschall-Sensor
const int Trigger_AusgangsPin = 0;
const int Echo_EingangsPin = 4;

// Benoetigte Variablen werden definiert
int maximumRange = 40;
int minimumRange = 2;
long Abstand;
long Dauer;


void initSensorUS() {
  pinMode(Trigger_AusgangsPin, OUTPUT);
  pinMode(Echo_EingangsPin, INPUT);
}


void getSensorUSValue() {
  // Abstandsmessung wird mittels des 10us langen Triggersignals gestartet
  digitalWrite(Trigger_AusgangsPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger_AusgangsPin, LOW); 

  // Nun wird am Echo-Eingang gewartet, bis das Signal aktiviert wurde
  // und danach die Zeit gemessen, wie lang es aktiviert bleibt
  Dauer = pulseIn(Echo_EingangsPin, HIGH);

  // Nun wird der Abstand mittels der aufgenommenen Zeit berechnet
  Abstand = Dauer/58.2;
  // Überprüfung ob gemessener Wert innerhalb der zulässingen Entfernung liegt
  if (Abstand >= maximumRange || Abstand <= minimumRange) {
    // Falls nicht wird eine Fehlermeldung ausgegeben.
    Serial.println("US-Sensor, Abstand ausserhalb des Messbereichs");
    //Serial.println("-----------------------------------");
  }
  else {
    // Der berechnete Abstand wird in der seriellen Ausgabe ausgegeben
    Serial.print("US-Sensor, Der Abstand betraegt: ");
    Serial.print(Abstand);
    Serial.println(" cm");
    //Serial.println("-----------------------------------");  
  }

  sensorValue = Abstand;
  Serial.print("US-Sensor, Analog IO - Sensor value: ");
  Serial.print(sensorValue);
  if ( sensorValue > 5 ) {
    sensorState = HIGH;
    Serial.print(" - Sensor state: ");
  } 
  else {
    sensorState = LOW;
    Serial.print(" - Sensor state: ");
  }
  Serial.println(sensorState);

}





// Case Press-Button
const int sensorPin = 15;    // 10K PULLDOWN

// digital IO of a button "on=nicht belegt, off=belegt"
void getButtonState() {
  sensorValue = digitalRead(sensorPin);
  Serial.print("Digital IO, Sensor value: ");
  Serial.print(sensorValue);
  // check if the pushbutton is pressed, if it is, the buttonState is HIGH
  if (sensorValue == HIGH) {
    sensorState = HIGH;
    Serial.print(" - Sensor state: ");
  } else {
    sensorState = LOW;
    Serial.print(" - Sensor state: ");
  }
  Serial.println(sensorState);
}

