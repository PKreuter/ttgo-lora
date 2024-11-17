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
const int Echo_EingangsPin = 2;                // Pos 4 / bn
const int Trigger_AusgangsPin = 4;             // Pos 2 / ws/bn

// PressButton and IR Sensor PIN
const int sensorPin = 4;    // 10K PULLDOWN


// Benoetigte Variablen werden definiert
int maximumRange = 100;
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
  debugOutput("US-Sensor, Abstand: " +String(Abstand), 5);
  // Überprüfung ob gemessener Wert innerhalb der zulässingen Entfernung liegt
  if (Abstand >= maximumRange || Abstand <= minimumRange) {
    // Falls nicht wird eine Fehlermeldung ausgegeben.
    debugOutput("US-Sensor, Abstand ausserhalb des Messbereichs", 2);
  }
  else {
    // Der berechnete Abstand wird in der seriellen Ausgabe ausgegeben
    debugOutput("US-Sensor, Der Abstand betraegt: " +String(Abstand)+ " cm", 5);
  }

  sensorValue = Abstand;
  if ( sensorValue > 1 and sensorValue < 8) {
    sensorState = HIGH;
    debugOutput("US-Sensor, Analog IO - Sensor value: " +String(sensorValue)+ " - Sensor state: " +String(sensorState), 4);
  } 
  else {
    sensorState = LOW;
    debugOutput("US-Sensor, Analog IO - Sensor value: " +String(sensorValue)+ " - Sensor state: " +String(sensorState), 4);
  }

}



// Case Press-Button

// digital IO of a button "on=nicht belegt, off=belegt"
void getButtonState() {
  sensorValue = digitalRead(sensorPin);
  // check if the pushbutton is pressed, if it is, the buttonState is HIGH
  if (sensorValue == HIGH) {
    sensorState = HIGH;
    debugOutput("Digital IO, Sensor value: " +String(sensorValue)+ " - Sensor state: " +String(sensorState), 4);
  } else {
    sensorState = LOW;
    debugOutput("Digital IO, Sensor value: " +String(sensorValue)+ " - Sensor state: " +String(sensorState), 4); 
  }
}

// Case IR Sensor
/**
When something passes between the two, and it’s not transparent to IR, then
the“beam is broken“and the receiver will output a signal.
**/

int a,b,c;

// digital IO of a button "on=nicht belegt, off=belegt"
void getSensorIRState() {
 
  delayMicroseconds(500);  //wait
  a = analogRead(sensorPin);
  debugOutput("Analog noise+signal: " +String(a),5);

  delayMicroseconds(500);  //wait
  b = analogRead(sensorPin);
  debugOutput("Analog noise: " +String(b),5);

  delayMicroseconds(500);  //wait
  c = analogRead(sensorPin);
  debugOutput("Analog noise: " +String(b),5);

/
  debugOutput("difference:[ (noise+signal)-(noise)] just signal: " +String(a-b),5);

  int x = (a+b+c)/3;
  debugOutput("New Value :" +String(x),4);

 
  sensorValue = (a+b+c)/3;  

  if (sensorValue < 2000) {
    sensorState = HIGH;
    debugOutput("Analog IO, Sensor value: " +String(sensorValue)+ " - Sensor state: " +String(sensorState), 4);
  } else {
    sensorState = LOW;
    debugOutput("Analog IO, Sensor value: " +String(sensorValue)+ " - Sensor state: " +String(sensorState), 4); 
  }
}

