/*
  AnalogReadSerial
  Reads 5 analog inputs on pins 0-4, prints the result to the serial monitor.
 */

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pins 0-4:
  int sensorValue = analogRead(A0);
  Serial.print(sensorValue);
  Serial.print(" "); 
  sensorValue = analogRead(A1);
  Serial.print(sensorValue);
  Serial.print(" "); 
  sensorValue = analogRead(A2);
  Serial.print(sensorValue);
  Serial.print(" "); 
  sensorValue = analogRead(A3);
  Serial.print(sensorValue);
  Serial.print(" "); 
  sensorValue = analogRead(A4);
  Serial.println(sensorValue);
  delay(50);        // delay in between reads for stability
}
