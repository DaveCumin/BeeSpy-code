/*
  BASED ON AnalogReadSerial example file
  Prints out the ESF data
*/
// the setup routine runs once when you press reset:
void setup()
{
    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop()
{
    // read the input on analog pin 0:
    int sensorValue0 = analogRead(A0);
    Serial.print(sensorValue0);
    Serial.print(" ");
    int sensorValue1 = analogRead(A1);
    Serial.print(sensorValue1);
    Serial.print(" ");
    int sensorValue2 = analogRead(A2);
    Serial.print(sensorValue2);
    Serial.print(" ");
    int sensorValue3 = analogRead(A3);
    Serial.print(sensorValue3);
    Serial.print(" ");
    int sensorValue4 = analogRead(A4);
    Serial.print(sensorValue4);
    Serial.print(" ");
    int sensorValue5 = analogRead(A5);
    Serial.println(sensorValue5);
    delay(1); // delay in between reads for stability
}
