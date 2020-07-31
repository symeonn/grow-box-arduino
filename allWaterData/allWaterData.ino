#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_EC.h"
#include <EEPROM.h>

#define TEMP_SENSOR_ONE_WIRE_BUS 4

OneWire oneWire(TEMP_SENSOR_ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

const long LOOP_DELAY_MILIS = 5UL * 60UL * 1000UL; // 5m

//PH METER
#define SensorPin 0          //pH meter Analog output to Arduino Analog Input 0
float temp;

int phInitialRun = true;
int phMeterReading = false;
const int phInterruptPin = 2;
const int phLedPin =  8;
const int phRealyPin =  10;

//EC METER
#define EC_PIN A1
float voltage, ecValue, temperature = 25;
DFRobot_EC ec;

int ecInitialRun = true;
int ecMeterReading = false;
const int ecInterruptPin = 3;
const int ecLedPin =  9;
const int ecRealyPin =  11;

void setup() {
  // start serial port
  Serial.begin(9600);
  // Serial.println("Ready");    //Test the serial monitor

  // Start up the library
  tempSensor.begin();

  ec.begin();
  //pinMode(buttonPin, INPUT);
  pinMode(phInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(phInterruptPin), getPh, RISING );

  pinMode(ecInterruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ecInterruptPin), getEc, RISING );

  pinMode(phLedPin, OUTPUT);
  pinMode(ecLedPin, OUTPUT);

  pinMode(phRealyPin, OUTPUT);
  pinMode(ecRealyPin, OUTPUT);

  delay(3000);
  digitalWrite(phRealyPin, HIGH); //off
  digitalWrite(ecRealyPin, HIGH); //off

}

void loop() {

  float temperature = -1;
  float phValue = -1;
  float ecValue = -1;

  temperature = getTemperatureReadings();

  Serial.print(temperature);
  Serial.print(",");
  Serial.print(phValue);
  Serial.print(",");
  Serial.print(ecValue);
  Serial.println("");

  delay(LOOP_DELAY_MILIS); 
  
}

void getPh() {
  //Serial.println("interupt");
  //Serial.println(phInitialRun);

  if (phInitialRun == true) {
    phInitialRun = false;
  } else if (!phMeterReading) {
    phMeterReading = true;
    //Serial.println("reading");

    digitalWrite(phLedPin, HIGH);
    digitalWrite(phRealyPin, LOW); //on
    delay(2000);

    float temperature = -1;
    float phValue = -1;
    float ecValue = -1;

    temperature = getTemperatureReadings();
    phValue = getPHreadings();
    // ecValue = getECreadings();

    blinkLED(phLedPin);

    Serial.print(temperature);
    Serial.print(",");
    Serial.print(phValue);
    Serial.print(",");
    Serial.print(ecValue);
    Serial.println("");

    delay(100);
    digitalWrite(phLedPin, LOW);
    phMeterReading = false;

    delay(500);
    digitalWrite(phRealyPin, HIGH); //off
  }
}

void getEc() {
  //Serial.println("interupt");
  //Serial.println(phInitialRun);

  if (ecInitialRun == true) {
    ecInitialRun = false;
  } else if (!ecMeterReading) {
    ecMeterReading = true;
    //Serial.println("reading");

    digitalWrite(ecLedPin, HIGH);
    digitalWrite(ecRealyPin, LOW); //on
    delay(2000);

    float temperature = -1;
    float phValue = -1;
    float ecValue = -1;

    temperature = getTemperatureReadings();
    // phValue = getPHreadings();
    ecValue = getECreadings();

    blinkLED(ecLedPin);

    Serial.print(temperature);
    Serial.print(",");
    Serial.print(phValue);
    Serial.print(",");
    Serial.print(ecValue);
    Serial.println("");

    delay(100);
    digitalWrite(ecLedPin, LOW);
    ecMeterReading = false;

    delay(500);
    digitalWrite(ecRealyPin, HIGH); //off
  }
}

void blinkLED(int ledPin) {
  digitalWrite(ledPin, LOW);
  delay(500);
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
  digitalWrite(ledPin, HIGH);
  delay(500);
}

float getTemperatureReadings() {
  tempSensor.requestTemperatures();
  return tempSensor.getTempCByIndex(0);
}

float getPHreadings() {
  float phbuf[10];

  for (int i = 0; i < 10; i++) { //Get 10 sample value from the sensor for smooth the value
    phbuf[i] = analogRead(SensorPin);
    delay(100);
  }

  for (int i = 0; i < 9; i++) { //sort the analog from small to large
    for (int j = i + 1; j < 10; j++) {
      if (phbuf[i] > phbuf[j]) {
        temp = phbuf[i];
        phbuf[i] = phbuf[j];
        phbuf[j] = temp;
      }
    }
  }
  unsigned long int sum = 0;  //Store the average value of the sensor feedback

  for (int i = 2; i < 8; i++)               //take the average value of 6 center sample
    sum += phbuf[i];

  float phValue = (float)sum * 5.0 / 1024 / 6; //convert the analog into millivolt
  phValue = 3.5 * phValue;                  //convert the millivolt into pH value

  //Serial.print("    pH:");
  //Serial.print(phValue, 2);
  //Serial.println(" ");

  return phValue;
}

float getECreadings() {

  float ecbuf[10];

  for (int i = 0; i < 10; i++) { //Get 10 sample value from the sensor for smooth the value

    voltage = analogRead(EC_PIN) / 1024.0 * 5000; // read the voltage
    temperature = getTemperatureReadings();  // read your temperature sensor to execute temperature compensation
    ecbuf[i] =  ec.readEC(voltage, temperature); // convert voltage to EC with temperature compensation

    //Serial.print("single read: ");
    //Serial.println(ecbuf[i]);

    delay(1000);
    ec.calibration(voltage, temperature); // calibration process by Serail CMD

  }

  for (int i = 0; i < 9; i++) { //sort the analog from small to large
    for (int j = i + 1; j < 10; j++) {
      if (ecbuf[i] > ecbuf[j]) {
        temp = ecbuf[i];
        ecbuf[i] = ecbuf[j];
        ecbuf[j] = temp;
      }
    }
  }

  float sum = 0;  //Store the average value of the sensor feedback

  for (int i = 2; i < 8; i++) {              //take the average value of 6 center sample
    //Serial.println(ecbuf[i]);
    sum += ecbuf[i];
  }

  //Serial.print("sum: ");
  //Serial.println(sum);

  float avgValue = (float)sum / 6;

  //Serial.print("avg: ");
  //Serial.println(avgValue);

  return avgValue;
}
