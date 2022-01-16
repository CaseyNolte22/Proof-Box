#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <AutoPID.h>
#include <RBDdimmer.h>

// So the controller starts sooner
unsigned long updateTimer = 1900;

// Define all pins here
#define BUILTIN_LED D0
#define POT_PIN A0
#define RED_PIN D6
#define GREEN_PIN D4
#define OUTPUT_PIN D5
#define ZC_PIN D7
#define DHTPIN D3  
#define DHTTYPE DHT22

// PID Tuning
#define OUTPUT_MIN 0
#define OUTPUT_MAX 100
#define KP 10
#define KI 0
#define KD 0

// Creating neccesary objects
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
dimmerLamp dimmer(OUTPUT_PIN, ZC_PIN);
double temperature, humidity, setPoint, outputVal;
AutoPID myPID(&temperature, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// map but supports floating point
float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  Serial.begin(9600);

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(OUTPUT_PIN, OUTPUT);

  dht.begin();       // Begin DHT Reader

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); 
  lcd.print("Initializing!");

  Serial.println("Dimmer Program is starting...");
  dimmer.begin(NORMAL_MODE, ON);
  Serial.println("Dimmer Program started");

  myPID.setBangBang(10); // 10 degrees from setpoint absolute will set to 100% or 0%
  myPID.setTimeStep(2000); // PID updates every 2 seconds
}

void loop() {

  // Getting setpoint
  int analogValue = analogRead(POT_PIN);
  setPoint = floatMap(analogValue, 0, 1023, 60, 90);

  // Read temperature every 2 seconds. Sends data over serial for plotting and tuning
  if (updateTimer > 2000) {
    humidity = dht.readHumidity();
    temperature = dht.readTemperature(true);

    Serial.print(temperature);
    Serial.print(" ");
    Serial.print(setPoint);
    Serial.print(" ");
    Serial.print(humidity);
    Serial.print(" ");
    Serial.println(outputVal);

    updateTimer = 0;

    // In case of error
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      lcd.clear();
      lcd.print("Sensor Fail");
      return;
    } else {
    }
  }

  // Run the PID, then set dimmer to output correct power
  myPID.run();
  dimmer.setPower(outputVal);
  double tempDiff = abs(temperature - setPoint);

  // Status LED, shows green and shades of yellow to red depending on nearness to setpoint
  if (tempDiff > 4) {
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(GREEN_PIN, LOW);
  } else if (tempDiff < 1) {
    digitalWrite(GREEN_PIN, HIGH);
    digitalWrite(RED_PIN, LOW);
  } else {
    analogWrite(RED_PIN, 50*tempDiff);
    analogWrite(GREEN_PIN, 256-(50*tempDiff));
  }
 

  // LCD shows setpoint, temperature, and current power
  lcd.setCursor(0, 0);
  lcd.print("Tmp: " + String(temperature, 1) + "*F " + String(outputVal, 0) + "% ");

  lcd.setCursor(0, 1);
  lcd.print("StP: " + String(setPoint, 1) + "*F      ");

  // Increment time by .1 seconds
  delay(100);
  updateTimer += 100;
}

