#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
//#include <Adafruit_BMP085_U.h>
//#include <Adafruit_L3GD20_U.h>
//#include <Adafruit_10DOF.h>

#define USE_OCTOWS2811
#include<OctoWS2811.h>
#include<FastLED.h>

#include <OneWire.h>
#include <DallasTemperature.h>

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
//Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

boolean wasBumped = false;
unsigned long bumpedAt;
#define BUMP_DURATION 3000
#define BUMP_THRESHOLD 10

/* LEDS */
#define NUM_LEDS_PER_STRIP 28
#define NUM_STRIPS 6
#define NUM_LEDS 168

#define DEFAULT_BRIGHTNESS 200
#define MAX_BRIGHTNESS 255

#define SENSOR_READ_TIMER 1000


CRGB leds[NUM_STRIPS * NUM_LEDS_PER_STRIP];

/* TEMP Sensors */
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);

/* Timers */
IntervalTimer sensorTimer;
volatile boolean shouldReadSensors = true;

void setup(void) {
  //Serial.begin(115200);
  //Serial.println(F("Starting")); Serial.println("");

  //  int ledPin = 13;
  //  pinMode(ledPin, OUTPUT);
  //  digitalWrite(ledPin, HIGH);

  // delay(500);

  /* Initialise the sensors */
  accel.begin();
  //gyro.begin();

  sensorTimer.begin(scheduleSensorRead, SENSOR_READ_TIMER);

  LEDS.addLeds<OCTOWS2811>(leds, NUM_LEDS_PER_STRIP);
  LEDS.setBrightness(DEFAULT_BRIGHTNESS);
}

void loop(void) {
  if (shouldReadSensors) {
    readSensors();
  }

  //Serial.println(F("Starting")); Serial.println("");
  synapse();
}

void readSensors(void) {
  /* Get a new sensor event */
  sensors_event_t event;

  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  wasBumped = event.acceleration.x > BUMP_THRESHOLD || event.acceleration.y > BUMP_THRESHOLD || event.acceleration.z > BUMP_THRESHOLD;

  if (wasBumped) {
    bumpedAt = millis();
    Serial.print("Bumped: "); Serial.println(bumpedAt);
  }
  noInterrupts()
  shouldReadSensors = false;
  interrupts();
}




void synapse() {
  double percentage = agitatedPercent();

  int leftRangeLow = 0;
  int leftRangeHigh = 83;
  int rightRangeLow = 84;
  int rightRangeHigh = 167;
  int delayTime = 100;

  int randomThresholdLeft = leftRangeHigh;
  int randomThresholdRight = rightRangeHigh;

  int brightness = DEFAULT_BRIGHTNESS;

  CRGB color = CRGB::White;

  if (percentage > 0) {
    randomThresholdLeft;
    randomThresholdRight;
    color = CRGB::Red;

    delayTime = 50.0 * (1 - percentage);
    brightness = (brightness * percentage);
  }

  randomizeLeds(leftRangeLow, randomThresholdLeft, color);
  randomizeLeds(rightRangeLow, randomThresholdRight, color);


  for (int j = 0; j < NUM_LEDS; j++) leds[j].fadeToBlackBy(random8(32, 128));
  LEDS.setBrightness(brightness);
  LEDS.show();
  delay(delayTime);
}

double agitatedPercent() {
  double percentage = 0;
  unsigned long elapsedTime = (millis() - bumpedAt);
  if (bumpedAt > 0 && elapsedTime < BUMP_DURATION) {
    percentage = 1.0 - (double(elapsedTime) / double(BUMP_DURATION));
  } else {
    bumpedAt = 0;
  }
  return percentage;
}

void randomizeLeds(int rangeLow, int rangeHigh, CRGB color) {
  int i = random8(rangeLow, rangeHigh);
  if (i < NUM_LEDS) leds[i] = color;
}

// Interrupt Handler to schedule sensor reads
void scheduleSensorRead(void)
{
  if (!shouldReadSensors) {
    shouldReadSensors = true;
  }
}


