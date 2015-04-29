#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

#define USE_OCTOWS2811
#include<OctoWS2811.h>
#include<FastLED.h>

#include <OneWire.h>
#include <DallasTemperature.h>

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

boolean wasBumped = false;
unsigned long bumpedAt;
#define BUMP_DURATION 3000
#define BUMP_THRESHOLD 10

/* LEDS */
#define NUM_LEDS_PER_STRIP 28
#define NUM_STRIPS 6

#define kMatrixWidth  28
#define kMatrixHeight 6
#define kMatrixSerpentineLayout  false

#define DEFAULT_BRIGHTNESS 128
#define MAX_BRIGHTNESS 255


CRGB leds[NUM_STRIPS * NUM_LEDS_PER_STRIP];


// x,y, & time values
uint32_t x,y,v_time,hue_time,hxy;

// Play with the values of the variables below and see what kinds of effects they
// have!  More octaves will make things slower.

#define DEFAULT_OCTAVES 1
#define DEFAULT_HUE_OCTAVES 2
#define DEFAULT_XSCALE 57771
#define DEFAULT_YSCALE 57771
#define DEFAULT_HUE_SCALE 1
#define DEFAULT_TIME_SPEED 1111
#define DEFAULT_HUE_SPEED 31
#define DEFAULT_X_SPEED 331
#define DEFAULT_Y_SPEED 1111



// how many octaves to use for the brightness and hue functions
uint8_t octaves=1;
uint8_t hue_octaves=2;

// the 'distance' between points on the x and y axis
int xscale=57771;
int yscale=57771;

// the 'distance' between x/y points for the hue noise
int hue_scale=1;

// how fast we move through time & hue noise
int time_speed=1111;
int hue_speed=0;

// adjust these values to move along the x or y axis between frames
int x_speed=331;
int y_speed=1111;



/* TEMP Sensors */
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);

/* Timers */
IntervalTimer sensorTimer;
volatile boolean shouldReadSensors = true;

void displaySensorDetails(void)
{
  sensor_t sensor;

  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));


  delay(500);
}

void setup(void)
{
  // initialize the x/y and time values
  random16_set_seed(8934);
  random16_add_entropy(analogRead(3));
  hxy = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
  x = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
  y = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
  v_time = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();
  hue_time = (uint32_t)((uint32_t)random16() << 16) + (uint32_t)random16();

  delay(500);
  
  /* Initialise the sensors */
   accel.begin();
   gyro.begin();

  sensorTimer.begin(scheduleSensorRead, 1000);

  LEDS.addLeds<OCTOWS2811>(leds, NUM_LEDS_PER_STRIP);
  LEDS.setBrightness(DEFAULT_BRIGHTNESS);
}

void loop(void)
{
  if (shouldReadSensors) {
    readSensors();
  }

  noise();


}

void readSensors(void)
{
  /* Get a new sensor event */
  sensors_event_t event;

  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  wasBumped = event.acceleration.x > BUMP_THRESHOLD || event.acceleration.y > BUMP_THRESHOLD || event.acceleration.z > BUMP_THRESHOLD;
  
  if (wasBumped) {
    bumpedAt = millis();
    //Serial.print("Bumped: "); Serial.println(bumpedAt);
  }
  
//  Serial.print(F("ACCEL "));
//  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
//  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
//  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  "); Serial.println("m/s^2 ");

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  //  Serial.print(F("GYRO  "));
  //  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  //  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  //  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");

  //Serial.println(F(""));
  noInterrupts()
  shouldReadSensors = false;
  interrupts();
}


// Interrupt Handler to schedule sensor reads
void scheduleSensorRead(void)
{
  if (!shouldReadSensors) {
    shouldReadSensors = true;
  }
}

void noise(void) {
  LEDS.setBrightness(getBrightness());
  
    // fill the led array 2/16-bit noise values
  fill_2dnoise16(LEDS.leds(), kMatrixWidth, kMatrixHeight, kMatrixSerpentineLayout,
                octaves,x,xscale,y,yscale,v_time,
                hue_octaves,hxy,hue_scale,hxy,hue_scale,hue_time, true);

  LEDS.show();

  // adjust the intra-frame time values
  x += x_speed;
  y += y_speed;
  v_time += time_speed;
  hue_time += hue_speed;
}

void synapse(void)
{
  int ledNums = random8(56);
  for (int i = 0; i < ledNums; i++) {
    int aLed = random8(NUM_STRIPS * NUM_LEDS_PER_STRIP);
    leds[aLed] = CRGB::Gray;
  }
  LEDS.show();
  LEDS.delay(100);
  LEDS.clear(true);
  LEDS.show();

}
void hueRainbow(void)
{
  LEDS.setBrightness(getBrightness());
  
  static uint8_t hue = 0;
  for (int i = 0; i < NUM_STRIPS; i++) {
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      leds[(i * NUM_LEDS_PER_STRIP) + j] = CHSV((32 * i) + hue + j, 192, 255);
    }
  }

  // Set the first n leds on each strip to show which strip it is
//  for (int i = 0; i < NUM_STRIPS; i++) {
//    for (int j = 0; j <= i; j++) {
//      leds[(i * NUM_LEDS_PER_STRIP) + j] = CRGB::Red;
//    }
//  }

  hue++;

  LEDS.show();
  LEDS.delay(10);
}

int getBrightness(void) {
  unsigned long elapsedTime = (millis() - bumpedAt);
  int brightness = DEFAULT_BRIGHTNESS;
  
  if (bumpedAt > 0 && elapsedTime < BUMP_DURATION) {
    double percentage = 1.0 - (double(elapsedTime) / double(BUMP_DURATION));
    //Serial.println(percentage);
    brightness = DEFAULT_BRIGHTNESS + (percentage * (MAX_BRIGHTNESS - DEFAULT_BRIGHTNESS));
    hue_scale = 5;
    time_speed = 5555;
  } else {
    hue_scale = 1;
    time_speed = 1111;
    bumpedAt = 0;
  }
  
  //Serial.print("BumpedAt: "); Serial.print(brightness); Serial.print(" "); Serial.println(elapsedTime);
  
  return brightness;
}

