#include <Arduino.h>
#include <PID_v1.h>

// pins

static constexpr auto ADC_PIN = 13;
static constexpr auto PWM_PIN = 32;

// variables

static double maxSpeed = 0.8;
static double dutyCycle = 0.5;
static double sensorReading = 0;

enum Direction{
  FORWARD,
  BACKWARD
  };

static Direction direction = Direction::FORWARD;
static double Vin = 0;
double controller(double);

void setup() {
  // put your setup code here, to run once:
  pinMode(ADC_PIN, ANALOG);
  pinMode(PWM_PIN, ANALOG);
  analogReadResolution(10);
}

void loop() {
  Vin = analogReadMilliVolts(ADC_PIN);
  Serial.println("Tension ADC:");
  Serial.print(Vin);
}