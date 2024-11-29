#include <Arduino.h>
#include <PID_v1.h>

// Pins
static constexpr auto ADC_PIN = 13;
static constexpr auto PWM_PIN = 32; 
static constexpr auto IN1_PIN = 21;
static constexpr auto IN2_PIN = 25;
// Variables
static double Vref = 1.5f;
static double Vin = 0;

static double Dref = 1500;
static double Din = 0;
static double filteredDin = 0;

static double Vo = 0;
static double filteredVin = 0;
static double alpha = 0.3;

enum Direction {
  FORWARD,
  BACKWARD
};

static Direction direction = Direction::FORWARD;

void controller();
void planta();
// Ku = 0.14     Tu = 1.8s
static double Kp = 0.08, Ki = 0.01, Kd = 0.025;
// static double Kp = 0.462, Kd = 0.16, Ki = 0.4;
PID controlador(&Din, &Vo, &Dref, Kp, Ki, Kd, DIRECT);

void setup() {
  // Configuración inicial
  Serial.begin(9600);
  pinMode(ADC_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  controlador.SetOutputLimits(-100, 100);
  controlador.SetMode(AUTOMATIC);
}

void loop() {
  delay(100);
  controller();
  planta();
}

void controller()
{
  Din = analogRead(ADC_PIN);
  Serial.print(Din);
  Serial.print(" \nVo: ");
  Serial.print(Vo);
  Serial.print("\n");

  controlador.Compute();
}

void planta()
{
  if (Vo > 0)
  {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  }
  else 
  {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }
  auto modVo = fabs(Vo);
  analogWrite(PWM_PIN, modVo*2);
}