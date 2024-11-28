#include <Arduino.h>
#include <PID_v1.h>

// Pins
static constexpr auto ADC_PIN = 13;  // Pin analógico de entrada
static constexpr auto PWM_PIN = 32; 
static constexpr auto IN1_PIN = 21;
static constexpr auto IN2_PIN = 25;
// Variables
static double Vref = 1.5f;
static double Vin = 0;          // Lectura de tensión

static double Dref = 500;
static double Din = 0;          // Lectura de tensión
static double filteredDin = 0;  // Tensión suavizada

static double Vo = 0;
static double filteredVin = 0;  // Tensión suavizada
static double alpha = 0.3;      // Factor de suavizado (0 < alpha <= 1)

// Para la dirección del movimiento (si es necesario)
enum Direction {
  FORWARD,
  BACKWARD
};

static Direction direction = Direction::FORWARD;

void controller();
void planta();

// Filtro de paso bajo (low-pass)
double lowPassFilter(double input, double alpha, double* pVal) {
  // Aplicar filtro de paso bajo exponencial
  *pVal = (alpha * input) + ((1.0 - alpha) * *pVal);
  return *pVal;
}

static double Kp = 1, Kd = 100, Ki = 200;
PID controlador(&Din, &Vo, &Dref, Kp, Ki, Kd, DIRECT);

void setup() {
  // Configuración inicial
  Serial.begin(9600);   // Iniciar comunicación serial
  pinMode(ADC_PIN, INPUT);  // Establecer pin de lectura como entrada
  pinMode(PWM_PIN, OUTPUT); // Establecer pin de salida PWM (si lo usas)
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  controlador.SetOutputLimits(-100, 100);
  controlador.SetMode(AUTOMATIC);
  controlador.SetSampleTime(10);
  analogReadResolution(10); // Resolución de 10 bits para el ADC (0-1023)
}

void loop() {
  controller();
  planta();
}

void controller()
{
  Din = analogRead(ADC_PIN);
  filteredDin = lowPassFilter(Din, alpha, &filteredDin);
  // if (filteredVin > 3 || filteredVin < 1)
  //   filteredVin = 2.5;
  Serial.print(" V\nFiltered Value: ");
  Serial.print(filteredDin);
  Serial.print(" V");
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
  analogWrite(PWM_PIN, modVo);
}