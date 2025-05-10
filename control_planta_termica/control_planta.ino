// Añadir librerías
#include <OneWire.h>           // (!!!! DIGITAL SENSOR !!!!)
#include <DallasTemperature.h> // (!!!! DIGITAL SENSOR !!!!)

// Definir variables
#define SensorInput_GPIO 8
#define OutputPWM_GPIO 9
#define pwmRes 12
#define pwmMax 4095

// Variables for Conversion
#define Uunits 100 // Set units for Control Output (u) [MAX value of (u) @ MAX pwmDutyCycle]

// Execution Time Control
unsigned long previousMillis = 0;  // For main loop function
long Ts = 1000;                    // Sample time in ms

// Measurement Variables
OneWire oneWire(SensorInput_GPIO);     // (!!!! DIGITAL SENSOR !!!!)
DallasTemperature TempSensor(&oneWire); // (!!!! DIGITAL SENSOR !!!!)
float tempF = 0.0;

// Control System Variables
float Ref = 32.0; // System Reference - Temperature [°C]
float escalones[] = {5, 10, -10, -5, 15, -12, 3, 8};
int numEscalones = sizeof(escalones) / sizeof(escalones[0]);
int escalonActual = 0;
unsigned long tiempoInicioEscalon = 0;
unsigned long duracionEscalon = 26000000/8;         // Duración de cada escalón [ms]
unsigned long tiempoAsentamientoInicial = 240000; // Tiempo de asentamiento [ms]
unsigned long tiempoInicial = 0;

float U_op = 30.0;      // Direct Control Output - FOR OPENLOOP or FEEDFORWARD [mA]
unsigned int pwmDuty = 0; // Control Output (Converted to PWM Duty Cycle)

// Avanzadas (serial)
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// Banderas de estado
bool settled = false;   // Ya pasó asentamiento
bool finished = false;  // Terminó el último escalón

// Prototipos
void setupPWMadj();
void analogWriteADJ(uint8_t pin, uint16_t val);
void recvWithStartEndMarkers();
void parseData();
void calibracion();

void setup() {
  tiempoInicial = millis();
  previousMillis = tiempoInicial - Ts;  // forzar primera llamada inmediata

  Serial.begin(115200);

  // Sensor digital
  TempSensor.begin();
  TempSensor.setResolution(12);

  // PWM
  setupPWMadj();
  analogWriteADJ(OutputPWM_GPIO, pwmDuty);

  delay(5000); // Espera para abrir monitor serial
}

void loop() {
  calibracion();
}

void calibracion() {
  if (finished) return; // ya no hacemos nada

  unsigned long currentMillis = millis();

  // Detectar fin del último escalón:
  if (settled
      && escalonActual == numEscalones - 1
      && currentMillis - tiempoInicioEscalon >= duracionEscalon) {
    finished = true;
    return;
  }

  // Ejecutar cada Ts ms
  if (currentMillis - previousMillis >= Ts) {
    previousMillis = currentMillis;

    // Leer temperatura
    TempSensor.requestTemperatures();
    tempF = TempSensor.getTempCByIndex(0);

    float U_t;

    // Fase de asentamiento
    if (!settled) {
      if (currentMillis - tiempoInicial >= tiempoAsentamientoInicial) {
        // Se cumple el asentamiento
        settled = true;
        tiempoInicioEscalon = currentMillis;
      }
      U_t = U_op;
    }
    // Fase de escalones
    else {
      if (currentMillis - tiempoInicioEscalon >= duracionEscalon
          && escalonActual < numEscalones - 1) {
        escalonActual++;
        tiempoInicioEscalon = currentMillis;
      }
      U_t = U_op + escalones[escalonActual];
    }

    // Saturación y conversión a PWM
    float U_tl = min(max(U_t, 0.0f), float(Uunits));
    pwmDuty = int((U_tl / Uunits) * pwmMax);
    analogWriteADJ(OutputPWM_GPIO, pwmDuty);

    // Salida serial
    Serial.print("U:");
    Serial.print(U_t);
    Serial.print(",tempF:");
    Serial.println(tempF);
  }

  // Entrada serial avanzada
  recvWithStartEndMarkers();
  if (newData) {
    parseData();
    newData = false;
  }
}

/* Configure digital pins 9 y 10 como PWM 12-bit (≈3905 Hz). */
void setupPWMadj() {
  DDRB |= _BV(PB1) | _BV(PB2);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = 0x0fff;
}

/* Versión 12-bit de analogWrite, solo pines 9 y 10. */
void analogWriteADJ(uint8_t pin, uint16_t val){
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}

// Funciones de entrada serial avanzada

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker   = '>';
  char rc;

  while (Serial.available() > 0 && !newData) {
    rc = Serial.read();

    if (recvInProgress) {
      if (rc != endMarker) {
        receivedChars[ndx++] = rc;
        if (ndx >= numChars) ndx = numChars - 1;
      }
      else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseData() {
  Ref = atof(receivedChars);
}
