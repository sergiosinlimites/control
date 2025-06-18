#include <OneWire.h>           
#include <DallasTemperature.h> 

// Pines y configuraciones
#define SensorInput_GPIO 8
#define OutputPWM_GPIO 9
#define pwmRes 12
#define pwmMax 5
#define Uunits 100.0f   // Máximo valor de la señal de control (mA)

unsigned long previousMillis = 0;
const long Ts = 1000;   // Periodo muestreo en ms (1 segundo)

// Sensor temperatura
OneWire oneWire(SensorInput_GPIO);
DallasTemperature TempSensor(&oneWire);
float tempC = 0.0;

// Control PI discretizado
const float kp = 10.0;
const float ki = 0.238;
const float T = Ts / 1000.0f; // tiempo en segundos (1.0s)

// Variables control
float Ref = 32.0; // referencia temperatura °C
float uk = 0.0;
float uk_1 = 0.0;
float ek = 0.0; // error actual
float ek_1 = 0.0; // error anterior

// PWM
unsigned int pwmDuty = 0;

// Variables serial
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

// Prototipos
void setupPWMadj();
void analogWriteADJ(uint8_t pin, uint16_t val);
void recvWithStartEndMarkers();
void parseData();

void setup() {
  Serial.begin(115200);
  TempSensor.begin();
  TempSensor.setResolution(12);
  setupPWMadj();
  analogWriteADJ(OutputPWM_GPIO, 0); // Inicializamos PWM en 0
  delay(2000); // Espera monitor serial
  previousMillis = millis() - Ts;  // para llamar inmediato loop control
}

void loop() {
  unsigned long currentMillis = millis();

  // Ejecutar control cada Ts ms
  if (currentMillis - previousMillis >= Ts) {
    previousMillis = currentMillis;

    // Leer temperatura actual
    TempSensor.requestTemperatures();
    tempC = TempSensor.getTempCByIndex(0);

    // Error actual
    ek = Ref - tempC;

    // Control PI discretizado por Tusting
    float term1 = (kp + ki * T / 2.0f) * ek;
    float term2 = (ki * T / 2.0f - kp) * ek_1;
    uk = uk_1 + term1 + term2;

    // Saturar la señal
    if (uk > Uunits) uk = Uunits;
    else if (uk < 0.0f) uk = 0.0f;

    // Convertir a PWM (12-bit)
    pwmDuty = (unsigned int)((uk / Uunits) * pwmMax);
    analogWriteADJ(OutputPWM_GPIO, pwmDuty);

    // Guardar variables para siguiente iteración
    uk_1 = uk;
    ek_1 = ek;

    // Enviar datos por Serial para monitoreo
    Serial.print("Time(s):");
    Serial.print(currentMillis / 1000);
    Serial.print(", Ref:");
    Serial.print(Ref, 2);
    Serial.print(", Temp(C):");
    Serial.print(tempC, 2);
    Serial.print(", Error:");
    Serial.print(ek, 3);
    Serial.print(", Control(mA):");
    Serial.print(uk, 3);
    Serial.print(", PWM:");
    Serial.println(pwmDuty);
  }

  // Revisar entrada serial para cambio de referencia
  recvWithStartEndMarkers();
  if (newData) {
    parseData();
    newData = false;
  }
}

void setupPWMadj() {
  DDRB |= _BV(PB1) | _BV(PB2);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = 0x0fff;
}

void analogWriteADJ(uint8_t pin, uint16_t val){
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}

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
        if (ndx < numChars - 1) {
          receivedChars[ndx++] = rc;
        }
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
      ndx = 0;
    }
  }
}

void parseData() {
  float newRef = atof(receivedChars);
  if (newRef >= 0 && newRef <= 50) { // Validar rango razonable
    Ref = newRef;
    Serial.print("Nueva referencia: ");
    Serial.println(Ref);
  }
}
