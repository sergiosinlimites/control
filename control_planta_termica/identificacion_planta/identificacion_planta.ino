// Código para enviar un valor PWM fijo (0–1023) en el pin 9
// Cambia `desiredPWM` al valor que necesites y vuelve a cargar el programa.

const int pinPWM = 9;               // Pin PWM de salida
const int desiredPWM = 40;         // Valor PWM deseado (0–1023)

void setup() {
  Serial.begin(9600);
  pinMode(pinPWM, OUTPUT);

  // Ajusta resolución PWM a 10 bits en placas compatibles (e.g., Due)
  #if defined(analogWriteResolution)
    analogWriteResolution(10);
  #endif

  // Envía el PWM una sola vez
  int pwm8 = map(desiredPWM, 0, 1023, 0, 255);
  analogWrite(pinPWM, pwm8);

  // Informa por Serial
  Serial.print("PWM enviado (0–1023): ");
  Serial.println(desiredPWM);
}

void loop() {
  // No se requiere hacer nada en loop; el PWM se mantiene constante
}
