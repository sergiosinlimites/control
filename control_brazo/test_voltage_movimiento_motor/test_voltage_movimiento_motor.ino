const int pinPWM = 9;    // Pin PWM de salida (0–255)

void setup() {
  Serial.begin(9600);
  pinMode(pinPWM, OUTPUT);
  Serial.println("Ingrese un valor PWM (0–1023) y presione Enter:");
}

void loop() {
  if (Serial.available() > 0) {
    int inputValue = Serial.parseInt();          // Lee valor entero desde serial
    // Asegura que el valor esté en el rango 0–1023
    inputValue = constrain(inputValue, 0, 1023);

    // Ajusta a rango 0–255 para analogWrite
    int pwmValue = map(inputValue, 0, 1023, 0, 255);
    analogWrite(pinPWM, pwmValue);

    // Mostrar en serial valores recibidos y aplicados
    Serial.print("Valor recibido: ");
    Serial.print(inputValue);
    Serial.print("  -> PWM aplicado (0–255): ");
    Serial.println(pwmValue);

    Serial.println("\nIngrese otro valor PWM (0–1023):");
  }
}
