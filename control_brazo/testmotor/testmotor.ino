const int pinIN3 = 9;  // PWM del Arduino conectado a IN3 del puente H

void setup() {
 pinMode(pinIN3, OUTPUT);
}

void loop() {
  // Gira en un sentido
  analogWrite(pinIN3, 60);  // Velocidad baja
  delay(5000);

  // Parar motor (poniendo PWM en 0)
  analogWrite(pinIN3, 0);
  delay(2000);

  // Gira en sentido opuesto: si IN4 está a GND permanente,
  // este código solo servirá si el puente permite inversión con inversión de PWM.
  // Si no, solo funcionará un sentido.
  analogWrite(pinIN3, 60);  // Velocidad baja nuevamente
  delay(5000);

  analogWrite(pinIN3, 0);
  delay(2000);
}

