int lectura;
float voltaje;
float angulo;

void setup() {
Serial.begin(115200);
}
void loop() {
float lecturavf;
lectura = analogRead(A0); // Lectura entre 0 y 1023
lecturavf = (lectura - 217);
voltaje = (lecturavf * 5.0) / 1023.0; // Conversión a voltaje
angulo = (lecturavf * 360.0) / 1023.0; // Conversión a grados
Serial.print("Ángulo: ");
Serial.println(angulo);
//Serial.print("Corregido: ");
//Serial.println(lecturavf);
delay(100);
}